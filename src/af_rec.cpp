// src/af_rec.cpp    
// ============================================================================  
// af_rec.cpp — цифровой магнитофон: 4 слота во flash по 224 КБ    
//        
// Запись: af_rec_push (Core 1) кодирует IMA-ADPCM на лету — каждый        
// AF_REC_DECIM-й отсчёт (децимация ×2: 15464 -> 7732 Гц) -> ниббл        
// в ram_buf (224000 отсчётов = 112 КБ, ~29 с).        
// На стопе буфер сбрасывается во flash-слот активного банка через        
// flash-окно (park Core1 -> mute -> 125 МГц -> capture pause -> pwm off        
// -> IRQ-off -> erase+program -> restore), как в af_store_save.        
// Слот 224 КБ вмещает ~29.7 с ADPCM (229376 отсчётов).        
// Формат слота ver 0x02: codec 0=PCM16 (legacy/voice_14), 1=IMA-ADPCM.        
// Воспроизведение: чтение слота напрямую из XIP с интерполяцией ×2.    
// Пустой слот -> клип-заглушка voice_14 (1:1, без интерполяции).    
// Активный слот = код селектора GP13/GP28 + 1 (af_gpio_msg_sel, 1..4).        
// Запись GP9 и воспроизведение GP10 идут в слот по текущему коду.        
// Пины: GP9 = запись (тракт -> TX, микрофон, без PTT),      
//       GP10 = воспроизведение (тракт -> TX + PTT на GP8, звук в TX).      
// ============================================================================    
#include <Arduino.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
#include "af_gpio_ctl.h"       // af_gpio_msg_sel — активный слот 1..4   
#include "af_store.h"          // park/unpark, af_set_clk*, af_store_write_all    
#include "af_capture.h"        // af_capture_pause/resume    
#include "af_pwm.h"            // af_pwm_set_enabled/is_enabled    
#include "af_usb.h"            // af_usb_set_muted    
#include "pico/stdlib.h"  
#include "hardware/flash.h"    // flash_range_erase/program    
#include "hardware/sync.h"     // save_and_disable_interrupts    
#include "voice/voice_14.inc"  // клип-заглушка: играется при пустом слоте
#include "af_fm.h"             // af_fm_request_sync — обновить RECn.WAV на диске

typedef enum { ST_IDLE = 0, ST_REC, ST_PLAY } rec_state_t;    
  
static volatile rec_state_t state = ST_IDLE;  
volatile uint8_t af_rec_active = 0;   // 0=idle, 1=rec, 2=play — для af_dsp_poll  

static volatile uint32_t    rec_len = 0;             // валидных отсчётов в слоте    
static volatile uint32_t    pos     = 0;             // позиция записи/чтения    

static volatile uint8_t     rec_src_tx = 0;  
static volatile uint32_t    decim_phase = 0;   // фаза дециматора (запись)  
static volatile uint32_t    interp_phase = 0;  // фаза интерполятора (play)  
static int16_t              prev_samp = 0;     // предыдущий отсчёт для интерполяции  
static int16_t              cur_samp  = 0;     // декодированный отсчёт на позиции pos (prefetch)  
static volatile uint8_t     play_src = 0;      // 0 = ram_buf (×2 interp), 1 = voice_14 (1:1)    
static volatile uint32_t    play_len = 0;      // длина текущего воспроизведения

static bool fill_done = false;

// --- RAM-буфер записи --------------------------------------------------------      
// ADPCM-поток: 2 отсчёта/байт (нибблы, первый отсчёт в младшем).      
// +256 Б — запас под паддинг хвоста до границы страницы в rec_slot_write:    
// flash_range_program требует длину кратную 256 Б.    
// Аплоад WAV (af_fm) использует буфер как int16-стейджинг PCM,    
// при коммите кодируется in-place (выход вдвое короче входа).    
static uint8_t  ram_buf[AF_REC_RAM_BYTES + 256];    
    
// --- IMA-ADPCM (DVI, WAV tag 0x11): таблицы и состояние ----------------------    
// Кодер/декодер — в RAM: вызываются из горячего пути Core 1.    
static const int16_t adpcm_step[89] = {    
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,    
    50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,    
    337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,    
    2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,    
    15289,16818,18500,20350,22385,24623,27086,29794,32767    
};    
static const int8_t adpcm_idx[16] = { -1,-1,-1,-1, 2,4,6,8, -1,-1,-1,-1, 2,4,6,8 };    
    
typedef struct { int16_t pred; int8_t idx; } adpcm_st_t;    
static adpcm_st_t enc = {0, 0};    // состояние кодера (запись)    
static adpcm_st_t dec = {0, 0};    // состояние декодера (play)    
    
// Кодирование одного отсчёта -> ниббл.    
static inline uint8_t __not_in_flash_func(adpcm_enc)(int16_t s) { 
    int32_t step = adpcm_step[enc.idx];    
    int32_t diff = (int32_t)s - enc.pred;    
    uint8_t nib = 0;    
    if (diff < 0) { nib = 8; diff = -diff; }    
    int32_t delta = step >> 3;    
    if (diff >= step)       { nib |= 4; diff -= step;       delta += step; }    
    if (diff >= (step>>1))  { nib |= 2; diff -= (step>>1);  delta += (step>>1); }    
    if (diff >= (step>>2))  { nib |= 1;                     delta += (step>>2); }    
    enc.pred += (nib & 8) ? -delta : delta;    
    if (enc.pred > 32767) enc.pred = 32767; else if (enc.pred < -32768) enc.pred = -32768;    
    enc.idx += adpcm_idx[nib];    
    if (enc.idx < 0) enc.idx = 0; else if (enc.idx > 88) enc.idx = 88;    
    return nib;    
}    
    
// Декодирование ниббла -> отсчёт.    
static inline int16_t __not_in_flash_func(adpcm_dec)(uint8_t nib) {    
    int32_t step = adpcm_step[dec.idx];    
    int32_t delta = step >> 3;    
    if (nib & 4) delta += step;    
    if (nib & 2) delta += (step>>1);    
    if (nib & 1) delta += (step>>2);    
    dec.pred += (nib & 8) ? -delta : delta;    
    if (dec.pred > 32767) dec.pred = 32767; else if (dec.pred < -32768) dec.pred = -32768;    
    dec.idx += adpcm_idx[nib & 15];    
    if (dec.idx < 0) dec.idx = 0; else if (dec.idx > 88) dec.idx = 88;    
    return dec.pred;    
}    
    
// Чтение ниббла pos из ADPCM-потока (RAM-буфер или XIP-слот — одинаково).    
static inline uint8_t __not_in_flash_func(adpcm_get_nib)(const uint8_t* p, uint32_t i) {    
    uint8_t b = p[i >> 1];    
    return (i & 1) ? (b >> 4) : (b & 0x0F);    
}    
  
// --- Слот во flash ------------------------------------------------------------      
// Заголовок слота (первая страница, AF_REC_HDR_SIZE из af_config.h):        
// magic(2) + ver + bank + len(4) + crc + codec. Данные с offset AF_REC_HDR_SIZE.    
// Пустой слот = всё 0xFF. codec: AF_REC_CODEC_PCM / AF_REC_CODEC_ADPCM.    
#define AF_REC_MAGIC0     0x52u   // 'R'        
#define AF_REC_MAGIC1     0x43u   // 'C'        
#define AF_REC_VER        0x02u   // ver 2: +поле codec (offset 9)      
    
static uint32_t slot_len   = 0;   // отсчётов в слоте активного банка        
static uint32_t slot_off   = 0;   // flash-offset активного слота        
static uint8_t  slot_codec = AF_REC_CODEC_PCM;  // codec активного слота      
    
// Активный слот = код селектора GP13/GP28 + 1 (1..4).          
static uint8_t rec_bank(void) {        
    return af_gpio_msg_sel() + 1;        
}        
  
static uint32_t rec_slot_off(uint8_t bank) {          // bank = 1..4      
    return AF_REC_FLASH_OFF + (uint32_t)(bank - 1) * AF_REC_SLOT_SIZE;    
}    
  
// Читает заголовок слота bank из XIP; возвращает длину в отсчётах (0 = пусто).      
// codec слота пишется в *codec_out (если не NULL).    
static uint32_t rec_slot_len_codec(uint8_t bank, uint8_t* codec_out) {        
    const uint8_t* h = (const uint8_t*)(XIP_BASE + rec_slot_off(bank));        
    if (h[0] != AF_REC_MAGIC0 || h[1] != AF_REC_MAGIC1)        
        return 0;        
    if (h[2] != 0x01u && h[2] != AF_REC_VER)   // ver 1 = legacy PCM, ver 2 = +codec  
        return 0;        
    uint32_t len;        
    memcpy(&len, h + 4, 4);        
    uint8_t crc = 0;        
    for (int i = 0; i < 8; i++) crc ^= h[i];        
    if (crc != h[8]) return 0;        
    uint8_t codec = (h[2] == 0x01u) ? AF_REC_CODEC_PCM : h[9];  // ver1: codec нет  
    if (codec != AF_REC_CODEC_PCM && codec != AF_REC_CODEC_ADPCM) return 0;      
    uint32_t cap = (codec == AF_REC_CODEC_ADPCM) ? AF_REC_SLOT_SAMP      
                                               : AF_REC_SLOT_SAMP / 2u;      
    if (len > cap) return 0;   // лимит = ёмкость слота под данный codec      
    if (codec_out) *codec_out = codec;      
    return len;          
}        
    
static uint32_t rec_slot_len(uint8_t bank) {      
    return rec_slot_len_codec(bank, NULL);      
}      
    
// Указатель на данные слота в XIP (PCM16 или ADPCM-нибблы — по codec).      
static const uint8_t* rec_slot_data(void) {      
    return (const uint8_t*)(XIP_BASE + slot_off + AF_REC_HDR_SIZE);      
}      

// --- Экспорт слотов для af_fm (WAV-файлы на диске) ---------------------------      
uint32_t af_rec_slot_len(uint8_t bank) {      
    if (bank < 1 || bank > AF_REC_SLOTS) return 0;         // / nullptr / false  
    return rec_slot_len(bank);      
}      
      
const int16_t* af_rec_slot_data(uint8_t bank) {          
    if (bank < 1 || bank > AF_REC_SLOTS) return nullptr;          
    return (const int16_t*)(XIP_BASE + rec_slot_off(bank) + AF_REC_HDR_SIZE);        
}        
    
// codec слота: AF_REC_CODEC_PCM / AF_REC_CODEC_ADPCM (0 = PCM и для пустого).    
uint8_t af_rec_slot_codec(uint8_t bank) {          
    if (bank < 1 || bank > AF_REC_SLOTS) return AF_REC_CODEC_PCM;          
    uint8_t c = AF_REC_CODEC_PCM;    
    rec_slot_len_codec(bank, &c);    
    return c;    
}        
    
// --- Экспорт для аплоада WAV (af_fm): запись в ram_buf + коммит в слот -------        
// Буфер отдаётся как int16-стейджинг PCM; при коммите af_fm кодирует    
// PCM -> ADPCM in-place через af_rec_adpcm_encode_buf.    
int16_t* af_rec_ram_buf(void) {        
    return (int16_t*)ram_buf;        
}        
    
// In-place кодирование n_samp отсчётов PCM16 (в ram_buf как int16*)    
// в ADPCM-нибблы (в начало ram_buf). Выход вдвое короче входа — безопасно.    
// Вызывается из af_fm при коммите аплоада (Core 0, не горячий путь).    
void af_rec_adpcm_encode_buf(uint32_t n_samp) {      
    if (n_samp > AF_REC_RAM_BYTES / 2u)     // PCM-стейджинг: 2 Б/отсчёт    
        n_samp = AF_REC_RAM_BYTES / 2u;    
    adpcm_st_t e = {0, 0};      
    const int16_t* in = (const int16_t*)ram_buf;      
    for (uint32_t i = 0; i < n_samp; i++) {      
        // локальная копия enc — не трогаем состояние записи с АЦП    
        int32_t step = adpcm_step[e.idx];    
        int32_t diff = (int32_t)in[i] - e.pred;    
        uint8_t nib = 0;    
        if (diff < 0) { nib = 8; diff = -diff; }    
        int32_t delta = step >> 3;    
        if (diff >= step)      { nib |= 4; diff -= step;      delta += step; }    
        if (diff >= (step>>1)) { nib |= 2; diff -= (step>>1); delta += (step>>1); }    
        if (diff >= (step>>2)) { nib |= 1;                    delta += (step>>2); }    
        e.pred += (nib & 8) ? -delta : delta;    
        if (e.pred > 32767) e.pred = 32767; else if (e.pred < -32768) e.pred = -32768;    
        e.idx += adpcm_idx[nib];    
        if (e.idx < 0) e.idx = 0; else if (e.idx > 88) e.idx = 88;    
        uint8_t pair = (i & 1) ? (ram_buf[i >> 1] | (nib << 4)) : nib;    
        ram_buf[i >> 1] = pair;    
    }    
}    
      
bool af_rec_is_busy(void) {      
    return state != ST_IDLE;      
}      

static bool rec_slot_write(uint8_t bank, uint32_t n_samp, uint8_t codec);  // ниже    

// Коммит ram_buf -> слот bank (flash-окно ~2.5-3 с, мьют тракта).        
// Вызывается из af_fm_poll по таймауту после последнего сектора WAV.    
// codec = AF_REC_CODEC_ADPCM (ram_buf = нибблы) или AF_REC_CODEC_PCM.    
bool af_rec_slot_write(uint8_t bank, uint32_t n_samp, uint8_t codec) {          
    if (state != ST_IDLE) return false;          
    if (bank < 1 || bank > AF_REC_SLOTS) return false;            
    // Лимит = ram_buf (коммит всегда из него), а не ёмкость слота:      
    // ADPCM — AF_REC_RAM_SAMP отсчётов, PCM — AF_REC_RAM_BYTES/2.      
    uint32_t cap = (codec == AF_REC_CODEC_ADPCM) ? AF_REC_RAM_SAMP      
                                               : AF_REC_RAM_BYTES / 2u;      
    if (n_samp == 0 || n_samp > cap) return false;            
    if (!rec_slot_write(bank, n_samp, codec)) return false;        
    if (bank == rec_bank()) {           // активный слот — обновить кэш        
        slot_off = rec_slot_off(bank);        
        slot_len = n_samp;        
        slot_codec = codec;    
    }        
    return true;        
}        

// --- Flash-окно: erase+program слота. Вся функция в RAM. ----------------------    
// param = {off, src, bytes}: erase AF_REC_SLOT_SIZE, затем program src.    
struct rec_wr_t { uint32_t off; const uint8_t* src; uint32_t bytes; };    
  
static void __not_in_flash_func(do_rec_flash_write)(void* param) {      
    const rec_wr_t* w = (const rec_wr_t*)param;      
    flash_range_erase(w->off, AF_REC_SLOT_SIZE);      
    flash_range_program(w->off, w->src, AF_REC_HDR_SIZE);              // заголовок      
    flash_range_program(w->off + AF_REC_HDR_SIZE,      
                        (const uint8_t*)ram_buf, w->bytes);            // PCM      
}       
  
// Сброс ram_buf в слот bank. Вызывается из af_rec_stop (Core 0).      
// Окно ~2.5-3 с (erase 56 секторов) — звук/USB мьютятся, как при save.
// Пишем двумя program: заголовок 256 Б, затем ram_buf напрямую      
// (он в RAM — источник для flash_range_program валиден при XIP off).      
static bool rec_slot_write(uint8_t bank, uint32_t n_samp, uint8_t codec) {        
    static uint8_t hdr[AF_REC_HDR_SIZE];        
    memset(hdr, 0xFF, sizeof(hdr));        
    hdr[0] = AF_REC_MAGIC0; hdr[1] = AF_REC_MAGIC1; hdr[2] = AF_REC_VER;        
    hdr[3] = bank;        
    memcpy(hdr + 4, &n_samp, 4);        
    uint8_t crc = 0;        
    for (int i = 0; i < 8; i++) crc ^= hdr[i];        
    hdr[8] = crc;        
    hdr[9] = codec;                 // ver 0x02: поле codec (в crc не входит)    
      
    // Байт данных: PCM16 = 2 Б/отсчёт, ADPCM = 0.5 Б/отсчёт.    
    // Паддинг хвоста ram_buf до границы 256 Б — flash_range_program        
    // требует длину кратную странице.        
    uint32_t raw = (codec == AF_REC_CODEC_ADPCM) ? (n_samp + 1u) / 2u    
                                               : n_samp * 2u;    
    uint32_t data_bytes = (raw + 255u) & ~255u;        
    memset(ram_buf + raw, 0xFF, data_bytes - raw);        
    
    rec_wr_t w = { rec_slot_off(bank), hdr, data_bytes };      
  
    if (!af_store_core1_park(5000)) {    
        Serial.println("# rec: park TIMEOUT — flash write aborted");    
        return false;    
    }    
    af_usb_set_muted(true);    
    af_set_clk_nominal();                 // 125 МГц на окно — обязательно    
    af_capture_pause();    
    const bool pwm_was = af_pwm_is_enabled();    
    if (pwm_was) af_pwm_set_enabled(false);    
    uint32_t ints = save_and_disable_interrupts();    
    do_rec_flash_write(&w);               // RAM: erase+program, XIP off    
    flash_flush_cache();                  // до restore_interrupts — иначе вис    
    restore_interrupts(ints);    
    af_capture_resume();    
    af_usb_set_muted(false);    
    af_set_clk();                         // возврат на рабочий clk_sys    
    if (pwm_was) af_pwm_set_enabled(true);    
    af_store_core1_unpark();    
    return true;    
}

// Очистка всех слотов: erase всего региона магнитофона одним окном.        
// Вызывается из обработчика "banks clr" (af_cli.cpp, Core 0).        
// Окно ~3-4 с (erase 224 секторов) — звук/USB мьютятся, как при save.        
static void __not_in_flash_func(do_rec_flash_erase_all)(void* param) {        
    (void)param;        
    flash_range_erase(AF_REC_FLASH_OFF, AF_REC_FLASH_LEN);        
}        
    
bool af_rec_erase_all(void) {        
    if (state != ST_IDLE) return false;         // не во время rec/play        
    
    if (!af_store_core1_park(5000)) {        
        Serial.println("# rec: park TIMEOUT — erase aborted");        
        return false;        
    }        
    af_usb_set_muted(true);        
    af_set_clk_nominal();        
    af_capture_pause();        
    const bool pwm_was = af_pwm_is_enabled();        
    if (pwm_was) af_pwm_set_enabled(false);        
    uint32_t ints = save_and_disable_interrupts();        
    do_rec_flash_erase_all(NULL);        
    flash_flush_cache();        
    restore_interrupts(ints);        
    af_capture_resume();        
    af_usb_set_muted(false);        
    af_set_clk();        
    if (pwm_was) af_pwm_set_enabled(true);        
    af_store_core1_unpark();        
    
    slot_len = 0;                               // активный слот тоже пуст          
    Serial.println("# rec: all slots erased");          
    af_fm_request_sync();   // все RECn.WAV стали пустыми — перерендерить  
    return true;          
}           
  
// --- API --------------------------------------------------------------------      
// Заполнение пустых слотов клипом voice_14 при старте.  
// Клип 15464 Гц -> децимация ×2 в ram_buf (7732 Гц, ~42223 отсч., ~5.5 с).  
// rec_slot_write сам делает flash-окно (park/mute/125 МГц/IRQ-off).  
static void rec_fill_empty_slots(void) {    
    uint32_t n = clip_voice_14_len / AF_REC_DECIM;      // ~42223 отсч.    
    if (n > AF_REC_RAM_SAMP) n = AF_REC_RAM_SAMP;    
    bool filled = false;    
    for (uint8_t b = 1; b <= AF_REC_SLOTS; b++) {        
        if (rec_slot_len(b) != 0) continue;             // слот занят      
        // децимация ×2 + ADPCM-кодирование в ram_buf (нибблы)    
        enc.pred = 0; enc.idx = 0;    
        for (uint32_t i = 0; i < n; i++) {    
            uint8_t nib = adpcm_enc(clip_voice_14[i * AF_REC_DECIM]);    
            if (i & 1) ram_buf[i >> 1] |= (nib << 4);    
            else       ram_buf[i >> 1] = nib;    
        }    
        Serial.printf("# rec: slot %u empty -> fill voice14 (%lu samp adpcm)\n",      
                      (unsigned)b, (unsigned long)n);      
        rec_slot_write(b, n, AF_REC_CODEC_ADPCM);       // flash-окно внутри      
        filled = true;    
    }      
    if (filled) {  
        af_fm_request_sync();    
        af_usb_reenumerate();   // хост перечитает каталог — файлы появились  
    }  
}  

void af_rec_init(void) {
    fill_done = false;  
    rec_len = 0;
    slot_off = rec_slot_off(rec_bank());      
    slot_len = rec_slot_len_codec(rec_bank(), &slot_codec);      
    Serial.printf("# rec: slot bank=%u off=0x%lx len=%lu samp\n",    
                  (unsigned)rec_bank(), (unsigned long)slot_off,    
                  (unsigned long)slot_len);    
}    
  
bool af_rec_start(void) {        
    if (state != ST_IDLE) return false;        
    rec_len = 0;              
    pos = 0;        
    decim_phase = 0;      
    enc.pred = 0; enc.idx = 0;          // сброс ADPCM-кодера    
    rec_src_tx = af_dsp_tx_on() ? 1 : 0;          
    state = ST_REC;  
    af_rec_active = 1;  
    Serial.printf("# rec: recording (%s) bank=%u, ram buf %lu samp\n",    
                  rec_src_tx ? "tx" : "rx", (unsigned)rec_bank(),  
                  (unsigned long)AF_REC_RAM_SAMP);        
    return true;        
}        
  
void af_rec_stop(void) {              
    if (state == ST_PLAY) { af_rec_play_stop(); return; }   // stop универсальный          
    if (state != ST_REC) return;              
    state = ST_IDLE;  
    af_rec_active = 0;  
    rec_len = pos;            
    if (rec_len == 0) { Serial.println("# rec: nothing recorded"); return; }            
    Serial.printf("# rec: %lu samp (%lu.%lu s) -> flash bank=%u\n",    
                  (unsigned long)rec_len,    
                  (unsigned long)(rec_len * AF_REC_DECIM / AF_FS_OUT_HZ),    
                  (unsigned long)((rec_len * AF_REC_DECIM % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ),  
                  (unsigned)rec_bank());    
    uint8_t b = rec_bank();    
    if (rec_slot_write(b, rec_len, AF_REC_CODEC_ADPCM)) {        
        slot_off = rec_slot_off(b);        
        slot_len = rec_len;        
        slot_codec = AF_REC_CODEC_ADPCM;    
        Serial.println("# rec: slot written");      
        af_fm_request_sync();   // размер RECn.WAV изменился — перерендерить dir/FAT  
    } else {      
        Serial.println("# rec: flash write FAILED — запись только в RAM");    
    }    
}    
  
bool af_rec_play(void) {        
    if (state != ST_IDLE) return false;        
    pos = 0;      
    interp_phase = 0;      
    prev_samp = 0;      
    slot_off = rec_slot_off(rec_bank());    // слот активного банка        
    slot_len = rec_slot_len_codec(rec_bank(), &slot_codec);        
    dec.pred = 0; dec.idx = 0;              // сброс ADPCM-декодера      
    if (slot_len == 0) {                    // слот пуст — клип-заглушка        
        play_src = 1;        
        play_len = clip_voice_14_len;        
        Serial.printf("# rec: play %lu samp (voice14, bank=%u empty)\n",    
                      (unsigned long)play_len, (unsigned)rec_bank());        
    } else {        
        play_src = 0;        
        play_len = slot_len;        
        Serial.printf("# rec: play %lu samp (flash bank=%u)\n",    
                      (unsigned long)play_len, (unsigned)rec_bank());        
        // Prefetch: декодируем отсчёт pos=0 заранее — adpcm_dec stateful,    
        // в горячем пути декодируем ровно 1 раз на отсчёт (при pos++).    
        cur_samp = (slot_codec == AF_REC_CODEC_ADPCM)    
                 ? adpcm_dec(adpcm_get_nib(rec_slot_data(), 0))    
                 : ((const int16_t*)rec_slot_data())[0];    
    }        
    state = ST_PLAY;      
    af_rec_active = 2;    
    return true;        
}        
  
void af_rec_play_stop(void) {    
    if (state == ST_PLAY) {
        state = ST_IDLE;
        af_rec_active = 0;
        Serial.println("# rec: play stop"); 
    }
}    
  
// Геттеры — в RAM: вызываются из горячего пути Core 1.
bool     __not_in_flash_func(af_rec_is_recording)(void)  { return state == ST_REC; }      
bool     __not_in_flash_func(af_rec_is_playing)(void)    { return state == ST_PLAY; }      
uint32_t __not_in_flash_func(af_rec_len_samples)(void)   { return rec_len; }      
uint32_t __not_in_flash_func(af_rec_play_len)(void)      { return play_src ? (uint32_t)clip_voice_14_len : slot_len; }    
uint32_t __not_in_flash_func(af_rec_pos_samples)(void)   { return pos; }      
  
// --- Core 1, горячий путь (RAM) ----------------------------------------------    
void __not_in_flash_func(af_rec_push)(int16_t s) {  
    if (state != ST_REC) return;  
    if (pos >= AF_REC_RAM_SAMP) return;  
    if (++decim_phase >= AF_REC_DECIM) {   // каждый DECIM-й отсчёт  
        decim_phase = 0;  
        uint8_t nib = adpcm_enc(s);    
        uint32_t i = pos++;    
        if (i & 1) ram_buf[i >> 1] |= (nib << 4);    
        else       ram_buf[i >> 1] = nib;
    }  
}  
  
int16_t __not_in_flash_func(af_rec_mix)(int16_t s) {      
    if (state != ST_PLAY) return s;      
    if (pos >= play_len) { state = ST_IDLE; af_rec_active = 0; return s; }      
    int32_t v;    
    if (play_src) {                          // voice_14: полная частота, 1:1    
        v = s + clip_voice_14[pos++];    
    } else {                                 // flash-слот: интерполяция ×2    
        // adpcm_dec stateful — декодируем ровно 1 раз на отсчёт (prefetch    
        // в cur_samp), а не на каждый выходной сэмпл интерполятора.    
        if (++interp_phase >= AF_REC_DECIM) {    
            interp_phase = 0;    
            v = s + cur_samp;    
            prev_samp = cur_samp;    
            pos++;    
            if (pos < play_len) {    
                cur_samp = (slot_codec == AF_REC_CODEC_ADPCM)    
                         ? adpcm_dec(adpcm_get_nib(rec_slot_data(), pos))    
                         : ((const int16_t*)rec_slot_data())[pos];    
            }    
        } else {    
            v = s + (prev_samp + cur_samp) / 2;    
        }    
    }        
    if (v > 32767) v = 32767; else if (v < -32768) v = -32768;      
    return (int16_t)v;      
}        
  
int32_t __not_in_flash_func(af_rec_src)(int32_t mic) {      
    if (state != ST_PLAY) return mic;      
    if (pos >= play_len) { state = ST_IDLE; af_rec_active = 0; return mic; }      
    int32_t v;    
    if (play_src) {                          // voice_14: полная частота, 1:1    
        v = clip_voice_14[pos++];    
    } else {                                 // flash-слот: интерполяция ×2    
        // adpcm_dec stateful — декодируем ровно 1 раз на отсчёт (prefetch    
        // в cur_samp), а не на каждый выходной сэмпл интерполятора.    
        if (++interp_phase >= AF_REC_DECIM) {    
            interp_phase = 0;    
            v = cur_samp;    
            prev_samp = cur_samp;    
            pos++;    
            if (pos < play_len) {    
                cur_samp = (slot_codec == AF_REC_CODEC_ADPCM)    
                         ? adpcm_dec(adpcm_get_nib(rec_slot_data(), pos))    
                         : ((const int16_t*)rec_slot_data())[pos];    
            }    
        } else {    
            v = (prev_samp + cur_samp) / 2;    
        }    
    }        
    return v;                            // замещение микрофона    
}      
  
// --- Core 0: контроль записи (смена тракта, переполнение буфера) --------------  
void af_rec_poll(void) {
    if (!fill_done) {                 // первый poll — Core1 уже работает  
        fill_done = true;  
        rec_fill_empty_slots();  
    }  
    if (state != ST_REC) return;          
    if (pos >= AF_REC_RAM_SAMP) {           // RAM-буфер полон — стоп  
        Serial.println("# rec: ram buf full — stop");    
        af_rec_stop();    
    }    
}    
