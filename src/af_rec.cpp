// src/af_rec.cpp    
// ============================================================================  
// af_rec.cpp — цифровой магнитофон: 7 слотов во flash, по одному на банк.    
//    
// Запись: af_rec_push (Core 1) пишет в ram_buf каждый AF_REC_DECIM-й    
// отсчёт (децимация ×2: 15464 -> 7732 Гц, ~7.2 с в 56000 отсчётов).    
// На стопе буфер сбрасывается во flash-слот активного банка через    
// flash-окно (park Core1 -> mute -> 125 МГц -> capture pause -> pwm off    
// -> IRQ-off -> erase+program -> restore), как в af_store_save.    
// Воспроизведение: чтение слота напрямую из XIP с интерполяцией ×2.    
// Пустой слот -> клип-заглушка voice_14 (1:1, без интерполяции).    
// Активный слот = последний загруженный банк (af_gpio_last_bank):      
// обновляется при каждом apply_bank/'load N', в manual-режиме      
// (пины отпущены, код 0) сохраняет последнее значение.      
// Пины: GP9 = запись (тракт -> TX, микрофон, без PTT),      
//       GP10 = воспроизведение (тракт -> TX + PTT на GP8, звук в TX).      
// ============================================================================    
#include <Arduino.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
#include "af_gpio_ctl.h"       // af_gpio_last_bank — активный слот      
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
static volatile uint8_t     play_src = 0;      // 0 = ram_buf (×2 interp), 1 = voice_14 (1:1)    
static volatile uint32_t    play_len = 0;      // длина текущего воспроизведения

static bool fill_done = false;

// --- RAM-буфер записи --------------------------------------------------------    
// +128 отсчётов (256 Б) — запас под паддинг хвоста до границы страницы  
// в rec_slot_write: flash_range_program требует длину кратную 256 Б.  
static int16_t  ram_buf[AF_REC_RAM_SAMP + 128];  
  
// --- Слот во flash ------------------------------------------------------------    
// Заголовок слота (первая страница, 256 Б): magic + длина + банк + crc.    
// Данные PCM16 с offset AF_REC_HDR_SIZE. Пустой слот = всё 0xFF.    
#define AF_REC_HDR_SIZE   256u    
#define AF_REC_MAGIC0     0x52u   // 'R'    
#define AF_REC_MAGIC1     0x43u   // 'C'    
#define AF_REC_VER        0x01u    
  
static uint32_t slot_len  = 0;    // отсчётов в слоте активного банка      
static uint32_t slot_off  = 0;    // flash-offset активного слота      
    
// Активный банк: последний загруженный (af_gpio_ctl видит каждое      
// переключение — пины и CLI 'load N'). В manual-режиме (код 0)      
// возвращает последний активный — локальный last_bank не нужен.      
static uint8_t rec_bank(void) {      
    return af_gpio_last_bank();      
}      
  
static uint32_t rec_slot_off(uint8_t bank) {          // bank = 1..7    
    return AF_REC_FLASH_OFF + (uint32_t)(bank - 1) * AF_REC_SLOT_SIZE;    
}    
  
// Читает заголовок слота bank из XIP; возвращает длину в отсчётах (0 = пусто).    
static uint32_t rec_slot_len(uint8_t bank) {    
    const uint8_t* h = (const uint8_t*)(XIP_BASE + rec_slot_off(bank));    
    if (h[0] != AF_REC_MAGIC0 || h[1] != AF_REC_MAGIC1 || h[2] != AF_REC_VER)    
        return 0;    
    uint32_t len;    
    memcpy(&len, h + 4, 4);    
    uint8_t crc = 0;    
    for (int i = 0; i < 8; i++) crc ^= h[i];    
    if (crc != h[8]) return 0;    
    if (len > AF_REC_RAM_SAMP) return 0;    
    return len;    
}    
  
// Указатель на PCM-данные слота в XIP (для воспроизведения).    
static const int16_t* rec_slot_data(void) {    
    return (const int16_t*)(XIP_BASE + slot_off + AF_REC_HDR_SIZE);    
}    

// --- Экспорт слотов для af_fm (WAV-файлы на диске) ---------------------------      
uint32_t af_rec_slot_len(uint8_t bank) {      
    if (bank < 1 || bank > AF_STORE_BANKS) return 0;      
    return rec_slot_len(bank);      
}      
      
const int16_t* af_rec_slot_data(uint8_t bank) {      
    if (bank < 1 || bank > AF_STORE_BANKS) return nullptr;      
    return (const int16_t*)(XIP_BASE + rec_slot_off(bank) + AF_REC_HDR_SIZE);      
}      
  
// --- Экспорт для аплоада WAV (af_fm): запись в ram_buf + коммит в слот -------      
int16_t* af_rec_ram_buf(void) {      
    return ram_buf;      
}      
      
bool af_rec_is_busy(void) {      
    return state != ST_IDLE;      
}      

static bool rec_slot_write(uint8_t bank, uint32_t n_samp);  // определена ниже  

// Коммит ram_buf -> слот bank (flash-окно ~1.5-2 с, мьют тракта).      
// Вызывается из af_fm_poll по таймауту после последнего сектора WAV.      
bool af_rec_slot_write(uint8_t bank, uint32_t n_samp) {      
    if (state != ST_IDLE) return false;      
    if (bank < 1 || bank > AF_STORE_BANKS) return false;      
    if (n_samp == 0 || n_samp > AF_REC_RAM_SAMP) return false;      
    if (!rec_slot_write(bank, n_samp)) return false;      
    if (bank == rec_bank()) {           // активный слот — обновить кэш      
        slot_off = rec_slot_off(bank);      
        slot_len = n_samp;      
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
// Окно ~1.5-2 с (erase 32 секторов) — звук/USB мьютятся, как при save.      
// Пишем двумя program: заголовок 256 Б, затем ram_buf напрямую      
// (он в RAM — источник для flash_range_program валиден при XIP off).      
static bool rec_slot_write(uint8_t bank, uint32_t n_samp) {      
    static uint8_t hdr[AF_REC_HDR_SIZE];      
    memset(hdr, 0xFF, sizeof(hdr));      
    hdr[0] = AF_REC_MAGIC0; hdr[1] = AF_REC_MAGIC1; hdr[2] = AF_REC_VER;      
    hdr[3] = bank;      
    memcpy(hdr + 4, &n_samp, 4);      
    uint8_t crc = 0;      
    for (int i = 0; i < 8; i++) crc ^= hdr[i];      
    hdr[8] = crc;      
    
    // Паддинг хвоста ram_buf до границы 256 Б — flash_range_program      
    // требует длину кратную странице.      
    uint32_t data_bytes = (n_samp * 2 + 255u) & ~255u;      
    uint8_t* tail = (uint8_t*)ram_buf + n_samp * 2;      
    memset(tail, 0xFF, data_bytes - n_samp * 2);      
    
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
    for (uint8_t b = 1; b <= AF_STORE_BANKS; b++) {    
        if (rec_slot_len(b) != 0) continue;             // слот занят    
        // децимация ×2 в ram_buf    
        for (uint32_t i = 0; i < n; i++)    
            ram_buf[i] = clip_voice_14[i * AF_REC_DECIM];    
        Serial.printf("# rec: slot %u empty -> fill voice14 (%lu samp)\n",    
                      (unsigned)b, (unsigned long)n);    
        rec_slot_write(b, n);                           // flash-окно внутри    
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
    slot_len = rec_slot_len(rec_bank());    
    Serial.printf("# rec: slot bank=%u off=0x%lx len=%lu samp\n",    
                  (unsigned)rec_bank(), (unsigned long)slot_off,    
                  (unsigned long)slot_len);    
}    
  
bool af_rec_start(void) {        
    if (state != ST_IDLE) return false;        
    rec_len = 0;              
    pos = 0;      
    decim_phase = 0;    
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
    if (rec_slot_write(b, rec_len)) {      
        slot_off = rec_slot_off(b);      
        slot_len = rec_len;      
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
    slot_len = rec_slot_len(rec_bank());    
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
        ram_buf[pos++] = s;  
    }  
}  
  
int16_t __not_in_flash_func(af_rec_mix)(int16_t s) {      
    if (state != ST_PLAY) return s;      
    if (pos >= play_len) { state = ST_IDLE; af_rec_active = 0; return s; }      
    int32_t v;    
    if (play_src) {                          // voice_14: полная частота, 1:1    
        v = s + clip_voice_14[pos++];    
    } else {                                 // flash-слот: интерполяция ×2
        int16_t cur = rec_slot_data()[pos];   // XIP-чтение слота    
        if (++interp_phase >= AF_REC_DECIM) { interp_phase = 0; pos++; v = s + cur; }    
        else                                { v = s + (prev_samp + cur) / 2; }    
        prev_samp = cur;    
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
        int16_t cur = rec_slot_data()[pos];   // XIP-чтение слота    
        if (++interp_phase >= AF_REC_DECIM) { interp_phase = 0; pos++; v = cur; }    
        else                                { v = (prev_samp + cur) / 2; }    
        prev_samp = cur;    
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
