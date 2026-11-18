// src/af_rec.cpp    
// ============================================================================    
// af_rec.cpp — цифровой магнитофон, один слот во flash.  
//  
// Запись: af_rec_push (Core 1) пишет int16 в ram_buf[AF_REC_RAM_SAMP]  
// в реальном времени — flash во время записи не трогаем.  
// Стоп: af_rec_stop (Core 0) дампит ram_buf во flash-слот  
// AF_REC_FLASH_OFF — последовательность как в af_store:  
// multicore_reset_core1 + set_sys_clock_khz(125000) + erase/program +  
// af_set_clk + rp2040.restartCore1. Тракт встаёт на ~1.5 с.  
//  
// Раскладка слота:  
//   [0..255]    заголовок rec_hdr_t: magic 'R','C', ver, len_samples  
//   [256..]     данные int16  
// Заголовок пишется ПОСЛЕДНИМ — прерванный дамп (питание, ребут)  
// читается как пустой слот.  
//  
// Воспроизведение: af_rec_mix/af_rec_src читают слот через XIP —  
// во время play flash-операций нет, чтение безопасно.    
// ============================================================================    
#include <Arduino.h>    
#include <string.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
#include "af_store.h"        // af_set_clk — восстановление клоков после дампа  
    
#include "pico/stdlib.h"    
#include "pico/multicore.h"   // multicore_reset_core1  
#include "hardware/flash.h"  // flash_range_erase/program  
#include "hardware/sync.h"   // save_and_disable_interrupts
#include "hardware/watchdog.h"  // watchdog_update в циклах дампа   

typedef enum { ST_IDLE = 0, ST_REC, ST_PLAY } rec_state_t;      
  
// Заголовок слота (256 байт — страница program). Пишется последним.  
#define AF_REC_HDR_BYTES  256u  
#define AF_REC_HDR_MAGIC0 'R'  
#define AF_REC_HDR_MAGIC1 'C'  
#define AF_REC_HDR_VER    1u  
  
typedef struct {  
    uint8_t  magic0, magic1, ver, flags;  
    uint32_t len_samples;  
    uint8_t  rsv[AF_REC_HDR_BYTES - 8];  
} rec_hdr_t;  
  
static const uint8_t* slot_base(void) {  
    return (const uint8_t*)(XIP_BASE + AF_REC_FLASH_OFF);  
}  
  
// Чтение заголовка при boot. Валиден => слот занят, rec_len = len_samples.  
static bool hdr_read(uint32_t* len_out) {  
    const rec_hdr_t* h = (const rec_hdr_t*)slot_base();  
    if (h->magic0 != AF_REC_HDR_MAGIC0 || h->magic1 != AF_REC_HDR_MAGIC1) return false;  
    if (h->ver != AF_REC_HDR_VER) return false;  
    if (h->len_samples == 0 || h->len_samples > AF_REC_RAM_SAMP) return false;  
    *len_out = h->len_samples;  
    return true;  
}  
  
// Flash I/O — только при сброшенной Core 1 и clk_sys=125 МГц (см. af_rec_stop).  
// SDK-обёртки сами делают connect/flush/enter_cmd_xip и save/restore IRQ.  
static void slot_erase_sector(uint32_t off) {        // off кратно 4096, внутри слота  
    flash_range_erase(AF_REC_FLASH_OFF + off, 4096);  
}  
static void slot_program(uint32_t off, const void* data, uint32_t len) {  
    flash_range_program(AF_REC_FLASH_OFF + off, (const uint8_t*)data, len);  
}  
    
static volatile rec_state_t state = ST_IDLE;      
volatile uint8_t af_rec_active = 0;   // 0=idle, 1=rec, 2=play — для af_dsp_poll  

static volatile uint32_t    rec_len = 0;             // валидных отсчётов в слоте    
static volatile uint32_t    pos     = 0;             // позиция записи/чтения    
static volatile uint8_t     rec_src_tx = 0;    
  
// --- RAM-буфер записи --------------------------------------------------------  
static int16_t  ram_buf[AF_REC_RAM_SAMP];  
// pos (ниже) — позиция записи/чтения, она же граница записанных данных.  

// --- API --------------------------------------------------------------------    
void af_rec_init(void) {      
    state = ST_IDLE;      
    af_rec_active = 0;  
    uint32_t len = 0;  
    if (hdr_read(&len)) {  
        rec_len = len;  
        Serial.printf("# rec: slot used %lu samp (%lu.%lu s)\n",  
                      (unsigned long)rec_len,  
                      (unsigned long)(rec_len / AF_FS_OUT_HZ),  
                      (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10 / AF_FS_OUT_HZ));  
    } else {  
        rec_len = 0;  
        Serial.printf("# rec: slot empty, max %u samp (%u s)\n",  
                      (unsigned)AF_REC_RAM_SAMP, (unsigned)(AF_REC_RAM_SAMP / AF_FS_OUT_HZ));  
    }  
}      
  
bool af_rec_start(void) {      
    if (state != ST_IDLE) return false;      
    // RAM-схема: flash не трогаем — стирание слота отложено до stop.    
    rec_len = 0;          
    pos = 0;  
    rec_src_tx = af_dsp_tx_on() ? 1 : 0;      
    state = ST_REC;
    rec_len = 0;    // старый слот логически пуст до дампа в af_rec_stop  
    af_rec_active = 1;
    Serial.printf("# rec: recording (%s), ram buf %lu samp\n",  
                  rec_src_tx ? "tx" : "rx", (unsigned long)AF_REC_RAM_SAMP);      
    return true;      
}      
  
void af_rec_stop(void) {      
    if (state == ST_PLAY) { af_rec_play_stop(); return; }      
    if (state != ST_REC) return;      
    state = ST_IDLE;      
    af_rec_active = 0;  
    rec_len = pos;      
    Serial.printf("# rec: saved %lu samp (%lu.%lu s)\n",      
                  (unsigned long)rec_len,      
                  (unsigned long)(rec_len / AF_FS_OUT_HZ),      
                  (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10 / AF_FS_OUT_HZ));      
  
    // --- Дамп ram_buf -> flash-слот. Тракт встаёт: Core 1 сброшена, ---  
    // --- IRQ запрещены на время каждой flash-операции (~45 мс/сектор). ---  
    if (rec_len == 0) return;  
  
    const uint32_t data_bytes = AF_REC_HDR_BYTES + rec_len * 2u;  
    const uint32_t sec_need = (data_bytes + 4095u) / 4096u;   // секторов под запись  
  
    multicore_reset_core1();                 // Core 1 в reset — XIP свободен  
    set_sys_clock_khz(125000, false);        // даунклок, как в af_store  
  
    for (uint32_t s = 0; s < sec_need; s++) {  
        slot_erase_sector(s * 4096u);  
        watchdog_update();                   // ~45 мс на сектор — кормим пса  
    }  
    // Данные — страницами по 256 байт из ram_buf  
    for (uint32_t off = 0; off < rec_len * 2u; off += 256u) {  
        uint32_t chunk = rec_len * 2u - off;  
        if (chunk > 256u) chunk = 256u;  
        slot_program(AF_REC_HDR_BYTES + off, (const uint8_t*)ram_buf + off, chunk);  
    }  
    // Заголовок — последним: прерванный дамп читается как пустой слот  
    rec_hdr_t h;  
    memset(&h, 0, sizeof(h));  
    h.magic0 = AF_REC_HDR_MAGIC0;  
    h.magic1 = AF_REC_HDR_MAGIC1;  
    h.ver    = AF_REC_HDR_VER;  
    h.len_samples = rec_len;  
    slot_program(0, &h, AF_REC_HDR_BYTES);  
  
    af_set_clk();                            // обратно 250 МГц + BAUDR (BUG-012)  
    rp2040.restartCore1();                   // Core 1: setup1 -> core1_main  
    Serial.println("# rec: flash dump done");  
}      
  
bool af_rec_play(void) {    
    if (state != ST_IDLE || rec_len == 0) return false;    
    pos = 0;    
    state = ST_PLAY;
    af_rec_active = 2;
    Serial.printf("# rec: play %lu samp\n", (unsigned long)rec_len);    
    return true;    
}    
  
void af_rec_play_stop(void) {    
    if (state == ST_PLAY) {
        state = ST_IDLE;
        af_rec_active = 0;
        Serial.println("# rec: play stop"); 
    }
}    
  
// Геттеры — в RAM: вызываются из af_voice_mix/af_dsp_poll (Core 1)  
// во время erase/program, когда XIP выключен.  
bool     __not_in_flash_func(af_rec_is_recording)(void)  { return state == ST_REC; }      
bool     __not_in_flash_func(af_rec_is_playing)(void)    { return state == ST_PLAY; }      
uint32_t __not_in_flash_func(af_rec_len_samples)(void)   { return rec_len; }      
uint32_t __not_in_flash_func(af_rec_pos_samples)(void)   { return pos; }      
  
// --- Core 1, горячий путь (RAM) ----------------------------------------------    
void __not_in_flash_func(af_rec_push)(int16_t s) {        
    if (state != ST_REC) return;        
    if (pos >= AF_REC_RAM_SAMP) return;   // буфер полон — stop сделает af_rec_poll  
    ram_buf[pos++] = s;        
}      
  
int16_t __not_in_flash_func(af_rec_mix)(int16_t s) {    
    if (state != ST_PLAY) return s;    
    if (pos >= rec_len) { state = ST_IDLE; af_rec_active = 0; return s; }    
    const int16_t* d = (const int16_t*)(slot_base() + AF_REC_HDR_BYTES);  
    int32_t v = s + d[pos++];    
    if (v > 32767) v = 32767; else if (v < -32768) v = -32768;    
    return (int16_t)v;    
}    
  
int32_t __not_in_flash_func(af_rec_src)(int32_t mic) {    
    if (state != ST_PLAY) return mic;    
    if (pos >= rec_len) { state = ST_IDLE; af_rec_active = 0; return mic; }    
    const int16_t* d = (const int16_t*)(slot_base() + AF_REC_HDR_BYTES);  
    return (int32_t)d[pos++];              // замещение микрофона    
}    
  
// --- Core 0: программирование готовых буферов ---------------------------------    
void af_rec_poll(void) {          
    if (state != ST_REC) return;          
    if ((af_dsp_tx_on() ? 1 : 0) != rec_src_tx) {   // тракт сменился — финализируем      
        Serial.println("# rec: src changed (rx<->tx) — stop");      
        af_rec_stop();      
        return;      
    }      
    if (pos >= AF_REC_RAM_SAMP) {           // RAM-буфер полон — стоп  
        Serial.println("# rec: ram buf full — stop");    
        af_rec_stop();    
    }    
}    
