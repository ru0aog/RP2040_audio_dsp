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
//  
// Этап 1: дамп ещё не внедрён — af_rec_stop только фиксирует rec_len,  
// слот валиден только если был записан ранее (после ребута — пуст).
//
// Этап 2: дамп внедрён с маркерами Serial.println — отладка,    
// после проверки на железе маркеры убираются (этап 3).       
//
// ============================================================================    
#include <Arduino.h>    
#include <string.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
#include "pico/stdlib.h"    
#include "af_store.h"          // af_set_clk — восстановление клоков после дампа  
#include "hardware/flash.h"    // flash_range_erase/program  
#include "hardware/sync.h"     // save_and_disable_interrupts  
#include "hardware/watchdog.h" // watchdog_update в циклах дампа  
#include "pico/multicore.h"    // multicore_reset_core1  

typedef enum { ST_IDLE = 0, ST_REC, ST_PLAY } rec_state_t;    
  
static volatile rec_state_t state = ST_IDLE;  
volatile uint8_t af_rec_active = 0;   // 0=idle, 1=rec, 2=play — для af_dsp_poll  

static volatile uint32_t    rec_len = 0;             // валидных отсчётов в слоте    
static volatile uint32_t    pos     = 0;             // позиция записи/чтения    

static volatile uint8_t     rec_src_tx = 0;      
    
// --- RAM-буфер записи --------------------------------------------------------    
static int16_t  ram_buf[AF_REC_RAM_SAMP];    
  
// --- Flash-слот: заголовок ---------------------------------------------------  
#define AF_REC_HDR_BYTES  256u  
  
typedef struct {  
    uint8_t  magic[2];      // 'R','C'  
    uint8_t  ver;  
    uint8_t  flags;  
    uint32_t len_samples;  
    uint8_t  rsv[248];  
} rec_hdr_t;                // ровно 256 байт  
  
static inline const uint8_t* slot_base(void) {  
    return (const uint8_t*)(XIP_BASE + AF_REC_FLASH_OFF);  
}  
  
// Чтение заголовка через XIP — только на Core 0, вне flash-операций.    
static bool hdr_read(uint32_t* len_out) {    
    const rec_hdr_t* h = (const rec_hdr_t*)slot_base();    
    if (h->magic[0] != 'R' || h->magic[1] != 'C') return false;    
    if (h->len_samples == 0 || h->len_samples > AF_REC_RAM_SAMP) return false;    
    *len_out = h->len_samples;    
    return true;    
}    
  
// --- Flash I/O: SDK-обёртки, вызывать ТОЛЬКО при остановленном Core 1 ---------    
// (после multicore_reset_core1 — Core 1 не трогает XIP, IRQ Core 0    
// запрещены внутри flash_range_* на время операции).    
static void slot_erase_sector(uint32_t off) {    
    uint32_t ints = save_and_disable_interrupts();    
    flash_range_erase(AF_REC_FLASH_OFF + off, FLASH_SECTOR_SIZE);    
    restore_interrupts(ints);    
}    
static void slot_program(uint32_t off, const void* data, uint32_t len) {    
    uint32_t ints = save_and_disable_interrupts();    
    flash_range_program(AF_REC_FLASH_OFF + off, (const uint8_t*)data, len);    
    restore_interrupts(ints);    
}    

// --- API --------------------------------------------------------------------    
void af_rec_init(void) {        
    uint32_t len = 0;  
    if (hdr_read(&len)) {  
        rec_len = len;  
        Serial.printf("# rec: slot used %lu samp (%lu.%lu s)\n",  
                      (unsigned long)rec_len,  
                      (unsigned long)(rec_len / AF_FS_OUT_HZ),  
                      (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ));  
    } else {  
        rec_len = 0;  
        Serial.println("# rec: slot empty");  
    }  
}        
  
bool af_rec_start(void) {      
    if (state != ST_IDLE) return false;      
    // RAM-схема: flash не трогаем — стирание слота отложено до stop.    
    rec_len = 0;          
    pos = 0;  
    rec_src_tx = af_dsp_tx_on() ? 1 : 0;      
    state = ST_REC;
    af_rec_active = 1;
    Serial.printf("# rec: recording (%s), ram buf %lu samp\n",  
                  rec_src_tx ? "tx" : "rx", (unsigned long)AF_REC_RAM_SAMP);      
    return true;      
}      
  
void af_rec_stop(void) {              
    if (state == ST_PLAY) { af_rec_play_stop(); return; }   // stop универсальный          
    if (state != ST_REC) return;              
    state = ST_IDLE;  
    af_rec_active = 0;  
    rec_len = pos;          
    if (rec_len == 0) { Serial.println("# rec: nothing recorded"); return; }          
    Serial.printf("# rec: saved %lu samp (%lu.%lu s)\n",              
                  (unsigned long)rec_len,            
                  (unsigned long)(rec_len / AF_FS_OUT_HZ),            
                  (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ));            
  
    // --- Дамп ram_buf -> flash-слот (этап 2: с маркерами) -------------------    
    Serial.println("# rec: dump: reset c1");    
    multicore_reset_core1();    
  
    Serial.println("# rec: dump: downclock");    
    set_sys_clock_khz(125000, false);    
  
    // Стирание: заголовочный сектор + сектора данных (округление вверх).    
    const uint32_t data_bytes = rec_len * 2u;    
    const uint32_t nsec = 1 + (data_bytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;    
    Serial.printf("# rec: dump: erase %lu sec\n", (unsigned long)nsec);    
    for (uint32_t i = 0; i < nsec; i++) {    
        slot_erase_sector(i * FLASH_SECTOR_SIZE);    
        watchdog_update();    
    }    
  
    // Данные — страницами по 256 байт; хвост добиваем 0xFF до страницы.    
    Serial.println("# rec: dump: program data");    
    uint8_t page[256];    
    for (uint32_t off = 0; off < data_bytes; off += 256u) {    
        uint32_t chunk = data_bytes - off;    
        if (chunk >= 256u) {    
            slot_program(AF_REC_HDR_BYTES + off, (const uint8_t*)ram_buf + off, 256u);    
        } else {    
            memcpy(page, (const uint8_t*)ram_buf + off, chunk);    
            memset(page + chunk, 0xFF, 256u - chunk);    
            slot_program(AF_REC_HDR_BYTES + off, page, 256u);    
        }    
        watchdog_update();    
    }    
  
    // Заголовок — последним: прерванный дамп читается как пустой слот.    
    Serial.println("# rec: dump: header");    
    rec_hdr_t h;    
    memset(&h, 0xFF, sizeof(h));    
    h.magic[0] = 'R'; h.magic[1] = 'C'; h.ver = 1; h.flags = 0;    
    h.len_samples = rec_len;    
    slot_program(0, &h, AF_REC_HDR_BYTES);    
  
    Serial.println("# rec: dump: restore clk + c1");    
    af_set_clk();    
    rp2040.restartCore1();    
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
  
// --- Core 0: контроль записи (смена тракта, переполнение буфера) --------------  
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
