// src/af_rec.cpp    
// ============================================================================    
// af_rec.cpp — цифровой магнитофон, один слот во flash.  
//  
// Запись: af_rec_push (Core 1) пишет int16 в ram_buf[AF_REC_RAM_SAMP]  
// в реальном времени — flash во время записи не трогаем.  
// Стоп: af_rec_stop (Core 0) дампит ram_buf во flash-слот    
// AF_REC_FLASH_OFF — последовательность как в af_store:      
// af_store_core1_park + set_sys_clock_khz(125000) +        
// save_and_disable_interrupts + do_rec_dump (RAM) + restore_interrupts +    
// flash_flush_cache + af_set_clk + af_store_core1_unpark.        
// Core 1 паркуется в safe-точке (не reset) — тракт поднимается мгновенно.    
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
#include "pico/multicore.h"    // multicore_reset_core1  
#include "hardware/watchdog.h"  // watchdog_update в do_rec_dump 

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
  
// --- Flash I/O: вызывается напрямую под save_and_disable_interrupts    
// при припаркованном Core 1 (как do_flash_write в af_store).    
// flash_safe_execute НЕ используем — его lockout молча возвращал ошибку.    
// Весь дамп — одна RAM-функция под одним запретом IRQ (как do_flash_write    
// в af_store): во время erase/program XIP выключен, код обязан быть в RAM.    
// memset/memcpy не вызываем — они могут лежать во flash; копия — циклом.    
static void __not_in_flash_func(do_rec_dump)(void* param) {      
    (void)param;      
    const uint32_t data_bytes = rec_len * 2u;      
    const uint32_t nsec = (AF_REC_HDR_BYTES + data_bytes + 4095) / 4096;      
  
    // Стирание — ОДНИМ вызовом на весь диапазон (как af_store: один заход,    
    // без повторных XIP-переходов между секторами).    
    flash_range_erase(AF_REC_FLASH_OFF, nsec * 4096u);    
  
    // Данные — напрямую из ram_buf, без промежуточного буфера.    
    // flash_range_program требует кратности 256: полные страницы пишем    
    // прямо из буфера, хвост (<256) — через статический SRAM-буфер.    
    const uint8_t* p = (const uint8_t*)ram_buf;    
    const uint32_t full = data_bytes & ~255u;    
    if (full)    
        flash_range_program(AF_REC_FLASH_OFF + AF_REC_HDR_BYTES, p, full);    
    const uint32_t tail = data_bytes - full;    
    if (tail) {    
        static uint8_t tail_page[256];            // статик — точно в SRAM    
        const uint8_t* src = p + full;    
        for (uint32_t i = 0; i < tail; i++) tail_page[i] = src[i];    
        for (uint32_t i = tail; i < 256u; i++) tail_page[i] = 0xFF;    
        flash_range_program(AF_REC_FLASH_OFF + AF_REC_HDR_BYTES + full,    
                            tail_page, 256u);    
    }    
  
    // Заголовок — последним: прерванный дамп читается как пустой слот.    
    rec_hdr_t h;      
    uint8_t* hb = (uint8_t*)&h;      
    for (uint32_t i = 0; i < sizeof(h); i++) hb[i] = 0xFF;      
    h.magic[0] = 'R'; h.magic[1] = 'C'; h.ver = 1; h.flags = 0;      
    h.len_samples = rec_len;      
    flash_range_program(AF_REC_FLASH_OFF, hb, AF_REC_HDR_BYTES);      
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
    Serial.flush();    
    multicore_reset_core1();    
    busy_wait_us(100);                        // как в af_store — дать Core 1 умереть    
    Serial.println("# rec: dump: downclock");    
    Serial.flush();    
    set_sys_clock_khz(125000, false);    
  
    const uint32_t nsec = (AF_REC_HDR_BYTES + rec_len * 2u + 4095) / 4096;    
    Serial.printf("# rec: dump: erase+program %lu sec\n", (unsigned long)nsec);      
    Serial.flush();    
    uint32_t ints = save_and_disable_interrupts();    
    do_rec_dump(NULL);              // RAM-функция: erase + program + заголовок    
    restore_interrupts(ints);    
    flash_flush_cache();            // сброс XIP-кэша после записи    
  
    Serial.println("# rec: dump: restore clk + c1");    
    af_set_clk();    
    rp2040.restartCore1();                    // Core 1 поднимается заново    
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
