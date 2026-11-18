// src/af_rec.cpp    
// ============================================================================    
// af_rec.cpp — цифровой магнитофон, один слот в RAM (отладочный этап).    
//    
// Запись: af_rec_push (Core 1) пишет int16 в ram_buf[AF_REC_RAM_SAMP].    
// Воспроизведение: af_rec_mix/af_rec_src читают ram_buf — flash не    
// трогаем вообще, инвариант «loop1() в RAM» для магнитофона не нужен.    
// Запись не переживает ребут — слот volatile.    
//    
// TODO(flash): вернуть дамп ram_buf -> flash-слот AF_REC_FLASH_OFF    
// (191 сектор) в af_rec_stop — последовательность как в af_store:    
// multicore_reset_core1 + set_sys_clock_khz(125000) + erase/program +    
// af_set_clk + rp2040.restartCore1. Заголовок rec_hdr_t писать последним.    
// ============================================================================    
#include <Arduino.h>    
#include <string.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
  
#include "pico/stdlib.h"  
// hardware/flash.h, hardware/sync.h, af_store.h — не нужны в RAM-режиме,  
// вернуть вместе с flash-дампом (см. TODO в шапке).  

typedef enum { ST_IDLE = 0, ST_REC, ST_PLAY } rec_state_t;    
  
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
    rec_len = 0;   // RAM-слот volatile — после ребута всегда пуст  
    Serial.printf("# rec: ram slot, max %lu samp (%lu s)\n",  
                  (unsigned long)AF_REC_RAM_SAMP,  
                  (unsigned long)(AF_REC_RAM_SAMP / AF_FS_OUT_HZ));  
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
    // RAM-режим: запись остаётся в ram_buf, flash не трогаем (см. TODO).  
    Serial.printf("# rec: saved %lu samp (%lu.%lu s)\n",            
                  (unsigned long)rec_len,          
                  (unsigned long)(rec_len / AF_FS_OUT_HZ),          
                  (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ));          
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
    const int16_t* d = ram_buf; 
    int32_t v = s + d[pos++];    
    if (v > 32767) v = 32767; else if (v < -32768) v = -32768;    
    return (int16_t)v;    
}    
  
int32_t __not_in_flash_func(af_rec_src)(int32_t mic) {    
    if (state != ST_PLAY) return mic;    
    if (pos >= rec_len) { state = ST_IDLE; af_rec_active = 0; return mic; }    
    const int16_t* d = ram_buf; 
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
