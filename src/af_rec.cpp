// src/af_rec.cpp    
// ============================================================================  
// af_rec.cpp — цифровой магнитофон, запись только в RAM.  
//  
// Запись: af_rec_push (Core 1) пишет в ram_buf каждый AF_REC_DECIM-й  
// отсчёт (децимация ×2: 15464 -> 7732 Гц, ~7.8 с в 60000 отсчётов).  
// Воспроизведение: af_rec_mix/af_rec_src читают ram_buf с интерполяцией  
// ×2 (линейная, между соседними отсчётами).  
// Пины: GP9 = запись (тракт -> TX, микрофон, без PTT),    
//       GP10 = воспроизведение (тракт -> TX + PTT на GP8, звук в TX).    
// Если с момента старта записи не было (rec_len == 0) — воспроизводится    
// клип-заглушка voice_14 (полная частота, без интерполяции). После первой    
// записи GP10/CLI play переключаются на RAM-буфер.    
// ============================================================================  
#include <Arduino.h>  
#include "af_rec.h"  
#include "af_config.h"  
#include "af_dsp.h"  
#include "pico/stdlib.h"
#include "voice/voice_14.inc"   // клип-заглушка: играется при пустом RAM-буфере 

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

// --- RAM-буфер записи --------------------------------------------------------  
static int16_t  ram_buf[AF_REC_RAM_SAMP];  
  


// --- API --------------------------------------------------------------------    
void af_rec_init(void) {  
    rec_len = 0;                    // RAM-only: после ребута буфер пуст  
    Serial.println("# rec: ram buf ready (no persistence)");  
}  
  
bool af_rec_start(void) {      
    if (state != ST_IDLE) return false;      
    // RAM-only: буфер очищается, flash не используется.
    rec_len = 0;            
    pos = 0;    
    decim_phase = 0;  
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
                  (unsigned long)(rec_len * AF_REC_DECIM / AF_FS_OUT_HZ),  
                  (unsigned long)((rec_len * AF_REC_DECIM % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ));  
    // flash-дамп убран — запись живёт только в RAM до следующей записи/ребута  
}  
  
bool af_rec_play(void) {      
    if (state != ST_IDLE) return false;      
    pos = 0;    
    interp_phase = 0;    
    prev_samp = 0;    
    if (rec_len == 0) {                     // записи не было — клип-заглушка    
        play_src = 1;    
        play_len = clip_voice_14_len;    
        Serial.printf("# rec: play %lu samp (voice14)\n", (unsigned long)play_len);    
    } else {    
        play_src = 0;    
        play_len = rec_len;    
        Serial.printf("# rec: play %lu samp (ram)\n", (unsigned long)play_len);    
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
uint32_t af_rec_play_len(void) {  return play_src ? (uint32_t)clip_voice_14_len : rec_len;  }  
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
    } else {                                 // ram_buf: интерполяция ×2    
        int16_t cur = ram_buf[pos];    
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
    } else {                                 // ram_buf: интерполяция ×2    
        int16_t cur = ram_buf[pos];    
        if (++interp_phase >= AF_REC_DECIM) { interp_phase = 0; pos++; v = cur; }    
        else                                { v = (prev_samp + cur) / 2; }    
        prev_samp = cur;    
    }    
    return v;                            // замещение микрофона    
}      
  
// --- Core 0: контроль записи (смена тракта, переполнение буфера) --------------  
void af_rec_poll(void) {          
    if (state != ST_REC) return;          
    if (pos >= AF_REC_RAM_SAMP) {           // RAM-буфер полон — стоп  
        Serial.println("# rec: ram buf full — stop");    
        af_rec_stop();    
    }    
}    
