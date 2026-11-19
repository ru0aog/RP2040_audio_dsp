// src/af_rec.h      
//        
// Цифровой магнитофон: запись только в RAM (Core 1 — tap записи        
// и подмес/замещение воспроизведения, Core 0 — контроль).        
// Источник записи — активный тракт после всех обработок, до подмеса        
// голосовых сообщений (чистый эфир/микрофон).        
// Воспроизведение: RX — подмес в выход; TX — замещение микрофона        
// (запись идёт через весь TX-тракт: компрессор/клиппер/АРУ).        
//        
// Запись — RAM-схема с децимацией ×2: каждый AF_REC_DECIM-й отсчёт        
// в ram_buf[AF_REC_RAM_SAMP] (15464 -> 7732 Гц, ~7.8 с).        
// Воспроизведение — линейная интерполяция ×2 обратно в 15464 Гц.        
// Flash не используется — запись не персистентна.        
// Пины: GP9 = запись (тракт -> TX, микрофон, без PTT),        
//       GP10 = воспроизведение (тракт -> TX + PTT на GP8).             
  
#pragma once    
  
#include <Arduino.h>
// RAM-буфер записи — ограничение свободная RAM.    
// AF_REC_MAX_SAMP = размер буфера (нужен CLI: recs/rec).    
#define AF_REC_RAM_SAMP   60000u          // 117 КБ    
#define AF_REC_DECIM      2u              // децимация ×2: запись 7732 Гц ≈ 7.8 с    
#define AF_REC_MAX_SAMP   AF_REC_RAM_SAMP    
  
void     af_rec_init(void);    
void     af_rec_poll(void);        // Core 0: контроль записи (buf full -> авто-стоп)        
  
bool     af_rec_start(void);       // старт записи (false = занят/нет места)    
void     af_rec_stop(void);        // стоп записи (rec_len = pos)  
bool     af_rec_play(void);        // старт воспроизведения (false = пусто/занято)    
void     af_rec_play_stop(void);    
  
bool     af_rec_is_recording(void);  
bool     af_rec_is_playing(void);  
  
// Флаг активности для горячего пути Core 1: 0=idle, 1=rec, 2=play.  
// Читается в af_dsp_poll вместо вызовов af_rec_* в idle — экономия  
// call-overhead на каждый выходной сэмпл. // Пишется только из af_rec.cpp    
// (Core 0), читается на Core 1.  
extern volatile uint8_t af_rec_active;  

uint32_t af_rec_len_samples(void); // длина записи в буфере (0 = пусто)      
uint32_t af_rec_pos_samples(void); // текущая позиция rec/play    
  
// Core 1, горячий путь — обе функции в RAM.    
void    af_rec_push(int16_t s);    // tap записи (no-op если не пишем)    
int16_t af_rec_mix(int16_t s);     // подмес воспроизведения в RX    
int32_t af_rec_src(int32_t mic);   // источник для TX: запись вместо микрофона    
uint32_t af_rec_play_len(void);    // длина текущего/последнего воспроизведения (ram или voice_14)  