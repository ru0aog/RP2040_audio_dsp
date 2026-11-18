// src/af_rec.h      
//      
// Цифровой магнитофон: один слот во flash (Core 0 — flash I/O,      
// Core 1 — tap записи и подмес воспроизведения).      
// Источник записи — активный тракт после всех обработок, до подмеса      
// голосовых сообщений (чистый эфир/микрофон).      
// Воспроизведение: RX — подмес в выход; TX — замещение микрофона      
// (запись идёт через весь TX-тракт: компрессор/клиппер/АРУ).      
//      
// Запись — RAM-схема: отсчёты копятся в статическом буфере      
// (AF_REC_RAM_SAMP, ~5 с), flash не трогаем. При stop — дамп      
// во flash в безопасном режиме: reset Core 1, clk 125 МГц,      
// erase+program, af_set_clk(), restartCore1. Без ребута.       
  
#pragma once    
  
#include <Arduino.h>
#include "af_config.h"   // AF_REC_FLASH_OFF/LEN  

// Геометрия слота — в af_config.h (AF_REC_FLASH_OFF/LEN).    
// Максимум отсчётов int16 в слоте — нужен и CLI (recs/rec).    
#define AF_REC_MAX_SAMP   (AF_REC_FLASH_LEN / 2)    
    
// RAM-буфер записи при 15464 Гц.    
// Ограничение — свободная RAM    
#define AF_REC_RAM_SAMP   60000u          // 117 КБ ≈ 3.9 с @ 15464 Гц  
  
void     af_rec_init(void);    
void     af_rec_poll(void);        // Core 0: контроль записи (src change, buf full)      
  
bool     af_rec_start(void);       // старт записи (false = занят/нет места)    
void     af_rec_stop(void);        // стоп + финализация заголовка    
bool     af_rec_play(void);        // старт воспроизведения (false = пусто/занято)    
void     af_rec_play_stop(void);    
  
bool     af_rec_is_recording(void);  
bool     af_rec_is_playing(void);  
  
// Флаг активности для горячего пути Core 1: 0=idle, 1=rec, 2=play.  
// Читается в af_dsp_poll вместо вызовов af_rec_* в idle — экономия  
// call-overhead на каждый выходной сэмпл. Пишется только из af_rec.c  
// (Core 0), читается на Core 1.  
extern volatile uint8_t af_rec_active;  

uint32_t af_rec_len_samples(void); // длина записи в слоте (0 = пустой)    
uint32_t af_rec_pos_samples(void); // текущая позиция rec/play    
  
// Core 1, горячий путь — обе функции в RAM.    
void    af_rec_push(int16_t s);    // tap записи (no-op если не пишем)    
int16_t af_rec_mix(int16_t s);     // подмес воспроизведения в RX    
int32_t af_rec_src(int32_t mic);   // источник для TX: запись вместо микрофона    
