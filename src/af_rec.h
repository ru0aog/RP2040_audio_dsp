// src/af_rec.h    
//    
// Цифровой магнитофон: один слот 25 с во flash (Core 0 — flash I/O,    
// Core 1 — tap записи и подмес воспроизведения).    
// Источник записи — активный тракт после всех обработок, до подмеса    
// голосовых сообщений (чистый эфир/микрофон).    
// Воспроизведение: RX — подмес в выход; TX — замещение микрофона    
// (запись идёт через весь TX-тракт: компрессор/клиппер/АРУ).    
  
#pragma once    
  
#include <Arduino.h>    
  
void     af_rec_init(void);    
void     af_rec_poll(void);        // Core 0: flash program/erase конвейер    
  
bool     af_rec_start(void);       // старт записи (false = занят/нет места)    
void     af_rec_stop(void);        // стоп + финализация заголовка    
bool     af_rec_play(void);        // старт воспроизведения (false = пусто/занято)    
void     af_rec_play_stop(void);    
  
bool     af_rec_is_recording(void);    
bool     af_rec_is_playing(void);    
uint32_t af_rec_len_samples(void); // длина записи в слоте (0 = пустой)    
uint32_t af_rec_pos_samples(void); // текущая позиция rec/play    
  
// Core 1, горячий путь — обе функции в RAM.    
void    af_rec_push(int16_t s);    // tap записи (no-op если не пишем)    
int16_t af_rec_mix(int16_t s);     // подмес воспроизведения в RX    
int32_t af_rec_src(int32_t mic);   // источник для TX: запись вместо микрофона    
