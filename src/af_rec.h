// src/af_rec.h        
//          
// Цифровой магнитофон: 4 слота во flash по 224 КБ (Core 1 — tap записи          
// и подмес/замещение воспроизведения, Core 0 — контроль).          
// Источник записи — активный тракт после всех обработок, до подмеса          
// голосовых сообщений (чистый эфир/микрофон).          
// Воспроизведение: RX — подмес в выход; TX — замещение микрофона          
// (запись идёт через весь TX-тракт: компрессор/клиппер/АРУ).          
//          
// Запись — RAM-схема с децимацией ×2: каждый AF_REC_DECIM-й отсчёт          
// в ram_buf[AF_REC_RAM_SAMP] (15464 -> 7732 Гц, ~7.2 с),          
// на стопе сброс во flash-слот через flash-окно.          
// Воспроизведение — чтение слота из XIP с линейной интерполяцией ×2          
// обратно в 15464 Гц. Слот 224 КБ вмещает ~14.8 с — запас под          
// потоковую запись/аплоад; лимит записи с АЦП = AF_REC_RAM_SAMP.          
// Пины: GP9 = запись (тракт -> TX, микрофон, без PTT),          
//       GP10 = воспроизведение (тракт -> TX + PTT на GP8).               
  
#pragma once    
  
#include <Arduino.h>
// RAM-буфер записи — ограничение свободная RAM.    
// AF_REC_MAX_SAMP = размер буфера (нужен CLI: recs/rec).    
// AF_REC_RAM_SAMP — в af_config.h (56000, ~7.2 с @ 7732 Гц)  
#define AF_REC_DECIM      2u              // децимация ×2: запись 7732 Гц ≈ 7.2 с в RAM      
#define AF_REC_MAX_SAMP   AF_REC_RAM_SAMP    
// Число слотов магнитофона = число сообщений (селектор GP13/GP28, код 0..3).  
#define AF_REC_SLOTS   4  
  
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
bool af_rec_erase_all(void);   // очистка всех 4 слотов (banks clr)    

// Доступ к слотам для файлового менеджера (WAV-экспорт, read-only).      
// bank = 1..4. len = отсчётов PCM16 @7732 Гц (0 = слот пуст).      
// data = XIP-указатель на PCM слота (NULL если bank вне диапазона).    
uint32_t       af_rec_slot_len(uint8_t bank);    
const int16_t* af_rec_slot_data(uint8_t bank);    

// --- Аплоад WAV с ПК (af_fm) -------------------------------------------------    
int16_t*     af_rec_ram_buf(void);                    // буфер для конвертации    
bool         af_rec_is_busy(void);                    // rec/play идёт    
bool         af_rec_slot_write(uint8_t bank, uint32_t n_samp);  // ram_buf -> flash    
