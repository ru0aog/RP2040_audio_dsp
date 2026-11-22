// src/modes/rtty_rx.h  
// ============================================================================  
// RTTY-декодер 45.45 бод / shift 170 Гц, Baudot ITA-2 + МТК-2 (UTF-8).  
// Вход: моно int16 @ AF_FS_OUT_HZ (15464 Гц), отвод из RX-тракта при rtty on.  
// Декодер на float — работает ТОЛЬКО на Core 0 (rttydec_poll из loop()).  
// Core 1 лишь кладёт отсчёты в SPSC-очередь через rttydec_push().  
// Захват: RSID-метка (rsid_rx) -> центр; fallback — сканер Гёрцеля.  
// Удержание: AFC по dphi между битовыми стробами (кламп +-22.7 Гц).  
// Вывод текста — Serial.write внутри модуля.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define RTTY_SHIFT_HZ     170.0f  
#define RTTY_BAUD         45.45f  
#define RTTY_DEF_CENTER   1500.0f   // стартовый центр сканера/ручной режим  
  
// Состояния захвата (для stats)  
enum rttydec_state_t : uint8_t {  
    RTTY_ST_RSID = 0,   // ждём RSID-метку  
    RTTY_ST_SCAN,       // сканер Гёрцеля ищет пару тонов  
    RTTY_ST_TRACK       // декодируем, AFC активен  
};  
  
// ---- обвязка (граница Core 1 / Core 0) ----  
void     rttydec_init(void);            // включить декодер (state=RSID/SCAN)  
void     rttydec_push(int16_t sample);  // Core 1: отсчёт в очередь (cheap)  
void     rttydec_poll(void);            // Core 0: дренаж + декодер + RSID + сканер  
void     rttydec_stop(void);            // сброс, очистка очереди  
uint32_t rttydec_dropped(void);         // отсчёты, не влезшие в очередь  
  
// ---- настройки (Core 0) ----  
void     rttydec_set_scan(bool on);     // сканер как fallback после RSID-таймаута  
void     rttydec_set_rsid(bool on);     // слушать RSID-метки  
void     rttydec_set_center(float hz);  // ручной центр -> сразу TRACK, без поиска  
void     rttydec_set_rev(bool on);      // инверсия полярности mark/space    
void     rttydec_set_sql(float lvl);    // сквелч 0..1 (0 = off)    
void     rttydec_set_dbg(bool on);      // диагностика # scan:/# trk: в Serial    
  
// ---- статус (для stats/банков) ----  
rttydec_state_t rttydec_state(void);  
float    rttydec_f_mark(void);  
float    rttydec_f_space(void);  
float    rttydec_freqerr(void);  
bool     rttydec_rev(void);  
bool     rttydec_scan_en(void);    
bool     rttydec_rsid_en(void);    
bool     rttydec_dbg_en(void);    
float    rttydec_sql(void);  
