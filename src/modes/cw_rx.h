// src/modes/cw_rx.h  
// ============================================================================  
// CW-декодер (морзянка) — порт из RP2040_radiomodem (FLDigi cw.cxx).  
// Вход: моно int16 @ AF_FS_OUT_HZ, отвод из RX-тракта в режиме cw on.  
// Декодер на float — работает ТОЛЬКО на Core 0 (cwdec_poll из loop()).  
// Core 1 лишь кладёт отсчёты в SPSC-очередь через cwdec_push().  
// Вывод символов — Serial.write внутри модуля.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define CW_TONE_OFFSET_HZ   600u     // дефолтный тон, если центр не задан  
#define CW_DEFAULT_WPM      15u      // стартовая скорость (PARIS), дальше трекинг  
  
// ---- обвязка (Core 0 / Core 1 граница) ----  
void     cwdec_init(uint32_t center_hz); // инициализация декодера на тон, Гц  
void     cwdec_push(int16_t sample);     // Core 1: отсчёт в очередь (cheap)  
void     cwdec_poll(void);               // Core 0: дренаж очереди -> cw_rx_push  
void     cwdec_set_wpm(uint8_t wpm);     // стартовая скорость 5..60  
void     cwdec_stop(void);               // флаш незавершённого символа + сброс  
uint32_t cwdec_dropped(void);            // отсчёты, не влезшие в очередь  
  
// ---- внутренний API декодера (вызывается только из cwdec_poll, Core 0) ----  
void cw_rx_init(uint32_t center_hz);  
void cw_rx_push(int16_t sample);  
void cw_rx_set_wpm(uint8_t wpm);  
void cw_rx_stop(void);  
