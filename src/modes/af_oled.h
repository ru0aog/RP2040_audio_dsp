// ============================================================================  
// OLED TZT 0.91" (SSD1306, 128x32, I2C 0x3C) — Core 0.  
// Bit-bang I2C на GP14 (SDA) / GP15 (SCL) — аппаратный I2C не занимаем.  
// Строка 0 — статус (RX/TX, band, cw f0), строки 1..3 — скролл текста  
// CW-декодера (21 символ/строку, шрифт 5x7 + 1px межсимвольный).  
// Обновление — пачками из af_oled_poll() в loop(): полная перерисовка  
// 512 байт по bit-bang ~50-100 мс, поэтому не чаще ~4 раз/сек.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define AF_OLED_SDA_PIN   14u  
#define AF_OLED_SCL_PIN   15u  
#define AF_OLED_ADDR      0x3Cu  
#define AF_OLED_W         128u  
#define AF_OLED_H         32u  
#define AF_OLED_ROWS      4u      // строк по 8 px  
#define AF_OLED_COLS      21u     // символов в строке (128/6)  
  
void af_oled_init(void);          // Core 0: init SSD1306; absent -> все no-op  
void af_oled_poll(void);          // Core 0: дренаж dirty-флага -> перерисовка  
void af_oled_putc(char c);        // символ от CW-декодера -> скролл-буфер  
void af_oled_status(void);        // пересобрать строку 0 (RX/TX/band/f0)  
void af_oled_clear(void);         // очистка экрана и буфера  
bool af_oled_present(void);       // модуль ответил на адрес  
