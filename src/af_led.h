// src/af_led.h  
//  
// Светодиодная индикация (heartbeat): WS2812B на GP16 (RP2040-Zero) и  
// GP23 (YD-RP2040) параллельно + монохромный LED на GP25 (Pico).  
// Порт led_blink.cpp из RP2040_beacon (bit-bang по SIO, тайминги под 220 МГц).  
  
#pragma once  
  
#include <Arduino.h>  
  
void af_led_init(void);                          // пины на выход, всё погашено  
void af_led_poll(void);                          // heartbeat, вызывать из loop()  
void af_led_set(uint8_t r, uint8_t g, uint8_t b);// произвольный цвет 0..255  
void af_led_off(void);
// Сообщить LED текущий банк (0=ручной, 1..7=ячейки). Вызывать из af_gpio_ctl  
// при смене кода на GP2..GP4 и из af_cli при 'load x'.  
void af_led_set_bank(uint8_t bank);