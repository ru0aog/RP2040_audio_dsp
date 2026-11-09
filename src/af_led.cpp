// src/af_led.cpp  
// ============================================================================  
// af_led.cpp — heartbeat-индикация: параллельный bit-bang WS2812B (GP16+GP23)  
// и монохромный LED (GP25). Порт led_blink.cpp v2.12 из RP2040_beacon.  
//  
// Тайминги бита пересчитаны под clk_sys=220 МГц (исходные были под ~125 МГц,  
// счётчики умножены на 1.76).  
//  
// Heartbeat (период 1000 мс):  
//   0..200 мс    — плавный набор 0 -> 15% (еле-еле тлеет)  
//   200..250 мс  — вспышка 80%  
//   250..500 мс  — плавный спад 80% -> 0  
//   500..1000 мс — полностью погашен
// ============================================================================  
#include <Arduino.h>  
#include <hardware/gpio.h>  
#include "af_led.h"  
  
#define PIN_ZERO   16   // WS2812B на RP2040-Zero  
#define PIN_YD     23   // WS2812B на YD-RP2040  
#define PIN_PICO   25   // LED_BUILTIN на обычной Pico  
  
static const uint32_t RGB_PINS_MASK = (1u << PIN_ZERO) | (1u << PIN_YD);  
static const uint32_t ALL_PINS_MASK = RGB_PINS_MASK | (1u << PIN_PICO);  
  
// Параллельная отправка байта на GP16 и GP23.  
// Тайминги WS2812B под 220 МГц: T1H~40, T1L~15, T0H~4, T0L~80 циклов.  
static void __time_critical_func(send_raw_byte_parallel)(uint8_t v) {  
    for (int i = 7; i >= 0; i--) {  
        if ((v >> i) & 1) {  
            sio_hw->gpio_set = RGB_PINS_MASK;  
            for (volatile int d = 0; d < 40; d++);  
            sio_hw->gpio_clr = RGB_PINS_MASK;  
            for (volatile int d = 0; d < 15; d++);  
        } else {  
            sio_hw->gpio_set = RGB_PINS_MASK;  
            for (volatile int d = 0; d < 4; d++);  
            sio_hw->gpio_clr = RGB_PINS_MASK;  
            for (volatile int d = 0; d < 80; d++);  
        }  
    }  
}  
  
void af_led_set(uint8_t r, uint8_t g, uint8_t b) {  
    noInterrupts();  
    send_raw_byte_parallel(g);   // WS2812B: порядок G-R-B  
    send_raw_byte_parallel(r);  
    send_raw_byte_parallel(b);  
    interrupts();  
    sio_hw->gpio_clr = RGB_PINS_MASK;  
    delayMicroseconds(600);      // latch  
  
    if (r || g || b) sio_hw->gpio_set = (1u << PIN_PICO);  
    else             sio_hw->gpio_clr = (1u << PIN_PICO);  
}  
  
void af_led_off(void) { af_led_set(0, 0, 0); }  
  
void af_led_init(void) {  
    gpio_init(PIN_ZERO); gpio_set_dir(PIN_ZERO, GPIO_OUT);  
    gpio_init(PIN_YD);   gpio_set_dir(PIN_YD,   GPIO_OUT);  
    gpio_init(PIN_PICO); gpio_set_dir(PIN_PICO, GPIO_OUT);  
    sio_hw->gpio_clr = ALL_PINS_MASK;  
}  
  
// ----------------------------------------------------------------------------  
// Heartbeat: плавная кривая яркости зелёного канала.  
// Яркость пересчитывается по фазе цикла; шина дёргается только при изменении  
// значения (last_g) — ~60 обновлений/сек по ~30 мкс, на DSP не влияет.  
// ----------------------------------------------------------------------------  
void af_led_poll(void) {  
    static uint32_t t0    = 0;  
    static uint8_t  last_g = 0xFF;   // принудительная первая отправка  
  
    uint32_t ph = (millis() - t0) % 1000;  
    uint8_t g;  
  
    if (ph < 200) {                       // набор 0 -> 64 (25%)  
        g = (uint8_t)((ph * 38) / 200);   // 38 - 15%, 64 - 25%
    } else if (ph < 250) {                // вспышка 204 (80%)  
        g = 204;  
    } else if (ph < 500) {                // спад 204 -> 0  
        g = (uint8_t)(204 - ((ph - 250) * 204) / 250);  
    } else {                              // 500..1000 мс — тишина  
        g = 0;  
    }  
  
    if (g != last_g) {  
        af_led_set(0, g, 0);  
        last_g = g;  
    }  
}