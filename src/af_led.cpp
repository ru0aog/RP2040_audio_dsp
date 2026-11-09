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
#include "af_dsp.h"     // af_dsp_nr_on(), af_dsp_notch_on()  
  
#define PIN_ZERO   16   // WS2812B на RP2040-Zero  
#define PIN_YD     23   // WS2812B на YD-RP2040  
#define PIN_PICO   25   // LED_BUILTIN на обычной Pico  

static uint8_t s_bank = 0;   // текущий банк, задаёт af_led_set_bank()  
  
void af_led_set_bank(uint8_t bank) { s_bank = bank; }  
  
// Базовый цвет банка (R,G,B), доли от максимума — умножаются на яркость.  
// bank 0 (ручной) — зелёный, как раньше.  
static const uint8_t BANK_RGB[8][3] = {  
    {  0, 255,   0},   // 0 ручной — зелёный  
    {255, 255,   0},   // 1 — жёлтый  
    {255, 128,   0},   // 2 — оранжевый  
    {  0, 255, 255},   // 3 — циан  
    {  0, 128, 255},   // 4 — голубой  
    {255,   0, 255},   // 5 — пурпурный  
    {255, 255, 255},   // 6 — белый  
    {128,   0, 255},   // 7 — фиолетовый  
};

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
// Heartbeat: цвет вспышки = цвет банка; NR подмешивает синий, NT — красный.  
void af_led_poll(void) {  
    static uint32_t t0     = 0;  
    static uint8_t  last_g = 0xFF, last_r = 0xFF, last_b = 0xFF;  
  
    uint32_t ph = (millis() - t0) % 1000;  
    uint8_t g;   // огибающая яркости 0..100  
  
    if (ph < 200) {                       // набор 0 -> 15 (~6%)  
        g = (uint8_t)((ph * 15) / 200);  
    } else if (ph < 220) {                // вспышка 100 (~40%), 20 мс  
        g = 100;  
    } else if (ph < 500) {                // спад 100 -> 0  
        g = (uint8_t)(100 - ((ph - 220) * 100) / 280);  
    } else {                              // 500..1000 мс — тишина  
        g = 0;  
    }  
  
    // Базовый цвет банка, масштабированный огибающей.  
    const uint8_t* c = BANK_RGB[s_bank & 7];  
    uint8_t r = (uint8_t)((g * c[0]) / 255);  
    uint8_t gg = (uint8_t)((g * c[1]) / 255);  
    uint8_t b = (uint8_t)((g * c[2]) / 255);  
  
    // Подмес фильтров: NR -> +синий, NT -> +красный (оба -> фиолетовый).  
    if (af_dsp_nr_on())    b = (uint8_t)(b + (g * 128) / 255);  
    if (af_dsp_notch_on()) r = (uint8_t)(r + (g * 128) / 255);  
  
    if (r != last_r || gg != last_g || b != last_b) {  
        af_led_set(r, gg, b);  
        last_r = r; last_g = gg; last_b = b;  
    }  
}