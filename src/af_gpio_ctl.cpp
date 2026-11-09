// src/af_gpio_ctl.cpp  
// ============================================================================  
// af_gpio_ctl.cpp — управление функциями DSP по GPIO (Core 0)  
//  
// Логика: пин с INPUT_PULLUP, активный уровень LOW (замкнут на GND).  
//   AF_PIN_NT        — LOW -> автонотч NT включён  
//   AF_PIN_NR        — LOW -> шумоподавление NR включено  
//   AF_PIN_BAND_WIDE — LOW -> широкая полоса (AF_BAND_WIDE_*),  
//                      HIGH -> узкая полоса (AF_BAND_NARROW_*)  
//  
// Дебаунс: опрос каждые AF_GPIO_POLL_MS мс, смена состояния принимается  
// после AF_GPIO_STABLE подряд одинаковых чтений.  
//  
// CLI (nt/nr/band) продолжает работать; при следующем изменении пина  
// GPIO-контур перезапишет настройку — это ожидаемое поведение.  
// ============================================================================  
#include <Arduino.h>  
#include "af_gpio_ctl.h"  
#include "af_config.h"  
#include "af_dsp.h"  
  
// ---------------------------------------------------------------------------  
// Состояние одного управляющего пина  
// ---------------------------------------------------------------------------  
struct gpio_ctl_t {  
    uint8_t  pin;  
    bool     stable;        // последнее принятое (отдебаунсенное) состояние  
    bool     last_raw;      // последнее сырое чтение  
    uint8_t  stable_cnt;    // сколько чтений подряд last_raw держится  
};  
  
static gpio_ctl_t ctl_nt;  
static gpio_ctl_t ctl_nr;  
static gpio_ctl_t ctl_band;  
  
static uint32_t t_next_poll = 0;  
  
// ---------------------------------------------------------------------------  
// Применение состояния пина к DSP  
// ---------------------------------------------------------------------------  
static void apply_nt(bool grounded) {  
    af_dsp_set_notch(grounded);  
    Serial.printf("[GPIO] GP%u %s -> nt=%s\n",  
                  ctl_nt.pin, grounded ? "GND" : "open",  
                  grounded ? "on" : "off");  
}  
  
static void apply_nr(bool grounded) {  
    af_dsp_set_nr(grounded);  
    Serial.printf("[GPIO] GP%u %s -> nr=%s\n",  
                  ctl_nr.pin, grounded ? "GND" : "open",  
                  grounded ? "on" : "off");  
}  
  
static void apply_band(bool grounded) {  
    // Замкнут -> широкая полоса, разомкнут -> узкая.  
    const uint32_t lo = grounded ? AF_BAND_WIDE_LO_HZ   : AF_BAND_NARROW_LO_HZ;  
    const uint32_t hi = grounded ? AF_BAND_WIDE_HI_HZ   : AF_BAND_NARROW_HI_HZ;  
    af_dsp_set_band(lo, hi);  
    Serial.printf("[GPIO] GP%u %s -> band=%lu..%lu Hz\n",  
                  ctl_band.pin, grounded ? "GND" : "open",  
                  (unsigned long)lo, (unsigned long)hi);  
}  
  
// ---------------------------------------------------------------------------  
// Опрос одного пина с дебаунсом; при смене stable вызывает apply()  
// ---------------------------------------------------------------------------  
static void poll_pin(gpio_ctl_t* c, void (*apply)(bool)) {  
    const bool raw = (digitalRead(c->pin) == LOW);   // true = замкнут на GND  
  
    if (raw == c->last_raw) {  
        if (c->stable_cnt < AF_GPIO_STABLE) {  
            if (++c->stable_cnt >= AF_GPIO_STABLE && raw != c->stable) {  
                c->stable = raw;  
                apply(raw);  
            }  
        }  
    } else {  
        c->last_raw   = raw;  
        c->stable_cnt = 1;  
    }  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void af_gpio_ctl_init(void) {  
    ctl_nt   = { AF_PIN_NT,        false, false, 0 };  
    ctl_nr   = { AF_PIN_NR,        false, false, 0 };  
    ctl_band = { AF_PIN_BAND_WIDE, false, false, 0 };  
  
    pinMode(ctl_nt.pin,   INPUT_PULLUP);  
    pinMode(ctl_nr.pin,   INPUT_PULLUP);  
    pinMode(ctl_band.pin, INPUT_PULLUP);  
  
    // Начальное состояние — по текущему уровню на пинах, без дебаунса.  
    ctl_nt.stable   = ctl_nt.last_raw   = (digitalRead(ctl_nt.pin)   == LOW);  
    ctl_nr.stable   = ctl_nr.last_raw   = (digitalRead(ctl_nr.pin)   == LOW);  
    ctl_band.stable = ctl_band.last_raw = (digitalRead(ctl_band.pin) == LOW);  
  
    apply_nt(ctl_nt.stable);  
    apply_nr(ctl_nr.stable);  
    apply_band(ctl_band.stable);  
  
    Serial.printf("[GPIO] nt=GP%u nr=GP%u band=GP%u (LOW=on/wide)\n",  
                  ctl_nt.pin, ctl_nr.pin, ctl_band.pin);  
  
    t_next_poll = millis() + AF_GPIO_POLL_MS;  
}  
  
void af_gpio_ctl_poll(void) {  
    const uint32_t now = millis();  
    if ((int32_t)(now - t_next_poll) < 0) return;  
    t_next_poll = now + AF_GPIO_POLL_MS;  
  
    poll_pin(&ctl_nt,   apply_nt);  
    poll_pin(&ctl_nr,   apply_nr);  
    poll_pin(&ctl_band, apply_band);  
}