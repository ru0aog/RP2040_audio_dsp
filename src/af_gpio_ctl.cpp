// src/af_gpio_ctl.cpp  
// ============================================================================  
// af_gpio_ctl.cpp — управление DSP по GPIO (Core 0)  
//  
//   GP2..GP4 — селектор банка настроек, 3 бита (LOW=1):  
//     000 = ручной режим (консоль), 001..111 = ячейки 1..7 (af_store_load).  
//   GP5 — LOW -> NR on,  GP6 — LOW -> NT on.  GP7 — LOW -> bypass on
//  
// Опрос каждые AF_GPIO_POLL_MS мс, дебаунс AF_GPIO_STABLE чтений.  
// Консольные команды имеют приоритет, но только до следующего изменения  
// пинов — тогда применяется состояние пинов/ячейка.  
// ============================================================================  
#include <Arduino.h>  
#include "af_gpio_ctl.h"  
#include "af_config.h"  
#include "af_dsp.h"  
#include "af_store.h"
#include "af_led.h"
#include "af_voice.h"
  
struct gpio_ctl_t {  
    uint8_t  pin;  
    bool     stable;  
    bool     last_raw;  
    uint8_t  stable_cnt;  
};  
  
static gpio_ctl_t ctl_bank[3];   // GP2,GP3,GP4 — биты 0,1,2  
static gpio_ctl_t ctl_nr;        // GP5  
static gpio_ctl_t ctl_nt;        // GP6
static gpio_ctl_t ctl_bypass;    // GP7
static gpio_ctl_t ctl_ptt;       // GP8 — LOW -> TX  
  
static uint32_t t_next_poll = 0;  
static uint8_t  bank_code   = 0; // текущий применённый код 0..7  
  
// ---------------------------------------------------------------------------  
static void apply_nr(bool grounded) {    
    if (af_dsp_set_nr(grounded)) {  
        Serial.printf("[GPIO] GP%u %s -> nr=%s\n",    
                      ctl_nr.pin, grounded ? "GND" : "open",    
                      grounded ? "on" : "off");  
        if (grounded) af_voice_play(VOICE_NR_ON);   // "адаптивное шумоподавление включено"  
    } else {  
        Serial.printf("[GPIO] nr: rejected (NT on)\n");    
    }  
}
  
static void apply_nt(bool grounded) {    
    int rc = af_dsp_set_notch(grounded);    
    Serial.printf("[GPIO] GP%u %s -> nt=%s\n",    
                  ctl_nt.pin, grounded ? "GND" : "open",    
                  grounded ? "on" : "off");    
    if (rc == 2) Serial.println("[GPIO] note: NR off (NT priority)");    
    if (grounded) af_voice_play(VOICE_NT_ON);   // "автоматический режекторный фильтр включен"  
}

static void apply_bypass(bool grounded) {    
    af_dsp_set_bypass(grounded);    
    Serial.printf("[GPIO] GP%u %s -> bypass=%s\n",    
                  ctl_bypass.pin, grounded ? "GND" : "open",    
                  grounded ? "on" : "off");    
    if (grounded) af_voice_play(VOICE_BYPASS_ON);   // "режим обхода включен"  
}

static void apply_ptt(bool grounded) {  
    af_dsp_set_tx(grounded);  
    Serial.printf("[GPIO] GP%u %s -> tx=%s\n",  
                  ctl_ptt.pin, grounded ? "GND" : "open",  
                  grounded ? "on" : "off");  
}  
  
static void apply_bank(uint8_t code) {  
    bank_code = code;  
    if (code == 0) {    
        af_led_set_bank(0);          // зелёный — ручной режим  
        Serial.println("[GPIO] bank=0 -> manual mode (console)");    
        return;    
    }
    if (af_store_load(code)) {
        af_led_set_bank(code);   // code = 0..7 с GP2..GP4
        if (!af_dsp_agc_on()) af_voice_play(VOICE_AGC_OFF);
        Serial.printf("[GPIO] bank=%u -> loaded band=%lu..%lu cw=%s f0=%lu bw=%lu\n",  
                      code,  
                      (unsigned long)af_dsp_band_lo_hz(),  
                      (unsigned long)af_dsp_band_hi_hz(),  
                      af_dsp_cw_on() ? "on" : "off",  
                      (unsigned long)af_dsp_cw_freq(),  
                      (unsigned long)af_dsp_cw_bw());
        if (af_dsp_cw_on()) af_voice_play(VOICE_CW_ON);   // "телеграфный фильтр включен"
    } else {  
        Serial.printf("[GPIO] bank=%u -> empty, ignored\n", code);  
    }  
}  
  
// ---------------------------------------------------------------------------  
static void poll_pin(gpio_ctl_t* c) {  
    const bool raw = (digitalRead(c->pin) == LOW);  
    if (raw == c->last_raw) {  
        if (c->stable_cnt < AF_GPIO_STABLE) {  
            if (++c->stable_cnt >= AF_GPIO_STABLE && raw != c->stable) {  
                c->stable = raw;  
                if (c == &ctl_nr)          apply_nr(raw);  
                else if (c == &ctl_nt)     apply_nt(raw);  
                else if (c == &ctl_bypass) apply_bypass(raw);
                else if (c == &ctl_ptt)    apply_ptt(raw);  
                else {  // банковый пин — пересчитать код и применить  
                    uint8_t code = 0;  
                    for (int i = 0; i < 3; i++)  
                        if (ctl_bank[i].stable) code |= (1 << i);  
                    apply_bank(code);  
                }  
            }  
        }  
    } else {  
        c->last_raw   = raw;  
        c->stable_cnt = 1;  
    }  
}  
  
// ---------------------------------------------------------------------------  
void af_gpio_ctl_init(void) {  
    ctl_bank[0] = { AF_PIN_BANK0, false, false, 0 };  
    ctl_bank[1] = { AF_PIN_BANK1, false, false, 0 };  
    ctl_bank[2] = { AF_PIN_BANK2, false, false, 0 };  
    ctl_nr      = { AF_PIN_NR,    false, false, 0 };  
    ctl_nt      = { AF_PIN_NT,    false, false, 0 };
    ctl_bypass  = { AF_PIN_BYPASS, false, false, 0 };
    ctl_ptt     = { AF_PIN_PTT,   false, false, 0 };  
    pinMode(ctl_ptt.pin, INPUT_PULLUP);  
    ctl_ptt.stable = ctl_ptt.last_raw = (digitalRead(ctl_ptt.pin) == LOW);  
    pinMode(ctl_bypass.pin, INPUT_PULLUP);  
    ctl_bypass.stable = ctl_bypass.last_raw = (digitalRead(ctl_bypass.pin) == LOW);  
  
    for (int i = 0; i < 3; i++) {  
        pinMode(ctl_bank[i].pin, INPUT_PULLUP);  
        ctl_bank[i].stable = ctl_bank[i].last_raw = (digitalRead(ctl_bank[i].pin) == LOW);  
    }  
    pinMode(ctl_nr.pin, INPUT_PULLUP);  
    pinMode(ctl_nt.pin, INPUT_PULLUP);  
    ctl_nr.stable = ctl_nr.last_raw = (digitalRead(ctl_nr.pin) == LOW);  
    ctl_nt.stable = ctl_nt.last_raw = (digitalRead(ctl_nt.pin) == LOW);  
  
    // Старт всегда с ручного режима — переходные состояния пинов  
    // при подаче питания не должны дёргать банки.  
    apply_bank(0);
    delay(200);      // ждём, пока подтянутся линии с внешней ёмкостью  

    // Посев по установившимся уровням — после delay.  
    for (int i = 0; i < 3; i++)  
        ctl_bank[i].stable = ctl_bank[i].last_raw = (digitalRead(ctl_bank[i].pin) == LOW);  
    ctl_nr.stable     = ctl_nr.last_raw     = (digitalRead(ctl_nr.pin) == LOW);  
    ctl_nt.stable     = ctl_nt.last_raw     = (digitalRead(ctl_nt.pin) == LOW);  
    ctl_bypass.stable = ctl_bypass.last_raw = (digitalRead(ctl_bypass.pin) == LOW);
    ctl_ptt.stable    = ctl_ptt.last_raw    = (digitalRead(ctl_ptt.pin) == LOW);  
    
    // Прогон дебаунса: stable отразит установившиеся уровни,  
    // а не переходку при старте (медленный подъём pull-up, дребезг).  
    for (uint8_t k = 0; k < AF_GPIO_STABLE + 1; k++) {  
        for (int i = 0; i < 3; i++) poll_pin(&ctl_bank[i]);  
        poll_pin(&ctl_nr);  
        poll_pin(&ctl_nt);  
        poll_pin(&ctl_bypass);
        poll_pin(&ctl_ptt);
        delay(AF_GPIO_POLL_MS);  
    }  
  
    // Теперь читаем реальное состояние пинов и применяем.  
    apply_nt(ctl_nt.stable);  
    apply_nr(ctl_nr.stable);  
    apply_bypass(ctl_bypass.stable);
    apply_ptt(ctl_ptt.stable);
    uint8_t code = 0;  
    for (int i = 0; i < 3; i++)  
        if (ctl_bank[i].stable) code |= (1 << i);  
    apply_bank(code);
  
    Serial.printf("[GPIO] bank=GP%u..GP%u nr=GP%u nt=GP%u bypass=GP%u ptt=GP%u (LOW=1/on)\n",  
                  AF_PIN_BANK0, AF_PIN_BANK2, AF_PIN_NR, AF_PIN_NT, AF_PIN_BYPASS, AF_PIN_PTT);  
  
    t_next_poll = millis() + AF_GPIO_POLL_MS;  
}  
  
void af_gpio_ctl_poll(void) {  
    const uint32_t now = millis();  
    if ((int32_t)(now - t_next_poll) < 0) return;  
    t_next_poll = now + AF_GPIO_POLL_MS;  
  
    for (int i = 0; i < 3; i++) poll_pin(&ctl_bank[i]);  
    poll_pin(&ctl_nr);  
    poll_pin(&ctl_nt);
    poll_pin(&ctl_bypass);
    poll_pin(&ctl_ptt);
}