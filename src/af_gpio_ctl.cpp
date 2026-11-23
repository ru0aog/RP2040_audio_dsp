// src/af_gpio_ctl.cpp  
// ============================================================================  
// af_gpio_ctl.cpp — управление DSP по GPIO (Core 0)  
//  
//   GP2..GP4 — селектор банка настроек, 3 бита (LOW=1):  
//     000 = ручной режим (консоль), 001..111 = ячейки 1..7 (af_store_load).  
//   GP5 — LOW -> NR on,  GP6 — LOW -> NT on.  GP7 — LOW -> bypass on  
//   GP8 — LOW -> TX (PTT); при playback магнитофона — выход PTT (LOW).    
//   GP9 — LOW -> запись: тракт -> TX (микрофон), без PTT.    
//   GP10 — LOW -> передача сообщения: селектор GP13/GP28 выбирает 0..3.      
//     Строка CW.TXT непустая -> CW-посылка (cwtx), пустая -> голосовой слот.      
//     Тракт -> TX + PTT на GP8; CW добивается по cwtx_busy после отпускания.      
//   GP13/GP28 — селектор сообщения магнитофона/CW.TXT, 2 бита (LOW=1):    
//     код 0..3 = строка CW.TXT или голосовой слот 1..4 (этапы 3-4).    
//   Уровневая логика: LOW = активно, open = стоп.    
//   sw_ptt_active: GP8 занят как выход — входная ветка PTT отсечена.      
//   af_gpio_ptt_tx_on/off — то же для CLI (play voice_14 в TX+PTT).      
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
#include "af_rec.h"        // af_rec_start/stop/play/play_stop — GP9/GP10
  
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
static gpio_ctl_t ctl_rec;       // GP9 — LOW -> запись магнитофона  
static gpio_ctl_t ctl_play;      // GP10 — LOW -> воспроизведение
static gpio_ctl_t ctl_sel[2];    // GP13/GP28 — селектор сообщения, 2 бита (LOW=1)    
  
static uint32_t t_next_poll = 0;      
static uint8_t  bank_code   = 0; // текущий применённый код 0..7      
static uint8_t  last_bank   = 1; // последний загруженный банк 1..7 (для af_rec)      
static bool     sw_ptt_active = false;  // GP8 — выход PTT (playback магнитофона)      
    
// Текущий код банка с GP2..GP4 (0 = ручной режим). Для cli_stats.      
uint8_t af_gpio_bank_code(void) { return bank_code; }      
  
// Последний загруженный банк 1..7 — в т.ч. в manual-режиме (bank_code=0).    
// Для af_rec: слот магнитофона привязан к последнему активному банку.    
uint8_t af_gpio_last_bank(void) { return last_bank; }      

// Текущий код селектора сообщений GP13/GP28: 0..3 (LOW=1).    
// Индекс строки CW.TXT / голосового слота 1..4.    
uint8_t af_gpio_msg_sel(void) {    
    return (uint8_t)(ctl_sel[0].stable | (ctl_sel[1].stable << 1));    
}    

// Установить код активного банка извне (CLI 'load N') — чтобы stats      
// 'bank=' отражал последний применённый банк, а не только пины.      
// Пины при следующем изменении перезапишут через apply_bank.      
void af_gpio_set_bank_code(uint8_t code) {      
    bank_code = code;      
    if (code >= 1 && code <= 7) last_bank = code;   // CLI load N — тоже банк      
}      

// ---------------------------------------------------------------------------  
static void apply_nr(bool grounded) {      
    const bool was_on = af_dsp_nr_on();             // состояние до вызова    
    if (!af_dsp_set_nr(grounded)) {      
        Serial.printf("[GPIO] GP%u -> nr: rejected (%s)\n",          
                      ctl_nr.pin,  
                      af_dsp_cw_on() ? "CW on" : "NT on");          
        return;                                     // голос не играем — NR не включился      
    }      
    if (!grounded && !was_on) return;               // отпускание без включения — молчим    
    Serial.printf("[GPIO] GP%u %s -> nr=%s\n",      
                  ctl_nr.pin, grounded ? "GND" : "open",      
                  grounded ? "on" : "off");    
    if (grounded) af_voice_play(VOICE_NR_ON);   // "адаптивное шумоподавление включено"    
}  
  
static void apply_nt(bool grounded) {        
    const bool was_on = af_dsp_notch_on();          // состояние до вызова    
    int rc = af_dsp_set_notch(grounded);        
    if (rc == 3) {        
        Serial.printf("[GPIO] GP%u -> nt: rejected (CW on)\n", ctl_nt.pin);        
        return;                                     // голос не играем — NT не включился      
    }        
    if (!grounded && !was_on) return;               // отпускание без включения — молчим    
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
    if (sw_ptt_active) return;   // GP8 — выход PTT при playback, вход игнорируем    
    af_dsp_set_tx(grounded);      
    Serial.printf("[GPIO] GP%u %s -> tx=%s\n",      
                  ctl_ptt.pin, grounded ? "GND" : "open",      
                  grounded ? "on" : "off");      
}      
  
// Уровневая логика: LOW = запись идёт, open = стоп.  
// af_rec_start вернёт false, если уже пишет/играет — молча игнорируем.  
static void apply_rec(bool grounded) {    
    if (grounded) {    
        if (af_rec_is_playing()) return;          // play приоритетнее    
        af_dsp_set_tx(1);                          // тракт -> TX (микрофон), без PTT    
        if (af_rec_start())    
            Serial.printf("[GPIO] GP%u GND -> rec on (tx path)\n", ctl_rec.pin);    
        else    
            af_dsp_set_tx(0);                      // не записали — тракт обратно    
    } else {    
        if (af_rec_is_recording()) {    
            af_rec_stop();    
            af_dsp_set_tx(0);                      // тракт -> RX    
            Serial.printf("[GPIO] GP%u open -> rec off\n", ctl_rec.pin);    
        }    
    }    
}    
  
static void apply_play(bool grounded) {      
    if (grounded) {      
        if (af_rec_is_recording()) af_rec_stop();  // play приоритетнее записи      
        if (af_dsp_cwtx_busy()) return;            // CW-посылка ещё идёт — ждём      
  
        const uint8_t idx = af_gpio_msg_sel();     // GP13/GP28 -> 0..3      
  
        if (af_cw_msgs[idx][0] != '\0') {      
            // --- CW-вызов: строка CW.TXT -> посылка через cwtx ---      
            // af_dsp_cwtx_start сам ставит текст в очередь и поднимает TX.      
            if (!af_dsp_cwtx_start(af_cw_msgs[idx])) {      
                Serial.printf("[GPIO] GP%u -> cw msg %u: queue busy\n",      
                              ctl_play.pin, idx + 1);      
                return;      
            }      
            sw_ptt_active = true;                   // GP8 -> выход PTT      
            pinMode(ctl_ptt.pin, OUTPUT);      
            digitalWrite(ctl_ptt.pin, LOW);         // PTT активен (LOW)      
            Serial.printf("[GPIO] GP%u GND -> cw msg %u (tx+ptt)\n",      
                          ctl_play.pin, idx + 1);      
        } else {      
            // --- Голосовой слот: как раньше ---      
            af_dsp_set_tx(1);                       // тракт -> TX      
            sw_ptt_active = true;                   // GP8 -> выход PTT      
            pinMode(ctl_ptt.pin, OUTPUT);      
            digitalWrite(ctl_ptt.pin, LOW);         // PTT активен (LOW)      
            if (af_rec_play())      
                Serial.printf("[GPIO] GP%u GND -> play on (tx+ptt)\n", ctl_play.pin);      
            else {                                  // нечего играть — всё вернуть      
                digitalWrite(ctl_ptt.pin, HIGH);      
                pinMode(ctl_ptt.pin, INPUT_PULLUP);      
                sw_ptt_active = false;      
                af_dsp_set_tx(0);      
            }      
        }      
    } else {      
        // Отпускание: голос стопаем сразу; CW-посылку не обрываем —      
        // она добивается до конца, PTT/TX отпустит добиватель в poll      
        // по !af_dsp_cwtx_busy().      
        if (af_rec_is_playing()) {      
            af_rec_play_stop();      
            digitalWrite(ctl_ptt.pin, HIGH);      
            pinMode(ctl_ptt.pin, INPUT_PULLUP);     // GP8 обратно на вход PTT      
            sw_ptt_active = false;      
            af_dsp_set_tx(0);                       // тракт -> RX      
            Serial.printf("[GPIO] GP%u open -> play off\n", ctl_play.pin);      
        }      
    }      
}   

// --- API для CLI: play voice_14 в TX с PTT (аналог GP10) -------------------    
// Включает тракт TX + GP8 как выход PTT (LOW). Возвращает false,    
// если PTT уже занят (sw_ptt_active) — повторный вызов не нужен.    
bool af_gpio_ptt_tx_on(void) {    
    if (sw_ptt_active) return false;    
    af_dsp_set_tx(1);    
    sw_ptt_active = true;    
    pinMode(ctl_ptt.pin, OUTPUT);    
    digitalWrite(ctl_ptt.pin, LOW);    
    return true;    
}    
    
// Отпускает PTT/TX: GP8 обратно на вход, тракт в RX.    
void af_gpio_ptt_tx_off(void) {    
    if (!sw_ptt_active) return;    
    digitalWrite(ctl_ptt.pin, HIGH);    
    pinMode(ctl_ptt.pin, INPUT_PULLUP);    
    sw_ptt_active = false;    
    af_dsp_set_tx(0);    
}    
  
static void apply_bank(uint8_t code) {  
    bank_code = code;  
    if (code == 0) {    
        af_led_set_bank(0);          // зелёный — ручной режим  
        Serial.println("[GPIO] bank pins released -> manual mode (settings kept)"); 
        return;    
    }
    if (af_store_load(code)) {    
        last_bank = code;        // запомнить для af_rec (слот = последний банк)    
        af_led_set_bank(code);   // code = 0..7 с GP2..GP4    
        af_voice_play((af_voice_id_t)(VOICE_PROF_1 + code - 1));  // "профиль N"  
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
                else if (c == &ctl_rec)    apply_rec(raw);  
                else if (c == &ctl_play)   apply_play(raw);    
                else if (c == &ctl_sel[0] || c == &ctl_sel[1]) {    
                    Serial.printf("[GPIO] msg sel -> %u\n", af_gpio_msg_sel());    
                }    
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
    ctl_rec     = { AF_PIN_REC,   false, false, 0 };  
    ctl_play    = { AF_PIN_PLAY,  false, false, 0 };    
    ctl_sel[0]  = { AF_MSG_SEL0_PIN, false, false, 0 };    
    ctl_sel[1]  = { AF_MSG_SEL1_PIN, false, false, 0 };    
    pinMode(ctl_ptt.pin, INPUT_PULLUP);    
    ctl_ptt.stable = ctl_ptt.last_raw = (digitalRead(ctl_ptt.pin) == LOW);    
    pinMode(ctl_rec.pin, INPUT_PULLUP);  
    pinMode(ctl_play.pin, INPUT_PULLUP);
    pinMode(ctl_sel[0].pin, INPUT_PULLUP);    
    pinMode(ctl_sel[1].pin, INPUT_PULLUP);    
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
    // bank_code не трогаем: af_store_init мог уже автозагрузить    
    // последний сохранённый банк — не затираем его отображение.    
    af_led_set_bank(0);  
    delay(200);      // ждём, пока подтянутся линии с внешней ёмкостью  

    // Посев по установившимся уровням — после delay.  
    for (int i = 0; i < 3; i++)  
        ctl_bank[i].stable = ctl_bank[i].last_raw = (digitalRead(ctl_bank[i].pin) == LOW);  
    ctl_nr.stable     = ctl_nr.last_raw     = (digitalRead(ctl_nr.pin) == LOW);  
    ctl_nt.stable     = ctl_nt.last_raw     = (digitalRead(ctl_nt.pin) == LOW);  
    ctl_bypass.stable = ctl_bypass.last_raw = (digitalRead(ctl_bypass.pin) == LOW);
    ctl_ptt.stable    = ctl_ptt.last_raw    = (digitalRead(ctl_ptt.pin) == LOW);    
    ctl_rec.stable    = ctl_rec.last_raw    = (digitalRead(ctl_rec.pin) == LOW);  
    ctl_play.stable   = ctl_play.last_raw   = (digitalRead(ctl_play.pin) == LOW);    
    ctl_sel[0].stable = ctl_sel[0].last_raw = (digitalRead(ctl_sel[0].pin) == LOW);    
    ctl_sel[1].stable = ctl_sel[1].last_raw = (digitalRead(ctl_sel[1].pin) == LOW);    
    
    // Прогон дебаунса: stable отразит установившиеся уровни,  
    // а не переходку при старте (медленный подъём pull-up, дребезг).  
    for (uint8_t k = 0; k < AF_GPIO_STABLE + 1; k++) {  
        for (int i = 0; i < 3; i++) poll_pin(&ctl_bank[i]);  
        poll_pin(&ctl_nr);  
        poll_pin(&ctl_nt);  
        poll_pin(&ctl_bypass);
        poll_pin(&ctl_ptt);  
        poll_pin(&ctl_rec);  
        poll_pin(&ctl_play);
        poll_pin(&ctl_sel[0]);    
        poll_pin(&ctl_sel[1]);    
        delay(AF_GPIO_POLL_MS);    
    }  
  
    // Теперь читаем реальное состояние пинов и применяем.  
    apply_nt(ctl_nt.stable);  
    apply_nr(ctl_nr.stable);  
    apply_bypass(ctl_bypass.stable);
    apply_ptt(ctl_ptt.stable);  
    // rec/play при старте не применяем: запись с замкнутого GP9 при boot  
    // не нужна, а play всё равно вернёт false — слот пуст.  
    uint8_t code = 0;    
    for (int i = 0; i < 3; i++)    
        if (ctl_bank[i].stable) code |= (1 << i);    
    if (code != 0)    
        apply_bank(code);   // 000 = manual: настройки и bank_code не трогаем  
  
    Serial.printf("[GPIO] bank=GP%u..GP%u nr=GP%u nt=GP%u bypass=GP%u ptt=GP%u rec=GP%u play=GP%u sel=GP%u/GP%u (LOW=1/on)\n",      
                  AF_PIN_BANK0, AF_PIN_BANK2, AF_PIN_NR, AF_PIN_NT, AF_PIN_BYPASS, AF_PIN_PTT,    
                  AF_PIN_REC, AF_PIN_PLAY, AF_MSG_SEL0_PIN, AF_MSG_SEL1_PIN);      
  
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
    poll_pin(&ctl_rec);    
    poll_pin(&ctl_play);
    poll_pin(&ctl_sel[0]);
    poll_pin(&ctl_sel[1]);
  
    // Playback закончился по буферу (GP10 или CLI play voice_14) — отпускаем PTT/TX.          
    // Для CW-вызова ждём конца посылки: cwtx_busy() держит PTT до последнего          
    // элемента + послесвечение огибающей.          
    if (sw_ptt_active && !af_rec_is_playing() && !af_dsp_cwtx_busy()) {          
        af_gpio_ptt_tx_off();          
        Serial.println("[GPIO] play end -> ptt/tx released");          
    }          
    // Запись закончилась по буферу (авто-стоп), а GP9 ещё нажат — тракт в RX      
    if (ctl_rec.stable && !af_rec_is_recording() && !sw_ptt_active && af_dsp_tx_on()) {      
        af_dsp_set_tx(0);      
        Serial.println("[GPIO] rec end (buf full) -> tx released");      
    }      
}  