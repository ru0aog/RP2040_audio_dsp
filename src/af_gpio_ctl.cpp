// src/af_gpio_ctl.cpp  
// ============================================================================  
// af_gpio_ctl.cpp — управление DSP по GPIO (Core 0)  
//  
//   GP2..GP4 — селектор банка настроек, 3 бита (LOW=1):  
//     000 = ручной режим (консоль), 001..111 = ячейки 1..7 (af_store_load).  
//   GP5 — LOW -> NR on,  GP6 — LOW -> NT on.  
//   GP7 — LOW -> маяк: слот GP13/GP28 в его моде (CW/RTTY/JTTY из CW.TXT);  
//     повтор каждые 3 с пока LOW; open -> немедленный обрыв посылки.  
//   GP8 — LOW -> TX (PTT); при playback магнитофона — выход PTT (LOW).    
//   GP9 — LOW -> запись: тракт -> TX (микрофон), без PTT.    
//   GP10 — LOW -> PTT + воспроизведение голосового слота GP13/GP28 (0..3).  
//     Только голос: CW.TXT здесь не используется (текст — через маяк GP7).  
//     open -> стоп воспроизведения; PTT отпускает общий добиватель.    
//   GP13/GP28 — селектор сообщения магнитофона/CW.TXT, 2 бита (LOW=1):    
//     код 0..3 = строка CW.TXT или голосовой слот 1..4 (этапы 3-4).    
//   Уровневая логика: LOW = активно, open = стоп.    
//   sw_ptt_active: GP8 занят как выход — входная ветка PTT отсечена.      
//   af_gpio_ptt_tx_on/off — то же для CLI (play voice_14 в TX+PTT).      
//
//   GP11/GP12 — ямбический ключ (dit/dah, LOW=нажат) или прямой ключ;    
//     режим через 'cw key', тон 1000 Гц через cwtx_manual_*, PTT на GP8.    
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
#include "modes/cw_tx.h"   // cwtx_manual_*, cwtx_busy, cwtx_el_rem_free — ямбик  
#include "modes/rtty_tx.h"  // rttytx_abort — обрыв маяка mbcn
#include "modes/jtty_tx.h"  // jttytx_abort — обрыв маяка mbcn


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

// ---- Ямбический ключ (GP11=dit, GP12=dah, LOW=нажат) ----    
#define KEY_DEBOUNCE_MS   2u     // 2 опроса × 10 мс    
#define KEY_IAMBIC_A      1    
#define KEY_IAMBIC_B      2    
  
static uint8_t  key_mode = 0;        // 0=off, 1=iambic A, 2=iambic B, 3=straight    
static bool     key_tx = false;      // TX+PTT подняты ключом    
static bool     dit_mem, dah_mem;    // память педалей на время элемента      
static bool     key_last_dash;       // последний сыгранный элемент (для чередования)      
static bool     key_straight_down;   // прямой ключ: текущее состояние    
static uint8_t  dit_db, dah_db;      // счётчики антидребезга    
static bool     dit_lvl, dah_lvl;    // устойчивые уровни (true=нажато)    

// --- CW-маяк: циклическая посылка по таймеру (Core 0) ---    
static bool     beacon_on      = false;    
static uint32_t beacon_period  = 0;      // мс, 0 = выкл    
static uint32_t beacon_next_ms = 0;    
static char     beacon_text[AF_CW_MSG_LEN];   // текст посылки (как строка CW.TXT)    
  
// --- Голосовой маяк: слот текущего банка по таймеру ---    
static bool     vbcn_on       = false;    
static uint32_t vbcn_period   = 0;       // мс, 0 = выкл    
static uint32_t vbcn_next_ms  = 0;    
static bool     vbcn_playing  = false;   // посылка идёт — период от конца    

// --- Маяк GP7: слот GP13/GP28 в его моде, повтор 3 с, обрыв по open ---  
#define MBCN_GAP_MS  3000u              // пауза между посылками  
static bool     mbcn_on       = false;    
static bool     mbcn_tx       = false;  // посылка идёт    
static uint8_t  mbcn_sel      = 0;      // слот, зафиксированный на посылку    
static uint8_t  mbcn_mode     = 0;      // мода, зафиксированная на посылку    
static uint32_t mbcn_gap_ms   = 0;      // время следующей посылки    
static uint32_t mbcn_stage_t0 = 0;      // millis() старта посылки (для лога)    
static int8_t   mbcn_sel_ovr  = -1;     // слот из CLI: -1 = с пинов GP13/GP28 

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

static void key_tx_off(void);   // forward: определена ниже  
bool af_gpio_ptt_tx_on(void);   // forward: определена ниже  
void af_gpio_ptt_tx_off(void);  // forward: определена ниже  

// Режим ключа: 0=off, 1=iambic A, 2=iambic B, 3=straight. Для CLI 'cw key'.    
void af_gpio_key_mode(uint8_t m) {    
    if (m > 3) return;    
    if (key_tx) key_tx_off();    
    key_mode = m;    
    dit_mem = dah_mem = key_straight_down = false;    
    dit_db = dah_db = 0;    
    dit_lvl = dah_lvl = false;    
    Serial.printf("[KEY] mode=%u (0=off 1=iambicA 2=iambicB 3=straight)\n", m);    
}    
  
uint8_t af_gpio_key_mode_get(void) { return key_mode; }    

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

// --- Маяк GP7: слот в его моде, повтор 3 с; off = немедленный обрыв ---  
// on=true — запуск (слот NULL/пустой -> отказ), on=false — обрыв сразу.  
bool af_gpio_mbcn_set(bool on) {    
    if (on) {    
        if (mbcn_on) return true;    
        Serial.printf("[GPIO] GP%u GND -> mbcn on\n", ctl_bypass.pin);    
        const uint8_t sel = (mbcn_sel_ovr >= 0) ? (uint8_t)mbcn_sel_ovr    
                                                : af_gpio_msg_sel();    
        if (af_cw_mode[sel] == AF_CW_MODE_NULL || af_cw_msgs[sel][0] == '\0') {    
            Serial.printf("[GPIO] mbcn: slot %u empty\n", (unsigned)sel);    
            return false;    
        }    
        beacon_on = false;                       // взаимоисключение маяков  
        vbcn_on   = false;  
        mbcn_on = true; mbcn_tx = false;  
        mbcn_gap_ms = millis();  
        Serial.println(F("[GPIO] mbcn on"));  
        return true;  
    } else {  
        af_gpio_mbcn_abort();                    // off = немедленный обрыв  
        return true;  
    }  
}  
bool af_gpio_mbcn_on(void) { return mbcn_on; }    
    
// Запуск маяка из CLI с явным слотом: mbcn on <0..3>.    
// Пины GP13/GP28 игнорируются до стопа (abort сбрасывает override).    
bool af_gpio_mbcn_set_sel(uint8_t slot) {      
    if (slot >= AF_CW_MSGS) return false;      
    mbcn_sel_ovr = (int8_t)slot;      
    if (!af_gpio_mbcn_set(true)) {    
        mbcn_sel_ovr = -1;          // отказ — слот снова с пинов    
        return false;    
    }    
    return true;    
}      
    
// Немедленный обрыв посылки и стоп маяка (вызывается и из mbcn_set(false)).      
void af_gpio_mbcn_abort(void) {      
    if (!mbcn_on && !mbcn_tx) return;    
    mbcn_on = false;    
    mbcn_tx = false;    
    mbcn_sel_ovr = -1;               // слот снова с пинов GP13/GP28    
    cwtx_abort();  
    rttytx_abort();  
    jttytx_abort();  
    af_gpio_ptt_tx_off();          // PTT отпустить сразу  
    af_dsp_set_tx(false);          // посылки оборваны — тракт в RX сразу  
    Serial.println(F("[GPIO] mbcn abort"));  
}  
  
// GP7: LOW -> старт, open -> немедленный обрыв посылки  
static void apply_mbcn(bool grounded) {    
    af_gpio_mbcn_set(grounded);   // печать событий — внутри сеттера  
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
        if (af_dsp_cwtx_busy() || af_dsp_rttytx_busy() || af_dsp_jttytx_busy())    
            return;                                // посылка ещё идёт — ждём        
    
        const uint8_t idx = af_gpio_msg_sel();     // GP13/GP28 -> 0..3        
    
        // --- Голосовой слот: PTT + воспроизведение ---        
        af_dsp_set_tx(1);                           // тракт -> TX        
        sw_ptt_active = true;                       // GP8 -> выход PTT        
        pinMode(ctl_ptt.pin, OUTPUT);        
        digitalWrite(ctl_ptt.pin, LOW);             // PTT активен (LOW)        
        if (af_rec_play())        
            Serial.printf("[GPIO] GP%u GND -> play voice slot %u (tx+ptt)\n",        
                          ctl_play.pin, idx + 1);        
        else {                                      // нечего играть — всё вернуть        
            digitalWrite(ctl_ptt.pin, HIGH);        
            pinMode(ctl_ptt.pin, INPUT_PULLUP);        
            sw_ptt_active = false;        
            af_dsp_set_tx(0);        
            Serial.printf("[GPIO] GP%u -> play: slot %u empty\n",        
                          ctl_play.pin, idx + 1);        
        }        
    } else {        
        // Отпускание: голос стопаем сразу, PTT/TX возвращаем.        
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
                else if (c == &ctl_bypass) apply_mbcn(raw);  
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

// Антидребезг педали: ~5 мс устойчивого уровня    
static bool key_read(uint8_t pin, uint8_t* db, bool* lvl) {    
    bool raw = (gpio_get(pin) == 0);    
    if (raw == *lvl) { *db = 0; return false; }    
    if (++*db < KEY_DEBOUNCE_MS) return false;   // poll ~1 мс    
    *db = 0; *lvl = raw;    
    return true;                                 // фронт смены    
}    
  
static void key_tx_on(void) {      
    if (key_tx) return;      
    if (cwtx_busy()) return;          // не перебиваем текстовую посылку/маяк      
    if (!af_gpio_ptt_tx_on()) return; // PTT занят (play) — не начинаем      
    key_tx = true;        
    cwtx_manual_begin();        
    Serial.println("[KEY] tx on");      
}      
  
static void key_tx_off(void) {      
    if (!key_tx) return;      
    key_tx = false;      
    cwtx_manual_end();               // хвост доиграет, active -> false      
    // PTT/TX НЕ отпускаем здесь — добиватель в af_gpio_ctl_poll      
    // отпустит по !af_dsp_cwtx_busy(), когда огибающая доиграет.      
    Serial.println("[KEY] tx off (tail pending)");      
}      
  
// Ямбик: выбор следующего элемента по памяти педалей    
static void key_iambic_poll(void) {      
    if (dit_lvl)  dit_mem = true;      
    if (dah_lvl)  dah_mem = true;      
    
    if (!key_tx) {      
        if (dit_mem || dah_mem) key_tx_on();   // первый элемент -> TX+PTT+manual    
        if (!key_tx) return;                   // PTT занят / посылка идёт    
    }      
    
    if (cwtx_el_rem_free()) {   
        bool dash;    
        if (dit_mem && dah_mem)      // обе — чередуем: даём противоположную    
            dash = !key_last_dash;   // key_last_dash — последний сыгранный    
        else if (dit_mem)  dash = false;    
        else if (dah_mem)  dash = true;    
        else {    
            if (!cwtx_busy()) key_tx_off();   // всё отпущено и доиграно    
            return;    
        }    
        if (cwtx_manual_feed(dash)) {    
            key_last_dash = dash;    
            if (key_mode == KEY_IAMBIC_A) { dit_mem = dah_mem = false; }    
            else { if (dash) dah_mem = false; else dit_mem = false; } // Mode B    
        }    
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
    pinMode(AF_KEY_DIT_PIN, INPUT_PULLUP);   // ямбик dit    
    pinMode(AF_KEY_DAH_PIN, INPUT_PULLUP);   // ямбик dah    
    dit_lvl = dah_lvl = false;    
    dit_db = dah_db = 0;    
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
    // apply_mbcn при старте не вызываем: маяк с замкнутого GP7 при boot  
    // не нужен — как rec/play. Пин подействует при следующем изменении. 
    apply_ptt(ctl_ptt.stable);  
    // rec/play при старте не применяем: запись с замкнутого GP9 при boot  
    // не нужна, а play всё равно вернёт false — слот пуст.  
    uint8_t code = 0;    
    for (int i = 0; i < 3; i++)    
        if (ctl_bank[i].stable) code |= (1 << i);    
    if (code != 0)    
        apply_bank(code);   // 000 = manual: настройки и bank_code не трогаем  
  
    Serial.printf("[GPIO] bank=GP%u..GP%u nr=GP%u nt=GP%u mbcn=GP%u ptt=GP%u rec=GP%u play=GP%u sel=GP%u/GP%u (LOW=1/on)\n",  
                  AF_PIN_BANK0, AF_PIN_BANK2, AF_PIN_NR, AF_PIN_NT, AF_PIN_BYPASS, AF_PIN_PTT,    
                  AF_PIN_REC, AF_PIN_PLAY, AF_MSG_SEL0_PIN, AF_MSG_SEL1_PIN);      
  
    t_next_poll = millis() + AF_GPIO_POLL_MS;  
}  

void af_gpio_beacon_set(const char* text, uint32_t period_s) {    
    if (!text || !*text || period_s == 0) {    
        beacon_on = false;    
        Serial.println("[BEACON] off");    
        return;    
    }    
    if (vbcn_on) { vbcn_on = false; Serial.println("[VBCN] off (cw beacon on)"); }    
    if (mbcn_on) { af_gpio_mbcn_abort(); Serial.println("[MBCN] off (cw beacon on)"); }  
    strncpy(beacon_text, text, AF_CW_MSG_LEN - 1);    
    beacon_text[AF_CW_MSG_LEN - 1] = '\0';    
    beacon_period  = period_s * 1000u;    
    beacon_next_ms = millis();            // первая посылка сразу    
    beacon_on      = true;    
    Serial.printf("[BEACON] on: \"%s\" every %lu s\n", beacon_text, (unsigned long)period_s);    
}    
    
bool af_gpio_beacon_on(void) { return beacon_on; }    
  
// Голосовой маяк: играет слот текущего банка (rec_bank), пустой слот —    
// клип-заглушку voice_14 (это делает сам af_rec_play). Период — от конца    
// воспроизведения. Включение снимает CW-маяк и наоборот.    
void af_gpio_vbcn_set(uint32_t period_s) {    
    if (period_s == 0) {    
        vbcn_on = false;    
        vbcn_playing = false;    
        Serial.println("[VBCN] off");    
        return;    
    }    
    if (beacon_on) { beacon_on = false; Serial.println("[BEACON] off (vbcn on)"); }    
    if (mbcn_on) { af_gpio_mbcn_abort(); Serial.println("[MBCN] off (vbcn on)"); }    
    vbcn_period  = period_s * 1000u;      
    vbcn_next_ms = millis();              // первая посылка сразу    
    vbcn_playing = false;    
    vbcn_on      = true;    
    Serial.printf("[VBCN] on: bank slot every %lu s\n", (unsigned long)period_s);    
}    
    
bool af_gpio_vbcn_on(void) { return vbcn_on; }    


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

    // Ямбический ключ: опрос педалей и автомат элементов    
    if (key_mode != 0) {    
        key_read(AF_KEY_DIT_PIN, &dit_db, &dit_lvl);    
        key_read(AF_KEY_DAH_PIN, &dah_db, &dah_lvl);    
        if (key_mode == 3) {                     // straight key    
            bool down = dit_lvl || dah_lvl;      // любой контакт = ключ нажат    
            if (down != key_straight_down) {    
                key_straight_down = down;    
                    if (down) { key_tx_on(); cwtx_manual_key_down(); }      
                    else      { cwtx_manual_key_up(); }      
            }    
            if (!key_straight_down && key_tx && !cwtx_busy()) key_tx_off();      
        } else {    
            key_iambic_poll();    
        }    
    }    

    // Добиватель РТТ
    // Playback/посылка закончилась — отпускаем PTT/TX.  
    // !mbcn_on: в паузах между посылками маяка PTT держим.  
    // busy всех трёх передатчиков — чтобы не отпустить PTT посреди посылки.  
    if (sw_ptt_active && !mbcn_on && !af_rec_is_playing()  
        && !af_dsp_cwtx_busy() && !af_dsp_rttytx_busy() && !af_dsp_jttytx_busy()) {  
        af_gpio_ptt_tx_off();  
        Serial.println("[GPIO] play end -> ptt/tx released");  
    }  
    // Запись закончилась по буферу (авто-стоп), а GP9 ещё нажат — тракт в RX      
    if (ctl_rec.stable && !af_rec_is_recording() && !sw_ptt_active && af_dsp_tx_on()) {      
        af_dsp_set_tx(0);      
        Serial.println("[GPIO] rec end (buf full) -> tx released");      
    }      

    // CW-маяк: время пришло и тракт свободен -> очередная посылка    
    if (beacon_on && (int32_t)(millis() - beacon_next_ms) >= 0) {    
        if (!af_dsp_cwtx_busy() && !af_rec_is_playing() && !key_tx) {    
            if (af_dsp_cwtx_start(beacon_text)) {    
                af_gpio_ptt_tx_on();      // PTT на время посылки    
                beacon_next_ms = millis() + beacon_period;    
            } else {    
                beacon_next_ms = millis() + 1000u;   // очередь занята — повтор через 1 с    
            }    
        } else {    
            beacon_next_ms = millis() + 500u;        // тракт занят — отложить    
        }    
    }    
  
    // Голосовой маяк: период отсчитывается от КОНЦА воспроизведения.    
    // PTT/TX отпускает общий добиватель выше по !af_rec_is_playing().    
    if (vbcn_on) {    
        if (vbcn_playing) {    
            if (!af_rec_is_playing()) {          // доиграла — отсчёт паузы    
                vbcn_playing = false;    
                vbcn_next_ms = millis() + vbcn_period;    
            }    
        } else if ((int32_t)(millis() - vbcn_next_ms) >= 0) {    
            if (!af_dsp_cwtx_busy() && !af_rec_is_playing() && !key_tx    
                && !af_rec_is_recording()) {    
                if (af_gpio_ptt_tx_on() && af_rec_play()) {    
                    vbcn_playing = true;    
                    Serial.printf("# vbcn: playing bank %u\n",    
                                  (unsigned)af_gpio_last_bank());    
                } else {    
                    af_gpio_ptt_tx_off();        // нечего играть — вернуть PTT    
                    vbcn_next_ms = millis() + 1000u;    
                }    
            } else {    
                vbcn_next_ms = millis() + 500u;  // тракт занят — отложить    
            }    
        }    
    }    
    // Маяк GP7: слот GP13/GP28 в его моде (af_cw_mode), повтор каждые  
    // MBCN_GAP_MS пока пин LOW. Размыкание обрабатывает apply_mbcn ->  
    // af_gpio_mbcn_abort (немедленный обрыв) — здесь пин не проверяем.  
    if (mbcn_on) {    
        const uint8_t sel = (mbcn_sel_ovr >= 0) ? (uint8_t)mbcn_sel_ovr    
                                                : af_gpio_msg_sel();    
        const uint8_t mode = af_cw_mode[sel];    
        const char* msg = af_cw_msgs[sel];  
        const char* mname = (mode == AF_CW_MODE_CW)   ? "CW"  
                          : (mode == AF_CW_MODE_RTTY) ? "RTTY" : "JTTY";  
        if (mode == AF_CW_MODE_NULL || msg[0] == '\0') {   // слот стёрли на ходу  
            af_gpio_mbcn_abort();  
            Serial.println(F("[GPIO] mbcn: slot empty -> stop"));  
        } else if (mbcn_tx) {    
            // ждём конца посылки — по моде, зафиксированной при старте    
            // (смена GP13/GP28 на ходу не ломает проверку)    
            bool busy = (mbcn_mode == AF_CW_MODE_CW)   ? af_dsp_cwtx_busy()    
                      : (mbcn_mode == AF_CW_MODE_RTTY) ? af_dsp_rttytx_busy()    
                                                     : af_dsp_jttytx_busy();    
            if (!busy) {    
                mbcn_tx = false;    
                uint32_t dt = millis() - mbcn_stage_t0;    
                const char* dname = (mbcn_mode == AF_CW_MODE_CW)   ? "CW"    
                                  : (mbcn_mode == AF_CW_MODE_RTTY) ? "RTTY" : "JTTY";    
                Serial.printf("[GPIO] mbcn: %s done (%lu.%lu s)\n", dname,    
                              (unsigned long)(dt / 1000u),    
                              (unsigned long)((dt % 1000u) / 100u));    
                mbcn_gap_ms = millis() + MBCN_GAP_MS;   // пауза 3 с -> повтор    
            }    
        } else if ((int32_t)(millis() - mbcn_gap_ms) >= 0) {  
            if (!af_rec_is_playing() && !af_rec_is_recording() && !key_tx) {  
                bool ok;  
                if (mode == AF_CW_MODE_CW)        ok = af_dsp_cwtx_start(msg);  
                else if (mode == AF_CW_MODE_RTTY) ok = af_dsp_rttytx_start(msg);  
                else                              ok = af_dsp_jttytx_start(msg);  
                if (ok) {    
                    af_gpio_ptt_tx_on();            // PTT LOW на посылку    
                    mbcn_tx = true;    
                    mbcn_sel  = sel;                // зафиксировать слот/моду    
                    mbcn_mode = mode;               // на время посылки    
                    mbcn_stage_t0 = millis();    
                    Serial.printf("[GPIO] mbcn: slot %u %s tx '%s'\n",  
                                  (unsigned)sel, mname, msg);  
                } else {  
                    Serial.printf("[GPIO] mbcn: %s start failed, retry in 1s\n", mname);  
                    mbcn_gap_ms = millis() + 1000u; // занято — повтор через 1 с  
                }  
            } else {  
                mbcn_gap_ms = millis() + 500u;  
            }  
        }  
    }  
}    

bool    af_gpio_ptt_active(void) { return sw_ptt_active; }  
bool    af_gpio_key_tx(void)     { return key_tx; }  
uint8_t af_gpio_key_mode(void)   { return key_mode; }  
