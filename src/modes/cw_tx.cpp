// src/modes/cw_tx.cpp — CW-передатчик: текст -> Морзе -> тон.  
//  
// Огибающая посылки — raised cosine (колокол), фронт/спад CWTX_RISE_MS:  
// убирает щелчки ключевания и внеполосные лепестки.  
// Тон — NCO по LUT синуса (256 отсчётов/период), фаза непрерывна  
// между посылками — нет скачков фазы на границах элементов.  
//  
// Тайминги PARIS: точка = 1, тире = 3, межэлемент = 1, межбуква = 3,  
// межслово = 7 (в единицах dot_samples = Fs*1.2/wpm).  
//  
// Разделение по ядрам:  
//   Core 0: cwtx_init (float: sinf для LUT), cwtx_send (текст -> очередь),  
//           cwtx_poll — вызывать из loop(): подаёт следующий символ.  
//   Core 1: cwtx_sample() — только int32/LUT, вызывается на каждый  
//           выходной отсчёт в TX-ветке af_dsp.  
// ============================================================================  
#include "cw_tx.h"  
#include "../af_config.h"     // AF_FS_OUT_HZ  
  
// ---------------------------------------------------------------------------  
// Таблица Морзе: битовое поле элементов, MSB-first, 1=тире 0=точка.  
// ---------------------------------------------------------------------------  
struct cwtx_sym_t { char ch; uint8_t len; uint8_t bits; };  
  
static const cwtx_sym_t CWTX_TAB[] = {  
    {'A',2,0b01},{'B',4,0b1000},{'C',4,0b1010},{'D',3,0b100},{'E',1,0b0},    
    {'F',4,0b0010},{'G',3,0b110},{'H',4,0b0000},{'I',2,0b00},{'J',4,0b0111},    
    {'K',3,0b101},{'L',4,0b0100},{'M',2,0b11},{'N',2,0b10},{'O',3,0b111},      
    {'P',4,0b0110},{'Q',4,0b1101},{'R',3,0b010},{'S',3,0b000},{'T',1,0b1},    
    {'U',3,0b001},{'V',4,0b0001},{'W',3,0b011},{'X',4,0b1001},{'Y',4,0b1011},    
    {'Z',4,0b1100},    
    {'0',5,0b11111},{'1',5,0b01111},{'2',5,0b00111},{'3',5,0b00011},  
    {'4',5,0b00001},{'5',5,0b00000},{'6',5,0b10000},{'7',5,0b11000},  
    {'8',5,0b11100},{'9',5,0b11110},  
    {'.',6,0b010101},{',',6,0b110011},{'?',6,0b001100},{'/',5,0b10010},  
    {'=',5,0b10001},{'+',5,0b01010},{'-',6,0b100001}, // -....-    
    {' ',0,0},  
};  
#define CWTX_TAB_LEN  (sizeof(CWTX_TAB)/sizeof(CWTX_TAB[0]))  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static int16_t  cwtx_lut[CWTX_LUT_LEN];   // Q15 синус  
static uint32_t cwtx_nco_step;            // шаг фазы: tone_hz * 2^32 / Fs  
static uint32_t cwtx_phase;               // аккумулятор фазы (непрерывный)  
  
static uint32_t cwtx_dot;                 // длительность точки, отсчётов  
static uint32_t cwtx_rise;                // фронт/спад, отсчётов  
  
// Очередь текста (Core 0 пишет, Core 0 же читает — SPSC не нужен,  
// разбор символов идёт в cwtx_poll на Core 0)  
#define CWTX_TEXT_LEN  96u  
static char     cwtx_text[CWTX_TEXT_LEN];  
static volatile uint8_t cwtx_tlen;        // символов в очереди  
static volatile uint8_t cwtx_tpos;        // текущий символ  
  
// Состояние элемента (читается Core 1 — volatile)  
static volatile bool     cwtx_key;        // ключ нажат (внутри элемента)  
static volatile uint32_t cwtx_el_rem;     // отсчётов до конца текущей фазы  
static volatile uint8_t  cwtx_bits;       // биты текущего символа  
static volatile uint8_t  cwtx_blen;       // элементов в символе  
static volatile uint8_t  cwtx_bpos;       // текущий элемент  
static volatile uint8_t  cwtx_gap;        // пауза после символа, в точках (1/3/7)  
static volatile bool     cwtx_active;     // идёт передача    
static volatile bool     cwtx_tail;       // хвост после последнего элемента подан    
  
// Огибающая: позиция внутри элемента для raised-cosine  
static volatile uint32_t cwtx_el_pos;     // отсчётов от начала элемента  
static volatile uint32_t cwtx_el_len;     // полная длина элемента  
  
// ---------------------------------------------------------------------------  
// Огибающая raised cosine, Q15. pos — позиция в элементе [0..len).  
// Фронт: 0..rise -> 0..32767; спад: len-rise..len -> 32767..0; середина 32767.  
// ---------------------------------------------------------------------------  
static inline int32_t cwtx_env(uint32_t pos, uint32_t len) {  
    if (pos < cwtx_rise) {  
        // (1 - cos(pi * pos/rise)) / 2  — через LUT синуса:  
        // sin(pi/2 * pos/rise)^2 = (1-cos(pi*pos/rise))/2  
        uint32_t idx = (pos * (CWTX_LUT_LEN / 4)) / cwtx_rise;   // 0..64 = 0..pi/2  
        int32_t s = cwtx_lut[idx];  
        return (s * s) >> 15;  
    }  
    if (pos >= len - cwtx_rise) {  
        uint32_t p = len - pos;                                   // rise..0  
        uint32_t idx = (p * (CWTX_LUT_LEN / 4)) / cwtx_rise;  
        int32_t s = cwtx_lut[idx];  
        return (s * s) >> 15;  
    }  
    return 32767;  
}  
  
// ---------------------------------------------------------------------------  
// Core 1: следующий отсчёт. Вызывается на каждый выходной сэмпл в TX-режиме.  
// ---------------------------------------------------------------------------  
int16_t cwtx_sample(void) {    
    if (cwtx_el_rem == 0) return 0;   // ждём, пока Core 0 подаст следующую фазу    
    cwtx_el_rem--;    
  
    // NCO: непрерывная фаза  
    cwtx_phase += cwtx_nco_step;  
    int32_t tone = cwtx_lut[(cwtx_phase >> 24) & (CWTX_LUT_LEN - 1)];  
  
    if (!cwtx_key) return 0;   // пауза — тишина, фаза крутится  
  
    int32_t env = cwtx_env(cwtx_el_pos, cwtx_el_len);  
    cwtx_el_pos++;  
    return (int16_t)((tone * env) >> 15);  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: разбор очереди — подача элементов/пауз в cwtx_el_rem.  
// Вызывать из loop() каждую итерацию.  
// ---------------------------------------------------------------------------  
void cwtx_poll(void) {  
    if (!cwtx_active) return;  
    if (cwtx_el_rem != 0) return;          // Core 1 ещё проигрывает фазу  
  
    // пауза после символа?  
    if (cwtx_gap) {  
        cwtx_key = false;  
        cwtx_el_rem = cwtx_gap * cwtx_dot;  
        cwtx_gap = 0;  
        return;  
    }  
  
    // следующий элемент текущего символа  
    if (cwtx_bpos < cwtx_blen) {  
        bool dash = (cwtx_bits >> (cwtx_blen - 1 - cwtx_bpos)) & 1;  
        cwtx_key = true;  
        cwtx_el_len = (dash ? 3u : 1u) * cwtx_dot;  
        cwtx_el_rem = cwtx_el_len;  
        cwtx_el_pos = 0;  
        cwtx_bpos++;  
        cwtx_gap = (cwtx_bpos < cwtx_blen) ? 1 : 3;   // межэлемент/межбуква  
        return;  
    }  
  
    // следующий символ  
    if (cwtx_tpos < cwtx_tlen) {  
        char c = cwtx_text[cwtx_tpos++];  
        if (c >= 'a' && c <= 'z') c -= 32;  
        const cwtx_sym_t* s = NULL;  
        for (uint8_t i = 0; i < CWTX_TAB_LEN; i++)  
            if (CWTX_TAB[i].ch == c) { s = &CWTX_TAB[i]; break; }  
        if (s == NULL) { cwtx_gap = 3; return; }      // неизвестный — как буква-пауза  
        if (s->len == 0) { cwtx_gap = 7; return; }    // пробел — межслово  
        cwtx_bits = s->bits;  
        cwtx_blen = s->len;  
        cwtx_bpos = 0;  
        return;                                       // элементы подаст след. вызов  
    }  
  
    // очередь пуста — хвост ~7 точек тишины, потом конец передачи.    
    // Отдельный флаг: cwtx_gap нельзя использовать как метку —    
    // ветка паузы выше съест её и хвост зациклится.    
    if (!cwtx_tail) {    
        cwtx_tail = true;    
        cwtx_key = false;    
        cwtx_el_rem = 7 * cwtx_dot;   // тишина-хвост, доигрывается Core 1    
    } else {    
        cwtx_active = false;    
        cwtx_key = false;    
    }    
}    
  
// ---------------------------------------------------------------------------  
// Core 0: API  
// ---------------------------------------------------------------------------  
void cwtx_init(uint32_t tone_hz) {  
    for (uint32_t i = 0; i < CWTX_LUT_LEN; i++)  
        cwtx_lut[i] = (int16_t)(sinf(2.0f * (float)M_PI * (float)i / CWTX_LUT_LEN) * 32767.0f);  
    cwtx_nco_step = (uint32_t)(((uint64_t)tone_hz << 32) / AF_FS_OUT_HZ);  
    cwtx_phase = 0;  
    cwtx_rise = (AF_FS_OUT_HZ * CWTX_RISE_MS) / 1000u;  
    cwtx_set_wpm(20);    
    cwtx_active = false;    
    cwtx_tail = false;    
    cwtx_tlen = cwtx_tpos = 0;    
    Serial.printf("[CWTX] init tone=%lu Hz rise=%lu ms\n",  
                  (unsigned long)tone_hz, (unsigned long)CWTX_RISE_MS);  
}  
  
void cwtx_set_wpm(uint8_t wpm) {  
    if (wpm < 5) wpm = 5;  
    if (wpm > 60) wpm = 60;  
    cwtx_dot = (uint32_t)((AF_FS_OUT_HZ * 1.2f) / (float)wpm);  
}  
  
bool cwtx_send(const char* text) {  
    if (cwtx_active) return false;  
    uint8_t n = 0;  
    while (text[n] && n < CWTX_TEXT_LEN - 1) { cwtx_text[n] = text[n]; n++; }  
    if (n == 0) return false;  
    cwtx_tlen = n;  
    cwtx_tpos = 0;  
    cwtx_bpos = cwtx_blen = 0;  
    cwtx_gap = 0;  
    cwtx_el_rem = 0;    
    cwtx_key = false;    
    cwtx_tail = false;    
    cwtx_active = true;    
    return true;    
}  
  
bool cwtx_busy(void) { return cwtx_active; }  
  
void cwtx_abort(void) {    
    cwtx_active = false;    
    cwtx_key = false;    
    cwtx_el_rem = 0;    
    cwtx_tail = false;    
    cwtx_tlen = cwtx_tpos = 0;    
}    

void cwtx_set_tone(uint32_t tone_hz) {  
    cwtx_nco_step = (uint32_t)(((uint64_t)tone_hz << 32) / AF_FS_OUT_HZ);  
}  
