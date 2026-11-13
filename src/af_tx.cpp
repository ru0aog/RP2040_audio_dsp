// src/af_tx.cpp    
// ============================================================================    
// af_tx.cpp — передающий тракт SSB (Core 1)    
//    
// Цепочка на AF_FS_OUT_HZ (~15.464 ksps):    
//   y -> полосовой FIR (tx_lo..tx_hi, 300..2700 по умолч.)    
//     -> компрессор (EMA-огибающая, gain = (thr/env)^(1-1/ratio))    
//     -> клиппер (жёсткий, порог tx_clip)    
//     -> постфильтр (тот же FIR — срезает гармоники клиппинга)    
//     -> makeup-gain -> int16    
//    
// Цель — «плотный» речевой сигнал: компрессия поднимает среднюю    
// мощность, клиппер добавляет ещё ~4-6 дБ, постфильтр убирает    
// гармоники клиппинга выше tx_hi — эфир не грязнится.    
// Весь горячий путь — целочисленный, int64-деление только в    
// компрессоре (~0.6% Core 1).    
// ============================================================================    
#include <Arduino.h>    
#include <math.h>    
#include <string.h>    
#include "af_tx.h"    
#include "af_config.h"    
    
// ---------------------------------------------------------------------------    
// Два экземпляра полосового FIR (пред- и постфильтр), структура как в    
// af_dsp.cpp: sinc + Блэкман-Харрис, Q15, симметричная свёртка.    
// ---------------------------------------------------------------------------    
#define TX_FIR_N 127    
    
static volatile uint32_t tx_lo_hz = AF_TX_LO_HZ_DEFAULT;    
static volatile uint32_t tx_hi_hz = AF_TX_HI_HZ_DEFAULT;    
    
static int16_t tx_fir_taps[TX_FIR_N];    
    
typedef struct {    
    int32_t buf[TX_FIR_N];    
    int     idx;    
} tx_fir_t;    
    
static tx_fir_t tx_pre, tx_post;    
    
static void tx_fir_design(void) {    
    const int   M     = (TX_FIR_N - 1) / 2;    
    const float fc_hi = (float)tx_hi_hz / AF_FS_OUT_HZ;    
    const float fc_lo = (float)tx_lo_hz / AF_FS_OUT_HZ;    
    float h[TX_FIR_N];    
    float sum = 0.0f;    
    for (int n = 0; n < TX_FIR_N; n++) {    
        const int m = n - M;    
        float s_hi = (m == 0) ? (2.0f * fc_hi)    
                              : sinf(2.0f * PI * fc_hi * m) / (PI * m);    
        float s_lo = (m == 0) ? (2.0f * fc_lo)    
                              : sinf(2.0f * PI * fc_lo * m) / (PI * m);    
        const float a = 2.0f * PI * n / (TX_FIR_N - 1);    
        float w = 0.35875f - 0.48829f * cosf(a)    
                         + 0.14128f * cosf(2.0f * a)    
                         - 0.01168f * cosf(3.0f * a);    
        h[n] = (s_hi - s_lo) * w;    
        sum += fabsf(h[n]);    
    }    
    const float norm = (sum > 0.0f) ? (2.0f / sum) : 1.0f;    
    for (int n = 0; n < TX_FIR_N; n++)    
        tx_fir_taps[n] = (int16_t)lroundf(h[n] * norm * 32768.0f);    
}    
    
static inline int32_t __not_in_flash_func(tx_fir_push)(tx_fir_t* f, int32_t x) {    
    f->buf[f->idx] = x;    
    if (++f->idx >= TX_FIR_N) f->idx = 0;    
    
    int64_t acc = 0;    
    int lo = f->idx;    
    int hi = f->idx + TX_FIR_N - 1;    
    if (hi >= TX_FIR_N) hi -= TX_FIR_N;    
    for (int k = 0; k < TX_FIR_N / 2; k++) {    
        acc += (int32_t)tx_fir_taps[k] * (f->buf[lo] + f->buf[hi]);    
        if (++lo >= TX_FIR_N) lo = 0;    
        if (--hi < 0)         hi = TX_FIR_N - 1;    
    }    
    acc += (int32_t)tx_fir_taps[TX_FIR_N / 2] * f->buf[lo];    
    return (int32_t)((acc + (1 << 14)) >> 15);    
}    
    
// ---------------------------------------------------------------------------    
// Компрессор: EMA-огибающая |x| (атака/спад), gain по степенному закону.    
// env > thr: gain = (thr/env)^(1 - 1/ratio) — при ratio=1 обход,    
// при ratio->inf лимитер. Реализовано кусочно: gain_q15 =    
//   env <= thr        -> 32768 (x1.0)    
//   env >  thr        -> (thr/env) * 32768, приподнято степенью:    
// упрощённо — линейная компрессия gain = thr/env с подмесом (1-1/ratio):    
//   gain = 1 - (1 - thr/env) * (1 - 1/ratio)    
// При ratio=1 -> gain=1; при большом ratio -> gain=thr/env (лимитер).    
// ---------------------------------------------------------------------------    
static volatile uint32_t tx_ratio_x10 = AF_TX_RATIO_X10_DEFAULT;  // x10    
static volatile uint32_t tx_clip_thr  = AF_TX_CLIP_DEFAULT;       // 0..32767   
static uint32_t tx_agc_target = AF_TX_AGC_TARGET;  // целевой уровень АРУ  
static uint32_t tx_gate_thr   = AF_TX_GATE_THR;    // порог гейта (0 = off)  
static uint32_t agc_env       = 0;                 // EMA огибающей АРУ, Q16  
static uint32_t gate_gain     = 0;                 // gain гейта, Q16 (0..65536)  
    
static int64_t tx_env_q16 = (int64_t)256 << 16;   // EMA огибающей, Q16    
    
// Порог компрессора — фиксированный, ~25% шкалы.    
#define TX_COMP_THR   8192    
    
static inline int32_t __not_in_flash_func(tx_compress)(int32_t x) {    
    int64_t ax  = (x < 0) ? -x : x;    
    int64_t tgt = ax << 16;    
    int64_t da  = tgt - tx_env_q16;    
    // атака ~5 мс, спад ~150 мс при 15.464 ksps    
    tx_env_q16 += (da * (da > 0 ? (int64_t)AF_TX_ENV_ATT_Q24    
                                : (int64_t)AF_TX_ENV_DEC_Q24)) >> 24;    
    if (tx_env_q16 < ((int64_t)1 << 16)) tx_env_q16 = (int64_t)1 << 16;    
    
    int32_t env = (int32_t)(tx_env_q16 >> 16);    
    if (env <= TX_COMP_THR) return x;          // ниже порога — без компрессии    
    
    // gain = 1 - (1 - thr/env) * (1 - 1/ratio), всё в Q15    
    int32_t inv    = (int32_t)(((int64_t)TX_COMP_THR << 15) / env); // thr/env Q15    
    int32_t depth  = 32768 - (32768 * 10) / (int32_t)tx_ratio_x10;  // 1-1/r Q15    
    int32_t gain   = 32768 - (((32768 - inv) * depth) >> 15);       // Q15    
    return (int32_t)(((int64_t)x * gain) >> 15);    
}    
    
// ---------------------------------------------------------------------------    
// Клиппер + makeup    
// ---------------------------------------------------------------------------    
static inline int32_t __not_in_flash_func(tx_clip)(int32_t x) {    
    int32_t t = (int32_t)tx_clip_thr;    
    if (x >  t) x =  t;    
    if (x < -t) x = -t;    
    // makeup: вернуть уровень к полной шкале (32767/clip_thr)    
    return (int32_t)(((int64_t)x * 32767) / t);    
}    

// ---------------------------------------------------------------------------    
// Точка входа из af_dsp_poll    
// ---------------------------------------------------------------------------

// АРУ: gain = target/env, кламп 1..max. EMA огибающей — общая с  
// компрессором нет, своя (агрессивнее атака, медленнее спад).  
static int32_t tx_agc(int32_t x) {  
    uint32_t a = (uint32_t)(x >= 0 ? x : -x);    
    uint32_t c = (a > (agc_env >> 8)) ? AF_TX_AGC_ATT_Q24 : AF_TX_AGC_REL_Q24;    
    agc_env += (uint32_t)(((int64_t)((a << 8) - agc_env) * c) >> 24);    
  
    uint32_t env = agc_env >> 8;  
    if (env < 64) env = 64;                          // защита от деления на ~0  
    int64_t g = ((int64_t)tx_agc_target << 15) / (int64_t)env;   // Q15  
    int64_t gmax = (int64_t)AF_TX_AGC_MAX_X10 * 3277;            // x10 -> Q15  
    if (g > gmax) g = gmax;  
    if (g < 32768) g = 32768;                        // АРУ только усиливает/держит  
    return (int32_t)(((int64_t)x * g) >> 15);  
}  
  
// Шумовой гейт: env < thr -> gain -> 0 (плавно), иначе -> 1.  
// Плавные фронты убирают щелчки на границах открытия/закрытия.  
static int32_t tx_gate(int32_t x) {  
    if (tx_gate_thr == 0) return x;                  // гейт выключен  
    uint32_t env = agc_env >> 8;                     // та же огибающая  
    uint32_t c = (env > tx_gate_thr) ? AF_TX_GATE_ATT_Q24 : AF_TX_GATE_REL_Q24;  
    uint32_t tgt = (env > tx_gate_thr) ? 65536u : 0u;  
    gate_gain += (uint32_t)(((int64_t)((int32_t)tgt - (int32_t)gate_gain) * c) >> 24);    
    if (gate_gain > 65536u) gate_gain = 65536u;  
    return (int32_t)(((int64_t)x * (int32_t)gate_gain) >> 16);  
}  

int16_t __not_in_flash_func(af_tx_process)(int32_t y) {    
    y = tx_fir_push(&tx_pre, y);     // речевая полоса      
    y = tx_agc(y);                   // АРУ: выравнивание уровня к target  
    y = tx_gate(y);                  // шумовой гейт: паузы -> тишина  
    y = tx_compress(y);              // компрессия      
    y = tx_clip(y);                  // клиппер + makeup    
    y = tx_fir_push(&tx_post, y);    // срез гармоник клиппинга    
    if (y >  32767) y =  32767;    
    if (y < -32768) y = -32768;    
    return (int16_t)y;    
}    
    
// ---------------------------------------------------------------------------    
// Управление (Core 0)    
// ---------------------------------------------------------------------------    
void af_tx_init(void) {
    agc_env   = 0;  
    gate_gain = 0;  
    tx_fir_design();    
    memset(&tx_pre,  0, sizeof(tx_pre));    
    memset(&tx_post, 0, sizeof(tx_post));    
    tx_env_q16 = (int64_t)256 << 16;    
}    
    
void af_tx_set_band(uint32_t lo_hz, uint32_t hi_hz) {    
    const uint32_t FS2 = AF_FS_OUT_HZ / 2;    
    if (hi_hz > FS2 - 200) hi_hz = FS2 - 200;    
    if (lo_hz >= hi_hz)    lo_hz = (hi_hz > 100) ? (hi_hz - 100) : 0;    
    tx_lo_hz = lo_hz;    
    tx_hi_hz = hi_hz;    
    tx_fir_design();    
}    
uint32_t af_tx_lo_hz(void) { return tx_lo_hz; }    
uint32_t af_tx_hi_hz(void) { return tx_hi_hz; }    
    
void af_tx_set_ratio(uint32_t r_x10) {    
    if (r_x10 < 10)  r_x10 = 10;    // 1:1 — обход    
    if (r_x10 > 100) r_x10 = 100;   // 10:1 — почти лимитер    
    tx_ratio_x10 = r_x10;    
}    
uint32_t af_tx_ratio(void) { return tx_ratio_x10; }    
    
void af_tx_set_clip(uint32_t thr) {    
    if (thr < 4096)  thr = 4096;    
    if (thr > 32767) thr = 32767;    
    tx_clip_thr = thr;    
}    
uint32_t af_tx_clip(void) { return tx_clip_thr; }

void af_tx_set_agc(uint32_t target) {  
    if (target < 4096)  target = 4096;  
    if (target > 32767) target = 32767;  
    tx_agc_target = target;  
}  
uint32_t af_tx_agc(void) { return tx_agc_target; }  
  
void af_tx_set_gate(uint32_t thr) {  
    if (thr > 32767) thr = 32767;  
    tx_gate_thr = thr;               // 0 = гейт off  
}  
uint32_t af_tx_gate(void) { return tx_gate_thr; }  