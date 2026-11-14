// src/af_tx.cpp    
// ============================================================================    
// af_tx.cpp — передающий тракт SSB (Core 1)    
//    
// Цепочка на AF_FS_OUT_HZ (~15.464 ksps):    
//   y -> полосовой FIR -> all-pass (симметризатор) -> АРУ -> гейт    
//     -> 2-полосная компрессия (split ~900 Гц: НЧ/ВЧ раздельно)    
//     -> клиппер (кубический/жёсткий) -> постфильтр -> int16    
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
static volatile int32_t  tx_depth_q15 =                           // 1-1/r, Q15  
    32768 - (32768 * 10) / (int32_t)AF_TX_RATIO_X10_DEFAULT;  
static volatile uint32_t tx_clip_thr  = AF_TX_CLIP_DEFAULT;       // 0..32767
static volatile int32_t  tx_makeup_q15 =                          // 32767/t, Q15  
    (int32_t)(((int64_t)32767 << 15) / AF_TX_CLIP_DEFAULT);  
static uint32_t tx_agc_target = AF_TX_AGC_TARGET;  // целевой уровень АРУ  
static uint32_t tx_gate_thr   = AF_TX_GATE_THR;    // порог гейта (0 = off)    
static volatile int32_t tx_gate_floor = AF_TX_GATE_FLOOR_Q16; // нижняя ступень, Q16  
static uint32_t agc_env       = 0;                 // EMA огибающей АРУ, Q16
static int32_t  tx_agc_gain_q15 = 32768;           // последний gain АРУ (debug)  
static uint32_t gate_gain     = 0;                 // gain гейта, Q16 (0..65536)  
static int64_t tx_env_q16 = (int64_t)256 << 16;   // EMA огибающей, Q16
static volatile uint8_t  tx_ap_on    = AF_TX_AP_ON_DEFAULT;  
static volatile uint8_t  tx_softclip = AF_TX_SOFTCLIP_ON_DEFAULT;  
static int32_t ap_x1 = 0, ap_y1 = 0;   // состояние all-pass  
static volatile uint8_t  tx_2band_on = AF_TX_2BAND_ON_DEFAULT;
static volatile uint8_t  tx_notch_on = AF_TX_NOTCH_ON_DEFAULT;  
// Коэффициенты нотчей Q15: b = {1, -2cos(w), 1}, a = {1, -2r cos(w), r^2}  
static int32_t n1_b1 = 0, n1_a1 = 0, n1_a2 = 0;  
static int32_t n2_b1 = 0, n2_a1 = 0, n2_a2 = 0;  
static int32_t n1_x1 = 0, n1_x2 = 0, n1_y1 = 0, n1_y2 = 0;  
static int32_t n2_x1 = 0, n2_x2 = 0, n2_y1 = 0, n2_y2 = 0;  
static uint16_t tx_gate_hold = 0;   // счётчик hangover, в отсчётах @AF_FS_OUT_HZ

// --- Этап 2: Dynamic Presence EQ (SNR-адаптивный) ---  
static volatile uint8_t  tx_pres_on    = AF_TX_PRES_ON_DEFAULT;  
static volatile int32_t  tx_presmax_q15 = AF_TX_PRES_MAX_Q15;  // потолок подъёма, Q15  
  
// Отвод env_hi_pre: HP ~1.5 кГц (биквад) + EMA — после АРУ, ДО гейта.  
// Источник для k_dyn и noise floor (env_hi компрессора в паузах = 0).  
static int32_t hp_x1 = 0, hp_x2 = 0, hp_y1 = 0, hp_y2 = 0;   // HP-биквад  
static int32_t hp_b0 = 0, hp_b1 = 0, hp_b2 = 0, hp_a1 = 0, hp_a2 = 0;  
static int64_t env_hi_pre_q16 = (int64_t)256 << 16;          // EMA |hp|, Q16  
static int64_t pres_nf_q16    = (int64_t)256 << 16;          // noise floor, Q16  
static int32_t k_dyn_q15      = 0;                           // сглаженный k_dyn, Q15  
  
// Peaking-биквад 2–2.7 кГц (после компрессора, до клиппера)  
static int32_t pe_x1 = 0, pe_x2 = 0, pe_y1 = 0, pe_y2 = 0;  
static int32_t pe_b0 = 0, pe_b1 = 0, pe_b2 = 0, pe_a1 = 0, pe_a2 = 0;  

static int32_t split_lp = 0;                    // состояние ФНЧ кроссовера  
static int64_t tx_env_lo_q16 = (int64_t)256 << 16;  // EMA огибающей НЧ-полосы  
static int64_t tx_env_hi_q16 = (int64_t)256 << 16;  // EMA огибающей ВЧ-полосы  

// Порог компрессора — фиксированный, ~25% шкалы.    
#define TX_COMP_THR   8192    

// All-pass 1-го порядка (phase rotator): y = a*(x - y1) + x1.  
// Амплитуда не меняется, фаза крутится — пики речи симметрируются,  
// клипперу достаётся более «квадратный» сигнал.  
static inline int32_t tx_allpass(int32_t x) {  
    int32_t y = (int32_t)(((int64_t)AF_TX_AP_COEFF_Q15 * (x - ap_y1)) >> 15) + ap_x1;  
    ap_x1 = x;  
    ap_y1 = y;  
    return y;  
}  

// Компрессия одной полосы: своя EMA-огибающая, свой порог.  
// gain = 1 - (1 - thr/env) * (1 - 1/ratio), всё в Q15.  
static inline int32_t __not_in_flash_func(tx_comp_band)(int32_t x,  
                                                        int64_t* env_q16,  
                                                        int32_t thr) {  
    int64_t ax  = (x < 0) ? -x : x;  
    int64_t tgt = ax << 16;  
    int64_t da  = tgt - *env_q16;  
    *env_q16 += (da * (da > 0 ? (int64_t)AF_TX_ENV_ATT_Q24  
                              : (int64_t)AF_TX_ENV_DEC_Q24)) >> 24;  
    if (*env_q16 < ((int64_t)1 << 16)) *env_q16 = (int64_t)1 << 16;  
  
    int32_t env = (int32_t)(*env_q16 >> 16);  
    if (env <= thr) return x;                  // ниже порога — без компрессии  
  
    int32_t inv    = (int32_t)(((int64_t)thr << 15) / env);         // thr/env Q15    
    int32_t gain   = 32768 - (((32768 - inv) * tx_depth_q15) >> 15); // Q15    
    return (int32_t)(((int64_t)x * gain) >> 15);  
}  
  
// 2-полосная компрессия: комплементарный split на ~900 Гц.  
// lo = ФНЧ(x), hi = x - lo — сумма тождественно равна x (фазово согласованы).  
// НЧ (гласные) и ВЧ (согласные) сжимаются раздельно -> нет pumping.  
static inline int32_t __not_in_flash_func(tx_compress)(int32_t x) {  
    if (!tx_2band_on)  
        return tx_comp_band(x, &tx_env_q16, TX_COMP_THR);   // старый 1-полосный  
  
    // Кроссовер: ФНЧ 1-го порядка, k = AF_TX_SPLIT_K_Q15  
    split_lp += (int32_t)(((int64_t)(x - split_lp) * AF_TX_SPLIT_K_Q15) >> 15);  
    int32_t lo = split_lp;  
    int32_t hi = x - lo;  
  
    lo = tx_comp_band(lo, &tx_env_lo_q16, AF_TX_COMP_THR_LO);  
    hi = tx_comp_band(hi, &tx_env_hi_q16, AF_TX_COMP_THR_HI);  
    return lo + hi;  
}  
    
// ---------------------------------------------------------------------------    
// Клиппер + makeup    
// ---------------------------------------------------------------------------    
static inline int32_t __not_in_flash_func(tx_clip)(int32_t x) {      
    int32_t t = (int32_t)tx_clip_thr;      
    if (x >  t) x =  t;      
    if (x < -t) x = -t;      
    return (int32_t)(((int64_t)x * tx_makeup_q15) >> 15);   // makeup кеширован  
}      

// Кубический клиппер: y = x - x^3/(3*t^2), нормирован к t на выходе.  
// Мягче hard clip — меньше гармоник на входе пост-FIR.  
// При |x| >= t выход = ±(2/3)*t*1.5 = ±t — непрерывно.  
static inline int32_t __not_in_flash_func(tx_soft_clip)(int32_t x) {  
    int32_t t = (int32_t)tx_clip_thr;  
    if (x >  t) x =  t;  
    if (x < -t) x = -t;  
    // x^3/(3*t^2): x*x может переполнить int32 при x~32767 -> int64  
    int64_t x3 = ((int64_t)x * x * x) / ((int64_t)t * t);   // ~x^3/t^2    
    int32_t y = x - (int32_t)(x3 / 3);    
    // нормировка *3/2 + makeup к 32767 — кешировано в tx_makeup_q15    
    return (int32_t)((((int64_t)y * 3) >> 1) * tx_makeup_q15 >> 15);    
}  

// Биквад-нотч (Direct Form I), коэффициенты Q15.  
// b0=1, b1=-2cos(w), b2=1; a1=-2r*cos(w), a2=r^2.  
static inline int32_t __not_in_flash_func(tx_notch)(int32_t x,  
        int32_t b1, int32_t a1, int32_t a2,  
        int32_t* x1, int32_t* x2, int32_t* y1, int32_t* y2) {  
    int64_t acc = ((int64_t)x << 15) + (int64_t)b1 * *x1 + ((int64_t)*x2 << 15)  
                - (int64_t)a1 * *y1 - (int64_t)a2 * *y2;  
    int32_t y = (int32_t)(acc >> 15);  
    *x2 = *x1; *x1 = x;  
    *y2 = *y1; *y1 = y;  
    return y;  
}  

// HP-биквад (Direct Form I), коэффициенты Q15 — отвод для env_hi_pre.  
static inline int32_t __not_in_flash_func(tx_hp_push)(int32_t x) {  
    int64_t acc = (int64_t)hp_b0 * x + (int64_t)hp_b1 * hp_x1  
                + (int64_t)hp_b2 * hp_x2  
                - (int64_t)hp_a1 * hp_y1 - (int64_t)hp_a2 * hp_y2;  
    int32_t y = (int32_t)(acc >> 15);  
    hp_x2 = hp_x1; hp_x1 = x;  
    hp_y2 = hp_y1; hp_y1 = y;  
    return y;  
}  
  
// Обновление env_hi_pre, noise floor и k_dyn — вызывается после tx_agc,  
// ДО tx_gate. env_hi_pre живёт на незагейтованном сигнале: в паузах  
// виден реальный шум ВЧ-полосы, а не ноль.  
static inline void __not_in_flash_func(tx_pres_sidechain)(int32_t x_agc) {  
    int32_t hp = tx_hp_push(x_agc);  
    int64_t ax = (hp < 0) ? -(int64_t)hp : (int64_t)hp;  
  
    // EMA огибающей ВЧ (атака/спад — общие константы компрессора)  
    int64_t tgt = ax << 16;  
    int64_t da  = tgt - env_hi_pre_q16;  
    env_hi_pre_q16 += (da * (da > 0 ? (int64_t)AF_TX_ENV_ATT_Q24  
                                    : (int64_t)AF_TX_ENV_DEC_Q24)) >> 24;  
    if (env_hi_pre_q16 < ((int64_t)1 << 16)) env_hi_pre_q16 = (int64_t)1 << 16;  
  
    // Noise floor: минимум-трекер — вниз быстро, вверх медленно.  
    // Обновление только при закрытом гейте (agc_env < txgate):  
    // в речи floor заморожен, «залипания» нет — при появлении паузы  
    // подстраивается за ~0.5 с.  
    uint32_t env8 = agc_env >> 8;  
    if (tx_gate_thr != 0 && env8 < tx_gate_thr) {  
        if (env_hi_pre_q16 < pres_nf_q16)  
            pres_nf_q16 = env_hi_pre_q16;                    // вниз — мгновенно  
        else  
            pres_nf_q16 += (env_hi_pre_q16 - pres_nf_q16) >> AF_TX_PRES_NF_UP_SHIFT;  
    }  
    if (pres_nf_q16 < ((int64_t)1 << 16)) pres_nf_q16 = (int64_t)1 << 16;  
  
    // k_dyn = clamp((env_hi_pre - 2*nf) / (4*nf), 0..1), Q15.  
    // Одна формула: и опорный уровень, и SNR-гейтинг — шум пауз  
    // (env ~ nf) даёт k_dyn ~ 0, речь (env >> nf) — k_dyn -> 1.  
    int64_t nf  = pres_nf_q16 >> 16;                          // в шкалу 0..32767  
    int64_t env = env_hi_pre_q16 >> 16;  
    int64_t num = env - 2 * nf;  
    int32_t k = 0;  
    if (num > 0) {  
        int64_t den = 4 * nf;  
        k = (int32_t)((num << 15) / den);  
        if (k > 32768) k = 32768;  
    }  
  
    // Сглаживание k_dyn: атака ~10 мс, спад ~100 мс — без щелчков  
    // на границах слогов.  
    int32_t c = (k > k_dyn_q15) ? AF_TX_PRES_K_ATT_Q24 : AF_TX_PRES_K_REL_Q24;  
    k_dyn_q15 += (int32_t)(((int64_t)(k - k_dyn_q15) * c) >> 24);  
}  
  
// Peaking-биквад с переменным подъёмом: gain = 1 + k_dyn*(presmax-1).  
// Реализовано как dry + k_dyn*(wet - dry): при k_dyn=0 — прозрачный  
// байпас, при k_dyn=1 — полный подъём presmax. Коэффициенты биквада  
// посчитаны на максимальный подъём; промежуточные k — линейный подмес.  
static inline int32_t __not_in_flash_func(tx_presence)(int32_t x) {  
    if (!tx_pres_on || k_dyn_q15 <= 0) return x;  
    int64_t acc = (int64_t)pe_b0 * x + (int64_t)pe_b1 * pe_x1  
                + (int64_t)pe_b2 * pe_x2  
                - (int64_t)pe_a1 * pe_y1 - (int64_t)pe_a2 * pe_y2;  
    int32_t wet = (int32_t)(acc >> 15);  
    pe_x2 = pe_x1; pe_x1 = x;  
    pe_y2 = pe_y1; pe_y1 = wet;  
    // y = x + k_dyn*(wet - x): k_dyn Q15  
    return x + (int32_t)(((int64_t)(wet - x) * k_dyn_q15) >> 15);  
}  

// ---------------------------------------------------------------------------    
// Точка входа из af_dsp_poll    
// ---------------------------------------------------------------------------

// АРУ: gain = target/env, кламп 1..max. EMA огибающей — общая с  
// компрессором нет, своя (агрессивнее атака, медленнее спад).  
static int32_t tx_agc(int32_t x) {  
    uint32_t a = (uint32_t)(x >= 0 ? x : -x);    
    uint32_t c = (a > (agc_env >> 8)) ? AF_TX_AGC_ATT_Q24 : AF_TX_AGC_REL_Q24;    
    // ВАЖНО: вычитание в int64 — иначе при agc_env > a<<8 беззнаковый    
    // андерфлоу даёт +4 млрд и огибающая залипает на UINT32_MAX    
    // (гейт permanently открыт, АРУ застревает на gain=1).    
    agc_env += (uint32_t)(((int64_t)((int64_t)(a << 8) - (int64_t)agc_env) * c) >> 24);      
  
    uint32_t env = agc_env >> 8;  
    if (env < 64) env = 64;                          // защита от деления на ~0  
    int64_t g = ((int64_t)tx_agc_target << 15) / (int64_t)env;   // Q15    
    int64_t gmax = (int64_t)AF_TX_AGC_MAX_X10 * 3277;            // x10 -> Q15    
    if (g > gmax) g = gmax;    
    if (g < 32768) g = 32768;                        // АРУ только усиливает/держит    
    // Гейт закрыт (env ниже порога) -> не разгонять фон: gain <= 1.    
    // Иначе в паузе АРУ поднимает шум комнаты до target и гейт    
    // пропускает его на floor-уровне — «сигнал проходит всегда».    
    if (tx_gate_thr != 0 && env < tx_gate_thr && g > 32768) g = 32768;  
    tx_agc_gain_q15 = (int32_t)g;                  // для stats (этап 0)  
    return (int32_t)(((int64_t)x * g) >> 15);  
}  
  
// Шумовой гейт: env < thr -> gain -> AF_TX_GATE_FLOOR_Q16 (-24 дБ),  
// иначе -> 1.0. Плавные фронты убирают щелчки на границах; нижняя  
// ступень вместо нуля убирает «мёртвые паузы» и уменьшает скачок  
// gain при открытии (0.063->1.0 вместо 0->1.0) — выброс FIR слабее.  
// Шумовой гейт с гистерезисом и hangover (этап 1):  
//   env > thr        -> открыть мгновенно, перезарядить hold  
//   env < thr*0.7    -> отсчитывать hangover, потом закрыть на floor  
//   между порогами   -> держать текущее состояние (нет болтанки)  
// Открытие мгновенное — начала слов не съедаются; EMA gate_gain  
// даёт плавные фронты без щелчков.  
static int32_t tx_gate(int32_t x) {  
    if (tx_gate_thr == 0) return x;                  // гейт выключен  
    uint32_t env = agc_env >> 8;                     // та же огибающая  
    const uint32_t thr_close = (tx_gate_thr * AF_TX_GATE_HYST_X10) / 10;  
    const uint16_t hold_n = (uint16_t)((AF_TX_GATE_HOLD_MS * AF_FS_OUT_HZ) / 1000u);  
  
    if (env > tx_gate_thr) {  
        tx_gate_hold = hold_n;                       // открытие мгновенное  
    } else if (env < thr_close) {  
        if (tx_gate_hold > 0) --tx_gate_hold;        // hangover  
    }  
    // env в [thr_close, thr] — hold не трогаем: гистерезис  
  
    uint32_t tgt = (tx_gate_hold > 0) ? 65536u : (uint32_t)tx_gate_floor;  
    uint32_t c = (tx_gate_hold > 0) ? AF_TX_GATE_ATT_Q24 : AF_TX_GATE_REL_Q24;  
    gate_gain += (uint32_t)(((int64_t)((int32_t)tgt - (int32_t)gate_gain) * c) >> 24);  
    if (gate_gain > 65536u) gate_gain = 65536u;  
    return (int32_t)(((int64_t)x * (int32_t)gate_gain) >> 16);  
}  

int16_t __not_in_flash_func(af_tx_process)(int32_t y) {      
    y = tx_fir_push(&tx_pre, y);      // речевая полоса        
    if (tx_ap_on) y = tx_allpass(y);  // симметризатор пиков речи  
    y = tx_agc(y);                    // АРУ: выравнивание уровня к target      
    tx_pres_sidechain(y);             // env_hi_pre/nf/k_dyn — ДО гейта  
    y = tx_gate(y);                   // шумовой гейт: паузы -> тишина      
    y = tx_compress(y);               // компрессия          
    y = tx_presence(y);               // dyn-presence EQ (этап 2)  
    y = tx_softclip ? tx_soft_clip(y) // кубический клиппер (мягкий)    
                    : tx_clip(y);     // жёсткий клиппер    
    y = tx_fir_push(&tx_post, y);     // срез гармоник клиппинга  
    if (tx_notch_on) {                // узкие нотчи против тонов-наводок  
        y = tx_notch(y, n1_b1, n1_a1, n1_a2, &n1_x1, &n1_x2, &n1_y1, &n1_y2);  
        y = tx_notch(y, n2_b1, n2_a1, n2_a2, &n2_x1, &n2_x2, &n2_y1, &n2_y2);  
    }  
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
    tx_gate_hold = 0;
    tx_fir_design();    
    memset(&tx_pre,  0, sizeof(tx_pre));    
    memset(&tx_post, 0, sizeof(tx_post));    
    tx_env_q16    = (int64_t)256 << 16;  
    tx_env_lo_q16 = (int64_t)256 << 16;  
    tx_env_hi_q16 = (int64_t)256 << 16;  
    split_lp = 0;  
    ap_x1 = 0;  
    ap_y1 = 0;
    // Коэффициенты нотчей: w = 2*pi*f0/Fs, b1 = -2cos(w), a1 = -2r*cos(w), a2 = r^2  
    {  
        const float r  = (float)AF_TX_NOTCH_R_Q15 / 32768.0f;  
        const float w1 = 2.0f * PI * (float)AF_TX_NOTCH1_HZ / AF_FS_OUT_HZ;  
        const float w2 = 2.0f * PI * (float)AF_TX_NOTCH2_HZ / AF_FS_OUT_HZ;  
        n1_b1 = (int32_t)lroundf(-2.0f * cosf(w1) * 32768.0f);  
        n1_a1 = (int32_t)lroundf(-2.0f * r * cosf(w1) * 32768.0f);  
        n1_a2 = (int32_t)lroundf(r * r * 32768.0f);  
        n2_b1 = (int32_t)lroundf(-2.0f * cosf(w2) * 32768.0f);  
        n2_a1 = (int32_t)lroundf(-2.0f * r * cosf(w2) * 32768.0f);  
        n2_a2 = n1_a2;  
        n1_x1 = n1_x2 = n1_y1 = n1_y2 = 0;  
        n2_x1 = n2_x2 = n2_y1 = n2_y2 = 0;  
    }
    // Этап 2: HP-биквад ~1.5 кГц для env_hi_pre (RBJ high-pass, Q=0.707)  
    {  
        const float w  = 2.0f * PI * (float)AF_TX_PRES_HP_HZ / AF_FS_OUT_HZ;  
        const float cw = cosf(w), sw = sinf(w);  
        const float al = sw / (2.0f * 0.7071f);  
        const float a0 = 1.0f + al;  
        hp_b0 = (int32_t)lroundf(((1.0f + cw) * 0.5f / a0) * 32768.0f);  
        hp_b1 = (int32_t)lroundf((-(1.0f + cw) / a0) * 32768.0f);  
        hp_b2 = hp_b0;  
        hp_a1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);  
        hp_a2 = (int32_t)lroundf(((1.0f - al) / a0) * 32768.0f);  
        hp_x1 = hp_x2 = hp_y1 = hp_y2 = 0;  
    }  
    // Этап 2: peaking-биквад AF_TX_PRES_FREQ_HZ, Q, подъём presmax (RBJ)  
    {  
        const float A  = sqrtf((float)tx_presmax_q15 / 32768.0f); // лин. амплитуда  
        const float w  = 2.0f * PI * (float)AF_TX_PRES_FREQ_HZ / AF_FS_OUT_HZ;  
        const float cw = cosf(w), sw = sinf(w);  
        const float al = sw / (2.0f * AF_TX_PRES_Q_X100 / 100.0f);  
        const float a0 = 1.0f + al / A;  
        pe_b0 = (int32_t)lroundf(((1.0f + al * A) / a0) * 32768.0f);  
        pe_b1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);  
        pe_b2 = (int32_t)lroundf(((1.0f - al * A) / a0) * 32768.0f);  
        pe_a1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);  
        pe_a2 = (int32_t)lroundf(((1.0f - al / A) / a0) * 32768.0f);  
        pe_x1 = pe_x2 = pe_y1 = pe_y2 = 0;  
    }  
    env_hi_pre_q16 = (int64_t)256 << 16;  
    pres_nf_q16    = (int64_t)256 << 16;  
    k_dyn_q15      = 0;  
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
    tx_depth_q15 = 32768 - (32768 * 10) / (int32_t)r_x10;  // кеш, 1 деление  
}      
uint32_t af_tx_ratio(void) { return tx_ratio_x10; }    
    
void af_tx_set_clip(uint32_t thr) {      
    if (thr < 4096)  thr = 4096;      
    if (thr > 32767) thr = 32767;      
    tx_clip_thr = thr;      
    tx_makeup_q15 = (int32_t)(((int64_t)32767 << 15) / (int64_t)thr); // кеш  
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

void af_tx_set_ap(bool on)        { tx_ap_on = on ? 1 : 0; }  
bool af_tx_ap(void)               { return tx_ap_on != 0; }  
  
void af_tx_set_softclip(bool on)  { tx_softclip = on ? 1 : 0; }  
bool af_tx_softclip(void)         { return tx_softclip != 0; }  

void af_tx_set_2band(bool on)     { tx_2band_on = on ? 1 : 0; }  
bool af_tx_2band(void)            { return tx_2band_on != 0; }  

void af_tx_set_gfloor(uint32_t q16) {    
    if (q16 > 65536u) q16 = 65536u;    
    tx_gate_floor = (int32_t)q16;    // 0 = полное закрытие    
}    
uint32_t af_tx_gfloor(void) { return (uint32_t)tx_gate_floor; }    

void af_tx_set_notch(bool on)   { tx_notch_on = on ? 1 : 0; }  
bool af_tx_notch(void)          { return tx_notch_on != 0; }  

// --- debug getters (этап 0, аудит) ---  
// Все значения в шкале 0..32767 (env), gain — Q15.  
uint32_t af_tx_env_lo(void)   { return (uint32_t)(tx_env_lo_q16 >> 16); }  
uint32_t af_tx_env_hi(void)   { return (uint32_t)(tx_env_hi_q16 >> 16); }  
uint32_t af_tx_agc_env(void)  { return agc_env >> 8; }  
uint32_t af_tx_agc_gain(void) { return (uint32_t)tx_agc_gain_q15; }  

void af_tx_set_pres(bool on)  { tx_pres_on = on ? 1 : 0; }  
bool af_tx_pres(void)         { return tx_pres_on != 0; }  
  
void af_tx_set_presmax(uint32_t db_x10) {   // потолок подъёма, дБ x10  
    if (db_x10 > 120) db_x10 = 120;         // max +12 дБ  
    // Q15 = 10^(dB/20): float только в сеттере, не в горячем пути  
    tx_presmax_q15 = (int32_t)lroundf(powf(10.0f, (float)db_x10 / 200.0f) * 32768.0f);  
    // пересчёт коэффициентов peaking-биквада под новый потолок  
    const float A  = sqrtf((float)tx_presmax_q15 / 32768.0f);  
    const float w  = 2.0f * PI * (float)AF_TX_PRES_FREQ_HZ / AF_FS_OUT_HZ;  
    const float cw = cosf(w), sw = sinf(w);  
    const float al = sw / (2.0f * AF_TX_PRES_Q_X100 / 100.0f);  
    const float a0 = 1.0f + al / A;  
    pe_b0 = (int32_t)lroundf(((1.0f + al * A) / a0) * 32768.0f);  
    pe_b1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);  
    pe_b2 = (int32_t)lroundf(((1.0f - al * A) / a0) * 32768.0f);  
    pe_a1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);  
    pe_a2 = (int32_t)lroundf(((1.0f - al / A) / a0) * 32768.0f);  
}  
uint32_t af_tx_presmax(void) { return (uint32_t)tx_presmax_q15; }  
  
// --- debug getters (этап 2) ---  
uint32_t af_tx_env_hi_pre(void) { return (uint32_t)(env_hi_pre_q16 >> 16); }  
uint32_t af_tx_pres_nf(void)    { return (uint32_t)(pres_nf_q16 >> 16); }  
uint32_t af_tx_k_dyn(void)      { return (uint32_t)k_dyn_q15; }  