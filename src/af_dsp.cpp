// src/af_dsp.cpp  
// ============================================================================  
// af_dsp.cpp — моно DSP-тракт НЧ (Core 1)  
//  
// Тракт: uint16-блоки ADC (250 ksps) -> DC-EMA -> CIC /2 -> HB /2 -> HB /2  
// -> HB /2 (15.625 ksps) -> полосовой FIR 127 (симметричный) -> CW-биквады ->  
// автонотч NT (NLMS) -> шумоподавление NR (LMS ALE) -> АРУ+лимитер ->  
// две SPSC-очереди int16-блоков (PWM и USB).  
//
// TX-тракт (tx_on): тот же дециматор -> af_tx_process (FIR-компрессор-  
// клиппер-постфильтр) -> те же очереди; выход PWM переключается GP0->GP1.  
//  
// Порт src/dsp/rx_dsp.cpp и src/audio/rx_audio.cpp из RP2040_receiver.  
// Коэффициенты HB и константы NT/NR/AGC — 1:1 из приёмника.  
// Весь горячий путь — целочисленный Q15/int64, без float, в RAM.  
// ============================================================================  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "af_dsp.h"  
#include "af_config.h"  
#include "af_capture.h"
#include "af_store.h"
#include "af_voice.h"     // подмес голосовых клипов в выход
#include "af_tx.h"        // передающий тракт SSB    
#include "af_pwm.h"       // af_pwm_set_tx_pin — выход GP0(RX)/GP1(TX) 
  
// ---------------------------------------------------------------------------  
// Half-band дециматоры — коэффициенты 1:1 из rx_dsp.cpp приёмника  
// ---------------------------------------------------------------------------  
  
// Каскад 0: проверенный 15-отводный, ~80 дБ.  
// Сумма = 16384 + 2·(10030 − 2250 + 532 − 120) = 32768 → DC-gain = 1.0.  
#define HB0_NTAPS 15  
static const int16_t hb0_taps[HB0_NTAPS] = {  
    -120,     0,   532,     0, -2250,  
       0, 10030, 16384, 10030,     0,  
    -2250,     0,   532,     0,  -120  
};  
  
// Каскады 1 и 2: 31 отвод, окно Блэкмана-Харриса, ~90 дБ заграждения.  
#define HBX_NTAPS 31  
static const int16_t hbx_taps[HBX_NTAPS] = {  
       0,     0,     3,     0,   -25,     0,   119,     0,  
    -399,     0,  1086,     0, -2760,     0, 10168, 16384,  
   10168,     0, -2760,     0,  1086,     0,  -399,     0,  
     119,     0,   -25,     0,     3,     0,     0  
};  
  
typedef struct {  
    int32_t d[HBX_NTAPS];          // линия задержки под максимальную длину  
    uint8_t w;                     // позиция записи  
    uint8_t phase;                 // триггер чётности децимации  
    uint8_t ntaps;                 // длина этого каскада  
    const int16_t* taps;           // таблица коэффициентов  
} hb_state_t;  
  
static hb_state_t hb[AF_HB_STAGES];  
  
// ---------------------------------------------------------------------------  
// CIC R=2, N=3 — интеграторы на 250 ksps, гребёнки на 125 ksps.  
// Переполнение int32 в интеграторах — штатное (по модулю 2^32).  
// ---------------------------------------------------------------------------  
static int32_t cic_int[AF_CIC_N], cic_comb[AF_CIC_N];  
static uint8_t cic_cnt = 0;  
  
// Возвращает true раз в AF_CIC_R входных отсчётов.  
static inline bool __not_in_flash_func(cic_push)(int32_t x, int32_t* y) {  
    cic_int[0] += x; cic_int[1] += cic_int[0]; cic_int[2] += cic_int[1];  
  
    if (++cic_cnt < AF_CIC_R) return false;  
    cic_cnt = 0;  
  
    int32_t t = cic_int[2];  
    for (int k = 0; k < AF_CIC_N; ++k) {  
        int32_t d = t - cic_comb[k]; cic_comb[k] = t; t = d;  
    }  

    // компенсация усиления фильтра-дециматора CIC
    // DC-gain CIC = R^N = 2^3 = 8 → масштаб с округлением (+0.5 LSB)
    // *y = (t + 4) >> 3;
    
    *y = (t + 32) >> 6;   // DC-gain CIC = 4^3 = 64, +32 = округление +0.5 LSB
    
    return true;  
}  
  
// Один шаг half-band FIR + децимация /2 (порт hb_push из rx_dsp.cpp).  
static inline bool __not_in_flash_func(hb_push)(hb_state_t* st, int32_t x, int32_t* out) {  
    st->d[st->w] = x;  
    if (++st->w >= st->ntaps) st->w = 0;  
  
    st->phase ^= 1;  
    if (st->phase) return false;      // нечётный вход — выхода нет  
  
    // Симметрия + нулевые нечётные отводы:  
    // HB0 15 отводов → 5 умножений, HBX 31 отвод → 8 умножений.  
    const int N = (int)st->ntaps;  
    const int C = (N - 1) >> 1;  
    int32_t acc = 0;  
    for (int k = 0; k < C; ++k) {  
        const int32_t c = st->taps[k];  
        if (c == 0) continue;  
        int idx_lo = (int)st->w + k;  
        if (idx_lo >= N) idx_lo -= N;  
        int idx_hi = (int)st->w + (N - 1 - k);  
        if (idx_hi >= N) idx_hi -= N;  
        acc += c * (st->d[idx_lo] + st->d[idx_hi]);  
    }  
    // центральный отвод (16384)  
    int idx_c = (int)st->w + C;  
    if (idx_c >= N) idx_c -= N;  
    acc += 16384 * st->d[idx_c];  
  
    *out = (acc + (1 << 13)) >> 14;   // Q14 → масштаб отсчёта, +0.5 LSB  
    return true;  
}  
  
// ---------------------------------------------------------------------------  
// Полосовой FIR (AF_FIR_N отводов, sinc + окно Блэкмана-Харриса, Q15)  
// Порт af_lp_* из rx_dsp.cpp — пересчёт на лету  
// Работает на AF_FS_OUT_HZ = 15.625 ksps.  
// ---------------------------------------------------------------------------  
static volatile uint32_t af_lo_hz = AF_FIR_LO_HZ_DEFAULT;
static volatile uint32_t af_hi_hz = AF_FIR_HI_HZ_DEFAULT;
// Полоса, сохранённая при включении CW — восстанавливается по cw off.  
static uint32_t af_saved_lo_hz = AF_BAND_NARROW_LO_HZ;  
static uint32_t af_saved_hi_hz = AF_BAND_NARROW_HI_HZ;  
  
static int16_t af_fir_taps[AF_FIR_N];  
static int32_t af_fir_buf[AF_FIR_N] = {};  
static int     af_fir_idx = 0;  
  
static void af_fir_init(void) {  
    const int   M     = (AF_FIR_N - 1) / 2;  
    const float fc_hi = (float)af_hi_hz / AF_FS_OUT_HZ;  
    const float fc_lo = (float)af_lo_hz / AF_FS_OUT_HZ;  
    float h[AF_FIR_N];  
    float sum_dc = 0.0f;  
    for (int n = 0; n < AF_FIR_N; n++) {  
        const int m = n - M;  
        float s_hi = (m == 0) ? (2.0f * fc_hi)  
                              : sinf(2.0f * PI * fc_hi * m) / (PI * m);  
        float s_lo = (m == 0) ? (2.0f * fc_lo)  
                              : sinf(2.0f * PI * fc_lo * m) / (PI * m);  
        // Блэкман-Харрис (4 члена): подавление в полосе заграждения ~-92 дБ  
        // (у Блэкмана ~-58 дБ). Переходная полоса чуть шире: ~5.5*Fs/N.  
        const float a = 2.0f * PI * n / (AF_FIR_N - 1);  
        float w = 0.35875f - 0.48829f * cosf(a)  
                         + 0.14128f * cosf(2.0f * a)  
                         - 0.01168f * cosf(3.0f * a);  
        h[n] = (s_hi - s_lo) * w;  
        sum_dc += fabsf(h[n]);  
    }  
    const float norm = (sum_dc > 0.0f) ? (2.0f / sum_dc) : 1.0f;  
    for (int n = 0; n < AF_FIR_N; n++)  
        af_fir_taps[n] = (int16_t)lroundf(h[n] * norm * 32768.0f);  
}  
  
// Один отсчёт полосового FIR — симметричная свёртка (порт af_lp_push):  
// AF_FIR_N/2 умножений вместо AF_FIR_N, ~40% экономии.  
static inline int32_t __not_in_flash_func(af_fir_push)(int32_t x) {  
    af_fir_buf[af_fir_idx] = x;  
    if (++af_fir_idx >= AF_FIR_N) af_fir_idx = 0;  
  
    int64_t acc = 0;  
    int lo = af_fir_idx;  
    int hi = af_fir_idx + AF_FIR_N - 1;  
    if (hi >= AF_FIR_N) hi -= AF_FIR_N;  
    for (int k = 0; k < AF_FIR_N / 2; k++) {  
        acc += (int32_t)af_fir_taps[k] * (af_fir_buf[lo] + af_fir_buf[hi]);  
        if (++lo >= AF_FIR_N) lo = 0;  
        if (--hi < 0)        hi = AF_FIR_N - 1;  
    }  
    acc += (int32_t)af_fir_taps[AF_FIR_N / 2] * af_fir_buf[lo];  
    return (int32_t)((acc + (1 << 14)) >> 15);   // Q15 → масштаб, +0.5 LSB  
}  

// ---------------------------------------------------------------------------  
// Фиксированные узкие нотчи RX — гребёнка наводок 1/2/3/4 кГц.  
// Биквад-нотч (RBJ notch), Q15, Direct Form I. Частоты зашиты в  
// af_config.h — коэффициенты считаются один раз в af_dsp_init.  
// ---------------------------------------------------------------------------  
static volatile bool rx_notch_on = (AF_RX_NOTCH_ON_DEFAULT != 0);  
  
typedef struct {  
    int32_t b1, a1, a2;          // Q15; b0=b2=1 (нормированный нотч)  
    int32_t x1, x2, y1, y2;  
} rx_notch_t;  
  
static rx_notch_t rx_notch[4];  
  
static void rx_notch_design(void) {  
    const float r  = (float)AF_RX_NOTCH_R_Q15 / 32768.0f;  
    const float hz[4] = { (float)AF_RX_NOTCH1_HZ, (float)AF_RX_NOTCH2_HZ,  
                          (float)AF_RX_NOTCH3_HZ, (float)AF_RX_NOTCH4_HZ };  
    for (int i = 0; i < 4; i++) {  
        const float w = 2.0f * PI * hz[i] / (float)AF_FS_OUT_HZ;  
        rx_notch[i].b1 = (int32_t)lrintf(-2.0f * cosf(w) * 32768.0f);  
        rx_notch[i].a1 = (int32_t)lrintf(-2.0f * r * cosf(w) * 32768.0f);  
        rx_notch[i].a2 = (int32_t)lrintf(r * r * 32768.0f);  
        rx_notch[i].x1 = rx_notch[i].x2 = rx_notch[i].y1 = rx_notch[i].y2 = 0;  
    }  
    __dmb();  
}  
  
static inline int32_t __not_in_flash_func(rx_notch_push)(int32_t x) {  
    for (int i = 0; i < 4; i++) {  
        rx_notch_t* n = &rx_notch[i];  
        int64_t acc = ((int64_t)x << 15)  
                    + (int64_t)n->b1 * n->x1 + ((int64_t)n->x2 << 15)  
                    - (int64_t)n->a1 * n->y1 - (int64_t)n->a2 * n->y2;  
        int32_t y = (int32_t)(acc >> 15);  
        n->x2 = n->x1; n->x1 = x;  
        n->y2 = n->y1; n->y1 = y;  
        x = y;  
    }  
    return x;  
}  

// ---------------------------------------------------------------------------  
// Телеграфный фильтр: AF_CW_SECTIONS каскадных биквадов BPF (RBJ, Q15).  
// Коэффициенты пересчитываются на Core 0 при смене f0/BW.  
// Каскад из N одинаковых секций сужает полосу: BW_sec = BW / sqrt(2^(1/N)-1).  
// Работает на AF_FS_OUT_HZ = 15.625 ksps.  
// ---------------------------------------------------------------------------  
static volatile bool     cw_on = false;  
static volatile uint32_t cw_f0 = AF_CW_F0_DEFAULT;  
static volatile uint32_t cw_bw = AF_CW_BW_DEFAULT;  
  
static volatile uint32_t cw_noise = 400;   // уровень добавки шума (±)  
  
// --- Постоянная добавка шума на вход CW-фильтра ---------------------------  
// LFSR-шум малого уровня суммируется с сигналом: фильтр всегда возбуждён  
// (не звенит на тишине, АРУ не хлопает), слабые сигналы проходят без  
// порогов — сквелча нет. Уровень задаётся командой `cw noise` (0..2000).  
static uint32_t cw_lfsr = 0x1ACEu;   // состояние ГПСП (не ноль!)  
  
typedef struct {  
    int16_t b0, a1, a2;     // Q15/Q14; b1=0, b2=-b0 (RBJ BPF, peak gain = 1)  
    int32_t x1, x2, y1, y2; // состояние (Direct Form I), y — Q19  
} cw_biquad_t;  
  
static cw_biquad_t cw_sec[AF_CW_SECTIONS];  
  
// Проектирование одной секции: Q = f0 / BW_sec, alpha = sin(w0)/(2Q).  
// Вызывается из af_dsp_set_cw_* (Core 0, редкая операция, float допустим).  
static void cw_design(void) {  
    // Расширение полосы секции, чтобы каскад дал заданные -3 дБ.  
    const float shrink = sqrtf(powf(2.0f, 1.0f / AF_CW_SECTIONS) - 1.0f); // ~0.51 для N=3  
    const float bw_sec = (float)cw_bw / shrink;  
    const float w0     = 2.0f * PI * (float)cw_f0 / (float)AF_FS_OUT_HZ;  
    const float alpha  = sinf(w0) * (float)AF_FS_OUT_HZ / (2.0f * (float)cw_f0) * bw_sec / (float)AF_FS_OUT_HZ;  
    // alpha = sin(w0) / (2*Q), Q = f0/bw_sec:  
    const float a0 = 1.0f + alpha;  
    const float b0 =  alpha / a0;  
    const float a1 = -2.0f * cosf(w0) / a0;  
    const float a2 = (1.0f - alpha) / a0;  
  
    for (int s = 0; s < AF_CW_SECTIONS; s++) {  
        cw_sec[s].b0 = (int16_t)lrintf(b0 * 32768.0f);  
        cw_sec[s].a1 = (int16_t)lrintf(a1 * 16384.0f);   // a-коэфф. в Q14 (|a1| до 2)  
        cw_sec[s].a2 = (int16_t)lrintf(a2 * 16384.0f);  
        cw_sec[s].x1 = cw_sec[s].x2 = cw_sec[s].y1 = cw_sec[s].y2 = 0;  
    }  
    __dmb();  
}  


// Подтяжка FIR под центральную частоту CW: f0 ± 250 Гц.  
static void cw_band_follow(void) {    
    int32_t lo = (int32_t)cw_f0 - 250;    
    int32_t hi = (int32_t)cw_f0 + 250;    
    if (lo < 50)   lo = 50;    
    if (hi > 7000) hi = 7000;    
    af_lo_hz = (uint32_t)lo;    
    af_hi_hz = (uint32_t)hi;    
    af_fir_init();    
}

// Состояния y1/y2 — Q19 (int32 с 4 битами дробной части): на узкой полосе  
// (Q~15-17 при bw 80) квантование int16 давало предельные циклы.  
// Между секциями передаётся Q19 без потери точности.  
static inline int32_t __not_in_flash_func(cw_push)(int32_t x) {  
    x <<= 4;                                          // Q0 -> Q19: единый формат всех секций  
    for (int s = 0; s < AF_CW_SECTIONS; s++) {  
        cw_biquad_t* q = &cw_sec[s];  
        // y = b0*(x - x2) - a1*y1 - a2*y2, всё в Q19:  
        //   b0: Q15, (x-x2): Q19 -> произведение Q34, >>15 -> Q19  
        //   a1/a2: Q14, y1/y2: Q19 -> произведение Q33, >>14 -> Q19  
        int64_t acc = ((int64_t)q->b0 * (x - q->x2)) >> 15;  
        acc -= ((int64_t)q->a1 * q->y1) >> 14;  
        acc -= ((int64_t)q->a2 * q->y2) >> 14;  
        int32_t ys = (int32_t)acc;                             // уже Q19  
        if (ys >  (32767 << 4)) ys =  (32767 << 4);  
        if (ys < -(32768 << 4)) ys = -(32768 << 4);  
        q->x2 = q->x1; q->x1 = x;  
        q->y2 = q->y1; q->y1 = ys;  
        x = ys;  
    }  
    return x >> 4;                                    // Q19 -> Q0 (int16 на выход)  
}  
  
// ---------------------------------------------------------------------------  
// Автоматический notch NT (адаптивный LMS, выход = ошибка)  
// NOTCH_N=24, NOTCH_DELAY=96 — 1:1 из приёмника, работает на 15.625 ksps.  
// ---------------------------------------------------------------------------  
  
static volatile bool notch_on = false;    
static int16_t notch_w[NOTCH_N]                  = {};    
static int32_t notch_buf[NOTCH_N + NOTCH_DELAY]  = {};    
static int     notch_idx = 0;    
static int64_t notch_pow = 0;   // EMA мощности опорного вектора    
  
// Скользящее окно энергии: кольцо последних NOTCH_N опорных отсчётов    
// и бегущая сумма их квадратов — вместо O(N) пересчёта на каждый отсчёт.    
static int32_t  notch_v[NOTCH_N] = {};    
static int      notch_vidx = 0;    
static int64_t  notch_energy = 0;
  
// Один отсчёт автонотча. Возвращает ОШИБКУ e = x - y (тон вычтен).  
static inline int32_t __not_in_flash_func(notch_lms_push)(int32_t x) {  
    notch_buf[notch_idx] = x;  
  
    // предсказание y по задержанным отсчётам; энергия — скользящим окном    
    int64_t acc = 0;    
    int p = notch_idx - NOTCH_DELAY;    
    if (p < 0) p += NOTCH_N + NOTCH_DELAY;    
    int32_t taps[NOTCH_N];    
    for (int k = 0; k < NOTCH_N; k++) {    
        int32_t v = notch_buf[p];    
        taps[k] = v;    
        acc += (int64_t)notch_w[k] * v;           // y в Q15    
        if (--p < 0) p += NOTCH_N + NOTCH_DELAY;    
    }    
    int32_t y = (int32_t)(acc >> 15);    
    
    // Скользящая энергия: вычесть ушедший отсчёт, добавить новый.    
    // Опорный вектор — отсчёты с задержкой NOTCH_DELAY, т.е. v = taps[0]    
    // (самый свежий из окна).    
    {    
        int32_t v_new = taps[0];    
        int32_t v_old = notch_v[notch_vidx];    
        notch_v[notch_vidx] = v_new;    
        notch_vidx = (notch_vidx + 1) % NOTCH_N;    
        notch_energy += (int64_t)v_new * v_new - (int64_t)v_old * v_old;    
        if (notch_energy < 0) notch_energy = 0;   // защита от дрейфа    
    }    
    int64_t energy = notch_energy;
  
    int32_t e = x - y;                            // ошибка = выход нотча  
  
    // EMA мощности (сглаживание знаменателя NLMS)  
    notch_pow += (energy - notch_pow) >> 5;  
    int64_t den = notch_pow + NOTCH_EPS;  
  
    // NLMS не требует точного деления: заменяем /den на сдвиг  
    // по старшему биту den (шаг mu гуляет в пределах x2 — допустимо).  
    // int64-деление на M0+ — это ~100 циклов на КАЖДЫЙ отсчёт.  
    int k = 63 - __builtin_clzll((uint64_t)den);   // den > 0 всегда  
    int64_t g = (((int64_t)e * NOTCH_MU_Q) << 15) >> k;  
    if (g >  NOTCH_G_MAX) g =  NOTCH_G_MAX;  
    if (g < -NOTCH_G_MAX) g = -NOTCH_G_MAX; 
  
    for (int k = 0; k < NOTCH_N; k++) {  
        int32_t upd = (int32_t)(((int64_t)g * taps[k]) >> 15);  
        int32_t w   = (int32_t)notch_w[k] + upd - ((int32_t)notch_w[k] >> NOTCH_LEAK_SHIFT);  
        if (w >  32767) w =  32767;  
        if (w < -32768) w = -32768;  
        notch_w[k] = (int16_t)w;  
    }  
  
    if (++notch_idx >= NOTCH_N + NOTCH_DELAY) notch_idx = 0;  
    return e;  
}  
  
// ---------------------------------------------------------------------------  
// Шумоподавление NR — LMS Adaptive Line Enhancer (ALE)  
// ---------------------------------------------------------------------------  
  
static volatile bool nr_on = false;  
static int32_t nr_buf[NR_N + NR_DELAY] = {0};  
static int     nr_idx = 0;  
static int16_t nr_w[NR_N]  = {0};  
static int64_t nr_pow = 0;  
  
static inline int32_t __not_in_flash_func(nr_lms_push)(int32_t x) {  
    nr_idx = (nr_idx + 1); if (nr_idx >= NR_N + NR_DELAY) nr_idx = 0;  
    nr_buf[nr_idx] = x;  
  
    // предсказание + энергия окна одним проходом  
    int64_t acc = 0, energy = 0;  
    int p = nr_idx - NR_DELAY; if (p < 0) p += NR_N + NR_DELAY;  
    int taps[NR_N];  
    for (int k = 0; k < NR_N; k++) {  
        taps[k] = p;  
        int32_t xk = nr_buf[p];  
        acc    += (int64_t)nr_w[k] * xk;  
        energy += (int64_t)xk * xk;  
        if (--p < 0) p += NR_N + NR_DELAY;  
    }  
    int32_t y = (int32_t)(acc >> 15);  
    int32_t e = x - y;                         // ошибка = шум  
  
    nr_pow += (energy - nr_pow) >> NR_PWR_SHIFT;  
  
    int64_t den = nr_pow + NR_EPS;  
    int k = 63 - __builtin_clzll((uint64_t)den);  
    int64_t g = (((int64_t)e * NR_MU_Q) << 15) >> k;
    if (g >  NR_G_MAX) g =  NR_G_MAX;  
    if (g < -NR_G_MAX) g = -NR_G_MAX;  
  
    for (int k = 0; k < NR_N; k++) {  
        int64_t upd = (g * nr_buf[taps[k]]) >> 15;  
        int32_t w   = (int32_t)nr_w[k];  
        w += (int32_t)upd - (w >> NR_LEAK_SHIFT);  
        if (w >  32767) w =  32767;  
        if (w < -32768) w = -32768;  
        nr_w[k] = (int16_t)w;  
    }  
  
    // dry/wet с фиксированной глубиной: out = x − NR_DEPTH_Q15·e  
    int32_t out = x - (int32_t)(((int64_t)e * NR_DEPTH_Q15) >> 15);  
    // makeup-gain — компенсация просадки уровня  
    out = (int32_t)(((int64_t)out * NR_MAKEUP_Q15) >> 15);  
    if (out >  32767) out =  32767;  
    if (out < -32768) out = -32768;  
    return out;  
}  
  
// ---------------------------------------------------------------------------  
// АРУ + лимитер — два контура, порт agc_limit из rx_audio.cpp приёмника.  
//   A) медленный левелинг по среднему |y| (τ≈10 с, симметрично);  
//   B) быстрый лимитер поверх (мгновенная атака, спад τ≈0.2 с).  
// Состояние в Q16, коэффициенты EMA в Q24.  
// На 15.625 ksps все постоянные времени вдвое длиннее в секундах.  
// ---------------------------------------------------------------------------  
static int64_t agc_avg_q16  = (int64_t)4096 << 16;   // среднее уровня, Q16  
static int64_t lim_peak_q16 = (int64_t)2048 << 16;   // пик лимитера, Q16

static volatile bool     agc_on    = true;    // АРУ on/off (Core 0 пишет)  
static volatile uint32_t gain_q15  = 32768;   // ручное усиление, Q15 (32768 = ×1.0)
static volatile bool     bypass_on = false;   // обход всего тракта

static volatile bool tx_on = false;   // false=RX, true=TX (PTT GP8 / "tx on")  
  
static inline int16_t __not_in_flash_func(agc_limit)(int32_t x) {  
    // --- Контур A: медленное среднее |x| ---  
    // В CW-режиме — в 8 раз медленнее и без контура B (лимитера):  
    // gain выставляется и держится, посылки не «дышат».  
    const uint32_t att_q24 = cw_on ? CW_AGC_AVG_ATT_Q24 : AGC_AVG_ATT_Q24;  
    const uint32_t dec_q24 = cw_on ? CW_AGC_AVG_DEC_Q24 : AGC_AVG_DEC_Q24;  
  
    int64_t ax  = (x < 0) ? -x : x;  
    int64_t tgt = ax << 16;  
    int64_t da  = tgt - agc_avg_q16;  
    agc_avg_q16 += (da * (da > 0 ? (int64_t)att_q24  
                                 : (int64_t)dec_q24)) >> 24;  
    if (agc_avg_q16 < AGC_AVG_MIN) agc_avg_q16 = AGC_AVG_MIN;  
  
    int32_t y1;  
    if (agc_on) {  
        int32_t avg = (int32_t)(agc_avg_q16 >> 16);  
        // Кламп усиления: gain = AGC_REF/avg должен остаться в  
        // [MIN_GAIN .. MAX_GAIN] => avg в [REF/MAX .. REF/MIN].  
        // В тишине avg упирается в REF/MAX_GAIN — шум не разгоняется  
        // под потолок, голосовые сообщения слышны без agc off.  
        const float max_g = cw_on ? CW_AGC_MAX_GAIN : AGC_MAX_GAIN;  
        const float min_g = cw_on ? CW_AGC_MIN_GAIN : AGC_MIN_GAIN;  
        int32_t avg_lo = (int32_t)(AGC_REF / max_g);  
        int32_t avg_hi = (int32_t)(AGC_REF / min_g);  
        if (avg < avg_lo) avg = avg_lo;  
        if (avg > avg_hi) avg = avg_hi;  
        // Точное деление вместо >>k: сдвиг по старшему биту давал  
        // gain с ошибкой до 2x и СКАЧКИ gain при пересечении avg  
        // границ степени двойки -> AM-модуляция -> гребёнка палок.  
        // int64-деление ~100 циклов x 15.5 ksps = ~0.6% Core 1.  
        y1 = (int32_t)(((int64_t)x * AGC_REF) / avg);  
    } else {  
        // АРУ off: фиксированное ручное усиление, множитель в Q15.  
        y1 = (int32_t)(((int64_t)x * (int64_t)gain_q15) >> 15);
    }  
  
    if (cw_on) {                       // CW: без лимитера, только насыщение  
        if (y1 >  32767) y1 =  32767;  
        if (y1 < -32768) y1 = -32768;  
        return (int16_t)y1;  
    }  
  
    // Контур B (быстрый лимитер) работает всегда — и при agc off.
    int64_t ay  = (y1 < 0) ? -y1 : y1;  
    int64_t tp  = ay << 16;  
    int64_t dp  = tp - lim_peak_q16;  
    lim_peak_q16 += (dp * (dp > 0 ? LIMIT_ATT_Q24  
                                  : LIMIT_DEC_Q24)) >> 24;  
    if (lim_peak_q16 < ((int64_t)1 << 16)) lim_peak_q16 = (int64_t)1 << 16;  
  
    int32_t lpk = (int32_t)(lim_peak_q16 >> 16);  
    if (lpk < 1) lpk = 1;  
    if (lpk > LIMIT_CEIL) {  
        y1 = (int32_t)(((int64_t)y1 * LIMIT_CEIL) / lpk);  
    }  
  
    if (y1 >  32767) y1 =  32767;  
    if (y1 < -32768) y1 = -32768;  
    return (int16_t)y1;  
}  
  
// ---------------------------------------------------------------------------  
// Насыщение int32 -> int16 (определение ДО af_dsp_poll)  
// ---------------------------------------------------------------------------  
static volatile uint32_t clip_count = 0;  
  
static inline int16_t __not_in_flash_func(sat16)(int32_t x) {  
    if (x >  32767) { ++clip_count; return  32767; }  
    if (x < -32768) { ++clip_count; return -32768; }  
    return (int16_t)x;  
}  
  
// ---------------------------------------------------------------------------  
// Выходные SPSC-очереди int16-блоков: пишет af_dsp_poll (Core 1),  
// читают af_pwm_poll и af_usb_poll (Core 0) — у каждого СВОЯ очередь  
// (fan-out: каждый блок публикуется в обе).  
// ---------------------------------------------------------------------------  
typedef struct {      
    volatile int16_t  buf[AF_OUT_NUM_BUFFERS][AF_OUT_BLOCK_SAMPLES];      
    volatile uint8_t  busy[AF_OUT_NUM_BUFFERS];   // 1 = буфер заполнен/читается      
    volatile uint8_t  order[AF_OUT_NUM_BUFFERS];  // order[слот] = индекс буфера      
    volatile uint8_t  head;                       // слот публикации      
    volatile uint8_t  tail;                       // слот чтения      
    volatile uint8_t  fill_buf;                   // буфер, который сейчас заполняется      
    volatile uint32_t dropped;      
    volatile uint32_t underruns;                  // читатель спросил при пустой очереди      
    volatile uint32_t published;                  // блоков опубликовано      
    volatile uint8_t  fill_max;                   // high-water занятых буферов      
} out_queue_t;      
  
static out_queue_t out_pwm;    
static out_queue_t out_usb;    
static uint32_t    out_fill_pos = 0;   // позиция внутри заполняемого блока    
  
// Диагностический режим: вместо DSP выдаём пилу-счётчик в обе очереди.    
// Записать поток на хосте (Audacity): скачок/повтор пилы = дефект    
// доставки на устройстве; идеальная пила = глитчи рисует хост.    
// Включается #define AF_RAMP_TEST 1 (перепрошивка) или командой ramp.    
#ifndef AF_RAMP_TEST    
#define AF_RAMP_TEST 0    
#endif    
static volatile bool ramp_on = (AF_RAMP_TEST != 0);    
static int16_t       ramp_val = 0;    
  
// Публикация отсчёта в обе очереди.  
static inline void __not_in_flash_func(out_emit)(int16_t s) {  
    // PWM-очередь  
    out_queue_t* q = &out_pwm;  
    q->buf[q->fill_buf][out_fill_pos] = s;  
    // USB-очередь  
    q = &out_usb;  
    q->buf[q->fill_buf][out_fill_pos] = s;  
  
    if (++out_fill_pos < AF_OUT_BLOCK_SAMPLES) return;  
    out_fill_pos = 0;  
  
    // Блок заполнен — публикуем в каждой очереди независимо.    
    for (int i = 0; i < 2; i++) {    
        out_queue_t* oq = (i == 0) ? &out_pwm : &out_usb;    
        // Ищем следующий свободный буфер ДО публикации:    
        // если свободных нет — блок НЕ публикуем (честный drop),    
        // иначе буфер публиковался бы дважды → сшивка у потребителя.    
        uint8_t nb = oq->fill_buf;    
        bool found = false;    
        for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++) {    
            uint8_t cand = (oq->head + b) % AF_OUT_NUM_BUFFERS;    
            if (!oq->busy[cand] && cand != oq->fill_buf) { nb = cand; found = true; break; }    
        }    
        if (!found) {    
            ++oq->dropped;                 // очередь полна — блок теряется    
            continue;                      // fill_buf остаётся, перезапишем его    
        }    
        oq->busy[oq->fill_buf] = 1;        
        oq->order[oq->head] = oq->fill_buf;   // явный порядок: слот -> буфер    
        __dmb();        
        oq->head = (oq->head + 1) % AF_OUT_NUM_BUFFERS;        
        oq->fill_buf = nb;        
        ++oq->published;      
        // High-water: сколько буферов занято после публикации.      
        uint8_t n = 0;      
        for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++)      
            if (oq->busy[b]) ++n;      
        if (n > oq->fill_max) oq->fill_max = n;      
    }  
}  
  
// Читательская сторона (Core 0).  
static const volatile int16_t* out_get(out_queue_t* q) {      
    if (q->tail == q->head) { ++q->underruns; return NULL; }      
    __dmb();    
    return q->buf[q->order[q->tail]];   // буфер по порядку публикации    
}      
  
static void out_release(out_queue_t* q) {    
    if (q->tail == q->head) return;    
    const uint8_t b = q->order[q->tail];   // освобождаем тот буфер, что читали    
    q->tail = (q->tail + 1) % AF_OUT_NUM_BUFFERS;    
    __dmb();    
    q->busy[b] = 0;    
}    
  
const volatile int16_t* af_dsp_out_get_pwm(void)     { return out_get(&out_pwm); }  
void                    af_dsp_out_release_pwm(void) { out_release(&out_pwm); }  
const volatile int16_t* af_dsp_out_get_usb(void)     { return out_get(&out_usb); }  
void                    af_dsp_out_release_usb(void) { out_release(&out_usb); }  
  
uint8_t af_dsp_out_fill_usb(void) {  
    uint8_t n = 0;  
    for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++)  
        if (out_usb.busy[b]) ++n;  
    return n;  
}  
  
// ---------------------------------------------------------------------------  
// Управление (Core 0, редкие операции)  
// ---------------------------------------------------------------------------  
// Возвращает false, если команда отклонена (CW on — полоса следует за f0).  
bool af_dsp_set_band(uint32_t f_lo_hz, uint32_t f_hi_hz) {  
    if (cw_on) return false;  
    const uint32_t FS2 = AF_FS_OUT_HZ / 2;  
    if (f_hi_hz > FS2 - 200) f_hi_hz = FS2 - 200;  
    if (f_lo_hz >= f_hi_hz)  f_lo_hz = (f_hi_hz > 100) ? (f_hi_hz - 100) : 0;  
    af_lo_hz = f_lo_hz;  
    af_hi_hz = f_hi_hz;  
    af_fir_init();  
    return true;  
}
  
uint32_t af_dsp_af_lo_hz(void) { return af_lo_hz; }  
uint32_t af_dsp_af_hi_hz(void) { return af_hi_hz; }  
  
// Возврат: 0 = выключено, 1 = включено, 2 = включено и NR снят (приоритет NT).  
int af_dsp_set_notch(bool on) {  
    int rc = on ? 1 : 0;  
    if (on && nr_on) {              // NT имеет приоритет — снимаем NR  
        af_dsp_set_nr(false);  
        rc = 2;  
    }  
    if (on && !notch_on) {          // сброс состояния при включении — без щелчка  
        memset(notch_buf, 0, sizeof(notch_buf));  
        memset(notch_w,   0, sizeof(notch_w));  
        memset(notch_v,   0, sizeof(notch_v));  
        notch_idx = 0;  
        notch_vidx = 0;  
        notch_energy = 0;  
        notch_pow = 0;  
    }  
    notch_on = on;  
    return rc;  
}

void af_dsp_set_rxnotch(bool on) {  
    if (on && !rx_notch_on) {          // чистый старт — без переходного звона  
        for (int i = 0; i < 4; i++)  
            rx_notch[i].x1 = rx_notch[i].x2 = rx_notch[i].y1 = rx_notch[i].y2 = 0;  
    }  
    rx_notch_on = on;  
}  
bool af_dsp_rxnotch_on(void) { return rx_notch_on; }  
  
// Возвращает false, если команда отклонена (NT on — приоритет NT).  
bool af_dsp_set_nr(bool on) {    
    if (on && notch_on) return false;   // NT и NR взаимоисключающие  
    if (on && !nr_on) {              // чистый старт → без щелчка    
        memset(nr_buf, 0, sizeof(nr_buf));    
        memset(nr_w,   0, sizeof(nr_w));    
        nr_idx = 0;    
        nr_pow = 0;    
    }    
    nr_on = on;    
    return true;  
}
  
void af_dsp_set_cw(bool on) {  
    if (on && !cw_on) {    
        cw_design();
        agc_avg_q16 = (int64_t)4096 << 16;  
        // Сохранить текущую полосу и подтянуть FIR под f0 ± 250 Гц.  
        af_saved_lo_hz = af_lo_hz;  
        af_saved_hi_hz = af_hi_hz;  
        cw_band_follow();  
    }  
    if (on) {  
        for (int s = 0; s < AF_CW_SECTIONS; s++)  
            cw_sec[s].x1 = cw_sec[s].x2 = cw_sec[s].y1 = cw_sec[s].y2 = 0;  
    }  
    if (!on && cw_on) {  
        // Восстановить полосу, которая была до включения CW.  
        af_lo_hz = af_saved_lo_hz;  
        af_hi_hz = af_saved_hi_hz;  
        af_fir_init();  
    }  
    cw_on = on;  
}
  
void af_dsp_set_cw_freq(uint32_t hz) {  
    if (hz < AF_CW_F0_MIN) hz = AF_CW_F0_MIN;  
    if (hz > AF_CW_F0_MAX) hz = AF_CW_F0_MAX;  
    cw_f0 = hz;  
    cw_design();  
    if (cw_on) cw_band_follow();  
}  
  
void af_dsp_set_cw_bw(uint32_t hz) {  
    if (hz < AF_CW_BW_MIN) hz = AF_CW_BW_MIN;  
    if (hz > AF_CW_BW_MAX) hz = AF_CW_BW_MAX;  
    cw_bw = hz;  
    cw_design();  
    if (cw_on) cw_band_follow();  
}
  
void af_dsp_set_cw_noise(uint32_t lvl) {  
    if (lvl > 2000) lvl = 2000;  
    cw_noise = lvl;  
}

void af_dsp_set_agc(bool on) {  
    agc_on = on;  
    if (on) agc_avg_q16 = (int64_t)4096 << 16;   // чистый старт контура A  
}  
  
void af_dsp_set_gain(float m) {  
    if (m < AF_GAIN_MIN) m = AF_GAIN_MIN;  
    if (m > AF_GAIN_MAX) m = AF_GAIN_MAX;  
    gain_q15 = (uint32_t)(m * 32768.0f + 0.5f);  
}  
float af_dsp_gain(void) { return (float)gain_q15 / 32768.0f; }

void af_dsp_set_bypass(bool on) { bypass_on = on; }
void af_dsp_set_ramp(bool on) { ramp_on = on; ramp_val = 0; }  
bool af_dsp_ramp_on(void)     { return ramp_on; }  
bool     af_dsp_agc_on(void)     { return agc_on; }  
bool     af_dsp_bypass_on(void)  { return bypass_on; }
bool     af_dsp_cw_on(void)    { return cw_on; }  
uint32_t af_dsp_cw_freq(void)  { return cw_f0; }  
uint32_t af_dsp_cw_bw(void)    { return cw_bw; }  
uint32_t af_dsp_cw_noise(void) { return cw_noise; }  
bool af_dsp_notch_on(void) { return notch_on; }  
bool af_dsp_nr_on(void)    { return nr_on; }

void af_dsp_set_tx(bool on) {      
    if (tx_on == on) return;      
    tx_on = on;      
    // ЭТАП 2: переключение входа ADC — RX: GP26/ADC0, TX: GP27/ADC1.      
    // Пауза ~10 мкс + дренаж FIFO внутри af_capture_set_channel.      
    af_capture_set_channel(on ? AF_TX_ADC_CH : AF_RX_ADC_CH);      
    // ЭТАП 4: переключение выхода PWM — RX: GP0 (chan A), TX: GP1 (chan B).      
    // Оба пина на одном слайсе 0: меняется только адрес записи DMA      
    // (cc low half = A, cc high half = B) и SIO/PWM-функция пинов.      
    af_pwm_set_tx_pin(on);      
    // Сброс DC-EMA не нужен: dc_est — static в af_dsp_poll,      
    // переоценится за ~256 отсчётов (~0.5 мс) — один щелчок, неслышно.      
}  
  
bool af_dsp_tx_on(void) { return tx_on; }  
  
// ---------------------------------------------------------------------------  
// Инициализация (Core 0, до af_capture_start)  
// ---------------------------------------------------------------------------  
static uint32_t core1_busy_us = 0;  
static uint32_t core1_max_us  = 0;          // пиковое время обработки блока  
static uint32_t core1_window_start_us = 0;  
static volatile uint32_t dsp_load_permille = 0;  
  
void af_dsp_init(void) {  
    // HB-каскады: первый 15 отводов, второй и третий — 31 отвод.  
    memset(hb, 0, sizeof(hb));  
    hb[0].ntaps = HB0_NTAPS; hb[0].taps = hb0_taps;  
    hb[1].ntaps = HBX_NTAPS; hb[1].taps = hbx_taps;  
    hb[2].ntaps = HBX_NTAPS; hb[2].taps = hbx_taps;  
  
    memset(cic_int, 0, sizeof(cic_int));  
    memset(cic_comb, 0, sizeof(cic_comb));  
    cic_cnt = 0;  
  
    af_fir_init();  
    cw_design();          // начальные коэффициенты CW-секций  
    rx_notch_design();    // коэффициенты фиксированных нотчей RX  
    af_tx_init();         // TX: FIR-компрессор-клиппер    
  
    memset(&out_pwm, 0, sizeof(out_pwm));    
    memset(&out_usb, 0, sizeof(out_usb));    
    for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++) {    
        out_pwm.order[b] = b;   // тождественное отображение на старте    
        out_usb.order[b] = b;    
    }    
    out_fill_pos = 0;    
    clip_count = 0;  
    core1_busy_us = 0;  
    core1_max_us = 0;  
    dsp_load_permille = 0;  
    core1_window_start_us = time_us_32();  
}  
  
// ---------------------------------------------------------------------------  
// Измерение загрузки Core 1 (порт core1_update_load из rx_dsp.cpp)  
// ---------------------------------------------------------------------------  
static inline void core1_update_load(uint32_t now_us) {  
    const uint32_t window_us = now_us - core1_window_start_us;  
    if (window_us < 1000000u) return;  
  
    dsp_load_permille = (uint32_t)(((uint64_t)core1_busy_us * 1000u) / window_us);  
    core1_busy_us = 0;  
    core1_window_start_us = now_us;  
}  
  
// ---------------------------------------------------------------------------  
// Главный поллинг (Core 1): блоки ADC -> весь тракт -> выходные очереди  
// ---------------------------------------------------------------------------  
void __not_in_flash_func(af_dsp_poll)(void) {      
    // DC-смещение ADC: EMA k=1/256 (порт dc_i/dc_q из rx_dsp.cpp).      
    static int32_t dc_est = 0;      
      
    // Бюджет дрейна: не больше 2 блоков за вызов (~5 мс). Иначе при    
    // накопленном бэклоге Core 1 уходит в длинный цикл, out_usb/out_pwm    
    // не обслуживаются читателями вовремя — вставки/провалы на выходе.    
    uint8_t budget = 2;  
      
    af_store_core1_park_poll();   // если Core 0 просит парковку — встаём в RAM-spin      
      
    const volatile uint16_t* blk;      
    while (budget-- > 0 && (blk = af_capture_get_block()) != NULL) {  
        const uint32_t t0 = time_us_32();   // busy = только обработка блока    
  
        for (uint32_t i = 0; i < AF_BLOCK_SAMPLES; i++) {  
            // Сырой отсчёт ADC: 12 бит в младших разрядах.  
            int32_t meas = (int32_t)blk[i] & 0x0FFF;  
  
            // Центрирование: EMA среднего, вычитаем из измерения.  
            dc_est += ((meas << 8) - dc_est) >> 8;  
            int32_t x = (meas - (dc_est >> 8)) << 4;   // 12 -> 16 бит  
  
            // Децимация 250k -> 15.625k: CIC/2 -> HB/2 -> HB/2 -> HB/2
            int32_t y;  
            if (!cic_push(x, &y))        continue;  
            if (!hb_push(&hb[0], y, &y)) continue;  
            if (!hb_push(&hb[1], y, &y)) continue;  
            if (!hb_push(&hb[2], y, &y)) continue;  
            y >>= 3;    // компенсация DC-gain трёх HB (2^3 = 8)
  
            // НЧ-тракт на 15.625 ksps.    
            if (bypass_on) {      
                // Обход: децимированный вход напрямую на выход,      
                // без FIR/CW/NT/NR/АРУ/лимитера.  
                // голос должен быть слышен и в обходе  
                out_emit(af_voice_mix(sat16(y)));  
                //out_emit(sat16(y));      
                continue;      
            }      
            if (tx_on) {    
                // TX-тракт: FIR -> компрессор -> клиппер -> постфильтр.    
                // RX-цепочка (FIR/CW/NT/NR/AGC) в TX не работает.    
                out_emit(af_tx_process(y));    
                continue;    
            }    
            y = af_fir_push(y);    
            if (cw_on) {    
                // Добавка шума: LFSR xorshift32, уровень из `cw noise`.  
                // Фильтр всегда возбуждён — не звенит на тишине;  
                // сигнал проходит без порогов (сквелча нет).  
                cw_lfsr ^= cw_lfsr << 13;  
                cw_lfsr ^= cw_lfsr >> 17;  
                cw_lfsr ^= cw_lfsr << 5;  
                int32_t noise = ((int32_t)(cw_lfsr & 0xFFFF) - 32768) >> 6; // ~±512  
                noise = (noise * (int32_t)cw_noise) >> 9;  
                y = cw_push(sat16(y + noise));  
            }  
            if (notch_on) y = notch_lms_push(y);  
            if (nr_on)    y = nr_lms_push(y);  
            int16_t s = agc_limit(y);        
            if (rx_notch_on) s = (int16_t)rx_notch_push(s);  // гребёнка — после АРУ,  
                                                           // чтобы контур не «отменял» нотчи  
            s = af_voice_mix(s);          // голосовые сообщения        
            if (ramp_on) {    
                // Пила: +100 на отсчёт при 15.625 ksps → период ~21 мс,    
                // амплитуда полная. Скачок ≠ +100 (mod 65536) = потеря/дубль.    
                s = ramp_val;    
                ramp_val += 100;    
            }    
            out_emit(sat16(s));  
        }  
        af_capture_release();    
    
        const uint32_t dt = time_us_32() - t0;    
        core1_busy_us += dt;    
        if (dt > core1_max_us) core1_max_us = dt;   // пиковая стоимость блока    
  
        // Парковка между блоками, а не только между вызовами:    
        // flash-операция не ждёт конца дрейна всей очереди.    
        af_store_core1_park_poll();  
    }    
    
    core1_update_load(time_us_32());    
}    
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
uint32_t af_dsp_load_permille(void) { return dsp_load_permille; }    
uint32_t af_dsp_out_dropped(void)   { return out_pwm.dropped + out_usb.dropped; }    
uint32_t af_dsp_clip_count(void)    { return clip_count; }    
    
// Раздельная диагностика очередей: кто теряет и кто голодает.    
uint32_t af_dsp_out_dropped_usb(void)    { return out_usb.dropped; }    
uint32_t af_dsp_out_dropped_pwm(void)    { return out_pwm.dropped; }    
uint32_t af_dsp_out_underruns_usb(void)  { return out_usb.underruns; }    
uint32_t af_dsp_out_underruns_pwm(void)  { return out_pwm.underruns; }    
uint32_t af_dsp_out_published_usb(void)  { return out_usb.published; }    
uint32_t af_dsp_out_published_pwm(void)  { return out_pwm.published; }    
uint8_t  af_dsp_out_fill_max_usb(void)   { return out_usb.fill_max; }    
uint8_t  af_dsp_out_fill_max_pwm(void)   { return out_pwm.fill_max; }    
  
// Пиковое время обработки одного ADC-блока, мкс. Чтение сбрасывает пик.  
uint32_t af_dsp_max_block_us(void) {  
    uint32_t v = core1_max_us;  
    core1_max_us = 0;  
    return v;  
}

uint8_t af_dsp_out_fill_pwm(void) {  
    uint8_t n = 0;  
    for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++)  
        if (out_pwm.busy[b]) ++n;  
    return n;  
}

uint32_t af_dsp_band_lo_hz(void) { return af_lo_hz; }  
uint32_t af_dsp_band_hi_hz(void) { return af_hi_hz; }