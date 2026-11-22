// src/af_dsp.cpp  
// ============================================================================  
// af_dsp.cpp — моно DSP-тракт НЧ (Core 1)  
//  
// Тракт: uint16-блоки ADC (495 ksps) -> DC-EMA -> CIC /4 -> HB /2 -> HB /2  
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
#include "af_rec.h"       // запись/воспроизведение цифрового магнитофона
#include "af_snr.h"
#include "modes/cw_rx.h"   // CW-декодер: cwdec_push (Core 1) / cwdec_poll (Core 0)  
#include "modes/cw_tx.h"   // CW-передатчик: cwtx_sample (Core 1) / cwtx_* (Core 0)  

static uint8_t nr_mode = 0;              // 0=LMS ALE, 1=spectral  
static volatile bool snr_reset_req = false;  // запрос сброса spec-NR (Core 0 -> Core 1)  
  
uint8_t af_dsp_nrmode(void) { return nr_mode; }  
  
// Core 0: только ставит флаг — сам сброс выполняет af_dsp_poll на Core 1,  
// иначе memset(snr_re/im/ola/tail) гоняется с af_snr_push -> битый кадр.  
void af_dsp_set_nrmode(uint8_t m) {  
    m = m ? 1 : 0;  
    if (nr_mode == m) return;            // повторный вызов — no-op, без лишнего сброса  
    nr_mode = m;  
    snr_reset_req = true;  
    __dmb();  
}  

// ---------------------------------------------------------------------------  
// Half-band дециматоры — коэффициенты 1:1 из rx_dsp.cpp приёмника  
// ---------------------------------------------------------------------------  
  
// Каскад 0: проверенный 15-отводный, ~80 дБ.  
// Сумма = 16384 + 2·(10030 − 2250 + 532 − 120) = 32768 → DC-gain = 1.0.  
#define HB0_NTAPS 15    
// Без const: таблица уходит в .data (RAM) — читается во время flash-erase    
// при записи магнитофона (Core 1 не паркуется, вариант B).    
static int16_t hb0_taps[HB0_NTAPS] = {    
    -120,     0,   532,     0, -2250,  
       0, 10030, 16384, 10030,     0,  
    -2250,     0,   532,     0,  -120  
};  
  
// Каскады 1 и 2: 31 отвод, окно Блэкмана-Харриса, ~90 дБ заграждения.  
#define HBX_NTAPS 31    
static int16_t hbx_taps[HBX_NTAPS] = {   // в RAM — см. hb0_taps    
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
  
// Два набора тапов: Core 0 пишет в неактивный, затем атомарно    
// переключает указатель — Core 1 всегда читает целостный набор.    
static int16_t af_fir_taps[2][AF_FIR_N];    
static volatile const int16_t* af_fir_taps_cur = af_fir_taps[0];    
static uint8_t af_fir_taps_w = 0;              // индекс записи (0/1)    
static int32_t af_fir_buf[AF_FIR_N] = {};      
static int     af_fir_idx = 0;  
// Второй экземпляр FIR — постфильтр после АРУ: режет внеполосный мусор,  
// который NR/makeup/АРУ поднимают за пределами полосы. Те же тапы.  
static int32_t af_fir_buf2[AF_FIR_N] = {};  
static int     af_fir_idx2 = 0;  
    
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
    const uint8_t w = af_fir_taps_w ^ 1;        // пишем в неактивный    
    for (int n = 0; n < AF_FIR_N; n++)    
        af_fir_taps[w][n] = (int16_t)lroundf(h[n] * norm * 32768.0f);    
    __dmb();                                     // тапы видны до указателя    
    af_fir_taps_cur = af_fir_taps[w];            // атомарная публикация    
    af_fir_taps_w   = w;    
}    
  
// Один отсчёт полосового FIR — симметричная свёртка (порт af_lp_push):  
// AF_FIR_N/2 умножений вместо AF_FIR_N, ~40% экономии.  
static inline int32_t __not_in_flash_func(af_fir_push)(int32_t x) {  
    af_fir_buf[af_fir_idx] = x;  
    if (++af_fir_idx >= AF_FIR_N) af_fir_idx = 0;  
  
    const int16_t* taps = (const int16_t*)af_fir_taps_cur;   // снимок указателя на весь блок 
    int64_t acc = 0;    
    int lo = af_fir_idx;    
    int hi = af_fir_idx + AF_FIR_N - 1;    
    if (hi >= AF_FIR_N) hi -= AF_FIR_N;    
    for (int k = 0; k < AF_FIR_N / 2; k++) {    
        acc += (int32_t)taps[k] * (af_fir_buf[lo] + af_fir_buf[hi]);    
        if (++lo >= AF_FIR_N) lo = 0;    
        if (--hi < 0)        hi = AF_FIR_N - 1;    
    }    
    acc += (int32_t)taps[AF_FIR_N / 2] * af_fir_buf[lo];      
    return (int32_t)((acc + (1 << 14)) >> 15);   // Q15 → масштаб, +0.5 LSB    
}    
  
// Постфильтр после АРУ — тот же набор тапов, своё состояние.  
static inline int32_t __not_in_flash_func(af_fir_push2)(int32_t x) {  
    af_fir_buf2[af_fir_idx2] = x;  
    if (++af_fir_idx2 >= AF_FIR_N) af_fir_idx2 = 0;  
  
    const int16_t* taps = (const int16_t*)af_fir_taps_cur;  
    int64_t acc = 0;  
    int lo = af_fir_idx2;  
    int hi = af_fir_idx2 + AF_FIR_N - 1;  
    if (hi >= AF_FIR_N) hi -= AF_FIR_N;  
    for (int k = 0; k < AF_FIR_N / 2; k++) {  
        acc += (int32_t)taps[k] * (af_fir_buf2[lo] + af_fir_buf2[hi]);  
        if (++lo >= AF_FIR_N) lo = 0;  
        if (--hi < 0)        hi = AF_FIR_N - 1;  
    }  
    acc += (int32_t)taps[AF_FIR_N / 2] * af_fir_buf2[lo];  
    return (int32_t)((acc + (1 << 14)) >> 15);  
}  

// ---------------------------------------------------------------------------  
// Фиксированные узкие нотчи RX — гребёнка наводок 1/2/3/4/5 кГц.
// Биквад-нотч (RBJ notch), Q15, Direct Form I. Частоты зашиты в  
// af_config.h — коэффициенты считаются один раз в af_dsp_init.  
// ---------------------------------------------------------------------------  
static volatile bool rx_notch_on = (AF_RX_NOTCH_ON_DEFAULT != 0);  
  
typedef struct {  
    int32_t b1, a1, a2;          // Q15; b0=b2=1 (нормированный нотч)  
    int32_t x1, x2, y1, y2;  
} rx_notch_t;  
  
static rx_notch_t rx_notch[5];    
    
static void rx_notch_design(void) {    
    const float r  = (float)AF_RX_NOTCH_R_Q15 / 32768.0f;    
    const float hz[5] = { (float)AF_RX_NOTCH1_HZ, (float)AF_RX_NOTCH2_HZ,    
                          (float)AF_RX_NOTCH3_HZ, (float)AF_RX_NOTCH4_HZ,    
                          (float)AF_RX_NOTCH5_HZ };    
    for (int i = 0; i < 5; i++) {    
        const float w = 2.0f * PI * hz[i] / (float)AF_FS_OUT_HZ;  
        rx_notch[i].b1 = (int32_t)lrintf(-2.0f * cosf(w) * 32768.0f);  
        rx_notch[i].a1 = (int32_t)lrintf(-2.0f * r * cosf(w) * 32768.0f);  
        rx_notch[i].a2 = (int32_t)lrintf(r * r * 32768.0f);  
        rx_notch[i].x1 = rx_notch[i].x2 = rx_notch[i].y1 = rx_notch[i].y2 = 0;  
    }  
    __dmb();  
}  
  
static inline int32_t __not_in_flash_func(rx_notch_push)(int32_t x) {    
    for (int i = 0; i < 5; i++) {    
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
// Dynamic Presence EQ RX: SNR-адаптивный подъём ~2.3 кГц.  
// Порт AF_TX_PRES_* из af_tx.cpp. Peaking-биквад после АРУ (урок BUG-007:  
// АЧХ-формирующие каскады — после контура АРУ). k_dyn управляется  
// огибающей ВЧ-полосы с трекером шумового пола: в паузах k→0, шум не  
// поднимается. Всё int32/Q15/Q24, деления только per-block.  
// ---------------------------------------------------------------------------  
static volatile bool     rx_pres_on      = (AF_RX_PRES_ON_DEFAULT != 0);  
static volatile uint32_t rx_pres_freq_hz = AF_RX_PRES_FREQ_HZ;  
static volatile uint32_t rx_pres_q_x100  = AF_RX_PRES_Q_X100;  
static volatile int32_t  rx_pres_max_q15 = AF_RX_PRES_MAX_Q15;  
static int32_t           rx_presmax_db_x10 = 15;   // дБ x10 — для stats и банка  

// --- Noise Blanker RX (125 ksps, после CIC до HB) ---  
static volatile bool nb_on = AF_NB_ON_DEFAULT;  
static volatile int32_t nb_thr_x10 = AF_NB_THR_X10;  
static int32_t  nb_env = 8192;       // медленная EMA |y|, старт ~средний уровень  
static int32_t  nb_last = 0;         // последний «чистый» отсчёт (hold)  
static uint8_t  nb_hold_cnt = 0;     // остаток удержания blank  
static volatile uint32_t nb_hits = 0;// срабатываний с прошлого чтения stats  

// HP-детектор ~1.5 кГц: отсекает гласные/F1, огибающая реагирует на согласные.  
static int32_t rxp_hp_x1 = 0, rxp_hp_x2 = 0, rxp_hp_y1 = 0, rxp_hp_y2 = 0;  
static int32_t rxp_hp_b0 = 0, rxp_hp_b1 = 0, rxp_hp_b2 = 0,  
               rxp_hp_a1 = 0, rxp_hp_a2 = 0;  
  
// Peaking-биквад (Q15 DF-I, полный набор коэффициентов — глубина меняется).  
static int32_t rxp_pk_x1 = 0, rxp_pk_x2 = 0, rxp_pk_y1 = 0, rxp_pk_y2 = 0;  
static int32_t rxp_pk_b0 = 32768, rxp_pk_b1 = 0, rxp_pk_b2 = 0,  
               rxp_pk_a1 = 0, rxp_pk_a2 = 0;   // старт = байпас  
  
static int64_t rxp_env_q16    = (int64_t)256 << 16;  // EMA |hp|, Q16  
static int64_t rxp_nf_q16     = (int64_t)256 << 16;  // noise floor, Q16  
static int64_t rx_in_lvl_q16  = (int64_t)256 << 16;  // EMA |x| после FIR, Q16 (S-meter)  
static int32_t rx_in_peak     = 0;                   // max |x| после FIR; чтение сбрасывает  
static int32_t rxp_k_q15      = 0;                   // сглаженный k_dyn, Q15  
static int32_t rxp_k_tgt_q15  = 0;                   // цель k (per-block)  
static uint8_t rxp_decim      = 0;                   // дециматор sidechain ÷2  
  
// Проектирование HP-детектора (RBJ high-pass). Core 0, float допустим.  
static void rx_pres_hp_design(void) {  
    const float w = 2.0f * PI * (float)AF_RX_PRES_HP_HZ / (float)AF_FS_OUT_HZ;  
    const float alpha = sinf(w) / (2.0f * 0.7071f);   // Q = 0.707  
    const float a0 = 1.0f + alpha;  
    rxp_hp_b0 = (int32_t)lrintf((1.0f + cosf(w)) * 0.5f / a0 * 32768.0f);  
    rxp_hp_b1 = (int32_t)lrintf(-(1.0f + cosf(w)) / a0 * 32768.0f);  
    rxp_hp_b2 = rxp_hp_b0;  
    rxp_hp_a1 = (int32_t)lrintf(-2.0f * cosf(w) / a0 * 32768.0f);  
    rxp_hp_a2 = (int32_t)lrintf((1.0f - alpha) / a0 * 32768.0f);  
    __dmb();  
}  
  
// Проектирование peaking-биквада с заданным подъёмом gain_q15 (32768 = +0 дБ,  
// 32768+rx_pres_max_q15 = полный подъём). Core 0 / per-block, float допустим.  
static void __not_in_flash_func(rx_pres_pk_design)(int32_t gain_q15) {  
    const float A  = (float)gain_q15 / 32768.0f;   // линейное усиление пика:  
                                                  // max_q15 = (A_max-1)*32768   
    const float w  = 2.0f * PI * (float)rx_pres_freq_hz / (float)AF_FS_OUT_HZ;  
    const float Q  = (float)rx_pres_q_x100 / 100.0f;  
    const float alpha = sinf(w) / (2.0f * Q);  
    const float a0 = 1.0f + alpha / A;  
    rxp_pk_b0 = (int32_t)lrintf((1.0f + alpha * A) / a0 * 32768.0f);  
    rxp_pk_b1 = (int32_t)lrintf(-2.0f * cosf(w) / a0 * 32768.0f);  
    rxp_pk_b2 = (int32_t)lrintf((1.0f - alpha * A) / a0 * 32768.0f);  
    rxp_pk_a1 = rxp_pk_b1;  
    rxp_pk_a2 = (int32_t)lrintf((1.0f - alpha / A) / a0 * 32768.0f);  
    __dmb();  
}  
  
// Один отсчёт: HP-детектор + EMA + peaking. k_dyn пересчитывается per-block  
// в af_dsp_poll (децимировано ÷2) — здесь только фильтрация.  
static inline int32_t __not_in_flash_func(rx_pres_push)(int32_t x) {  
    // HP-детектор (DF-I, Q15)  
    int64_t acc = (int64_t)rxp_hp_b0 * x + (int64_t)rxp_hp_b1 * rxp_hp_x1  
                + (int64_t)rxp_hp_b2 * rxp_hp_x2  
                - (int64_t)rxp_hp_a1 * rxp_hp_y1  
                - (int64_t)rxp_hp_a2 * rxp_hp_y2;  
    int32_t hp = (int32_t)(acc >> 15);  
    rxp_hp_x2 = rxp_hp_x1; rxp_hp_x1 = x;  
    rxp_hp_y2 = rxp_hp_y1; rxp_hp_y1 = hp;  
  
    // EMA |hp| Q16 + сглаживание k: децимировано ÷2.  
    if (++rxp_decim >= 2) {  
        rxp_decim = 0;  
        int64_t ahp = (hp < 0) ? -(int64_t)hp : (int64_t)hp;  
        int64_t tgt = ahp << 16;  
        int64_t d = tgt - rxp_env_q16;  
        rxp_env_q16 += (d * (d > 0 ? (int64_t)AF_RX_PRES_ENV_ATT_Q24  
                                   : (int64_t)AF_RX_PRES_ENV_REL_Q24)) >> 24;  
  
        // k -> rxp_k_tgt_q15: атака ~20 мс, спад ~200 мс  
        // (константы Q24 как AF_TX_PRES_K_*, но вызов децимирован ÷2 —  
        //  постоянные времени вдвое длиннее TX-номинала).  
        int32_t dk = rxp_k_tgt_q15 - rxp_k_q15;  
        rxp_k_q15 += (int32_t)(((int64_t)dk *  
            (dk > 0 ? (int64_t)AF_RX_PRES_K_ATT_Q24  
                    : (int64_t)AF_RX_PRES_K_REL_Q24)) >> 24);  
    }  
  
    // Peaking-биквад (DF-I, Q15); при k=0 коэффициенты = байпас  
    acc = (int64_t)rxp_pk_b0 * x + (int64_t)rxp_pk_b1 * rxp_pk_x1  
        + (int64_t)rxp_pk_b2 * rxp_pk_x2  
        - (int64_t)rxp_pk_a1 * rxp_pk_y1  
        - (int64_t)rxp_pk_a2 * rxp_pk_y2;  
    int32_t y = (int32_t)(acc >> 15);  
    rxp_pk_x2 = rxp_pk_x1; rxp_pk_x1 = x;  
    rxp_pk_y2 = rxp_pk_y1; rxp_pk_y1 = y;  
    return y;  
}  
  
// Per-block пересчёт k_dyn и коэффициентов пика. Вызывается из af_dsp_poll  
// один раз на выходной отсчётный блок (не per-sample).  
static void __not_in_flash_func(rx_pres_block_update)(void) {  
    int32_t env = (int32_t)(rxp_env_q16 >> 16);  
    int32_t nf  = (int32_t)(rxp_nf_q16 >> 16);  
  
    // Трекер nf: вниз — быстрая EMA 1/64 (следует за падением шума,  
    // но не за одиночным фединговым провалом), вверх — медленная  
    // EMA 1/4096; только при низкой огибающей (env < 2*nf — речь  
    // не попадает в оценку шума).  
    if (env < nf) {  
        rxp_nf_q16 += (rxp_env_q16 - rxp_nf_q16) >> 6;   // вниз — EMA 1/64  
    } else if (env < 2 * nf) {  
        rxp_nf_q16 += (rxp_env_q16 - rxp_nf_q16) >> AF_RX_PRES_NF_UP_SHIFT;  
    }  
    if (rxp_nf_q16 < ((int64_t)AF_RX_PRES_NF_MIN << 16))  
        rxp_nf_q16 = (int64_t)AF_RX_PRES_NF_MIN << 16;  
    nf = (int32_t)(rxp_nf_q16 >> 16);  
  
    // k = clamp((env - 2*nf) / (8*nf), 0, 1) в Q15 — одно деление per-block.  
    int32_t num = env - 2 * nf;  
    int32_t den = 8 * nf;  
    int32_t k = (num <= 0) ? 0 : (num >= den) ? 32768  
                              : (int32_t)(((int64_t)num << 15) / den);  
  
    rxp_k_tgt_q15 = k;   // сглаживание — в rx_pres_push (per-sample, ÷2)  
  
    // Пересчёт коэффициентов пика под текущий сглаженный k.  
    rx_pres_pk_design(32768 + (int32_t)(((int64_t)rxp_k_q15 * rx_pres_max_q15) >> 15));  
}  

// ---------------------------------------------------------------------------  
// RX low-shelf («бас») — статический RBJ low-shelf биквад, +0..+6 дБ  
// ниже rx_shelf_freq_hz. Стоит в конце тракта (после пост-FIR и presence):  
// подъём НЧ не попадает в контур АРУ и не съедается лимитером.  
// При rx_shelf_db_x10 == 0 — байпас без нагрузки.  
// ---------------------------------------------------------------------------  
static volatile int32_t  rx_shelf_db_x10  = AF_RX_SHELF_DB_X10_DEFAULT;  
static volatile uint32_t rx_shelf_freq_hz = AF_RX_SHELF_FREQ_HZ;  
static volatile uint32_t rx_shelf_q_x100  = AF_RX_SHELF_Q_X100;  
  
static int32_t rxs_b0 = 32768, rxs_b1 = 0, rxs_b2 = 0,  
               rxs_a1 = 0,     rxs_a2 = 0;   // старт = байпас  
static int32_t rxs_x1 = 0, rxs_x2 = 0, rxs_y1 = 0, rxs_y2 = 0;  
  
// Проектирование RBJ low-shelf. Core 0, float допустим.  
static void rx_shelf_design(void) {  
    const float A     = powf(10.0f, (float)rx_shelf_db_x10 / 400.0f); // dB/40  
    const float w     = 2.0f * PI * (float)rx_shelf_freq_hz / (float)AF_FS_OUT_HZ;  
    const float Q     = (float)rx_shelf_q_x100 / 100.0f;  
    const float alpha = sinf(w) / (2.0f * Q);  
    const float cw_   = cosf(w);  
    const float sq    = 2.0f * sqrtf(A) * alpha;  
    const float a0    = (A + 1.0f) + (A - 1.0f) * cw_ + sq;  
    rxs_b0 = (int32_t)lrintf(A * ((A + 1.0f) - (A - 1.0f) * cw_ + sq) / a0 * 32768.0f);  
    rxs_b1 = (int32_t)lrintf(2.0f * A * ((A - 1.0f) - (A + 1.0f) * cw_) / a0 * 32768.0f);  
    rxs_b2 = (int32_t)lrintf(A * ((A + 1.0f) - (A - 1.0f) * cw_ - sq) / a0 * 32768.0f);  
    rxs_a1 = (int32_t)lrintf(-2.0f * ((A - 1.0f) + (A + 1.0f) * cw_) / a0 * 32768.0f);  
    rxs_a2 = (int32_t)lrintf(((A + 1.0f) + (A - 1.0f) * cw_ - sq) / a0 * 32768.0f);  
    __dmb();  
}  
  
static inline int32_t __not_in_flash_func(rx_shelf_push)(int32_t x) {  
    int64_t acc = (int64_t)rxs_b0 * x + (int64_t)rxs_b1 * rxs_x1  
                + (int64_t)rxs_b2 * rxs_x2  
                - (int64_t)rxs_a1 * rxs_y1  
                - (int64_t)rxs_a2 * rxs_y2;  
    int32_t y = (int32_t)(acc >> 15);  
    rxs_x2 = rxs_x1; rxs_x1 = x;  
    rxs_y2 = rxs_y1; rxs_y1 = y;  
    return y;  
}  
  
// ---------------------------------------------------------------------------  
// RX harmonic exciter — полином Чебышёва T2(x) = 2x² − 1.  
// Pre-HP ~1.5 кГц выделяет полосу речи; T2 даёт чистую 2-ю гармонику  
// (3–6 кГц) + DC; post-HP ~2.5 кГц отбирает гармонику и отрезает DC  
// и исходную полосу. Подмес rx_exc_mix_q15 (дефолт ~10%).  
// Чётная нелинейность: нет нечётных гармоник и интермода внутри полосы;  
// на тихом сигнале/шуме подмес ∝ x² сам затухает.  
// ---------------------------------------------------------------------------  
static volatile bool    rx_exc_on      = (AF_RX_EXC_ON_DEFAULT != 0);  
static volatile int32_t rx_exc_mix_q15 = AF_RX_EXC_MIX_Q15_DEFAULT;  
static int32_t          rxe_harm_lvl   = 0;   // EMA |hp2| — уровень гармоник до подмеса  
static int32_t          rxe_harm_peak  = 0;   // пик |hp2|; чтение сбрасывает (как rx_in_peak)  
  
// Два ФВЧ-биквада (RBJ high-pass, Q=0.707), коэффициенты — один раз в init.  
static int32_t rxe_hp1_b0, rxe_hp1_b1, rxe_hp1_b2, rxe_hp1_a1, rxe_hp1_a2;  
static int32_t rxe_hp2_b0, rxe_hp2_b1, rxe_hp2_b2, rxe_hp2_a1, rxe_hp2_a2;  
static int32_t rxe_hp1_x1 = 0, rxe_hp1_x2 = 0, rxe_hp1_y1 = 0, rxe_hp1_y2 = 0;  
static int32_t rxe_hp2_x1 = 0, rxe_hp2_x2 = 0, rxe_hp2_y1 = 0, rxe_hp2_y2 = 0;  
  
static void rx_exc_hp_design(int32_t* b0, int32_t* b1, int32_t* b2,  
                             int32_t* a1, int32_t* a2, float hz) {  
    const float w = 2.0f * PI * hz / (float)AF_FS_OUT_HZ;  
    const float alpha = sinf(w) / (2.0f * 0.7071f);  
    const float a0 = 1.0f + alpha;  
    *b0 = (int32_t)lrintf((1.0f + cosf(w)) * 0.5f / a0 * 32768.0f);  
    *b1 = (int32_t)lrintf(-(1.0f + cosf(w)) / a0 * 32768.0f);  
    *b2 = *b0;  
    *a1 = (int32_t)lrintf(-2.0f * cosf(w) / a0 * 32768.0f);  
    *a2 = (int32_t)lrintf((1.0f - alpha) / a0 * 32768.0f);  
}  
  
static void rx_exc_design(void) {  
    rx_exc_hp_design(&rxe_hp1_b0, &rxe_hp1_b1, &rxe_hp1_b2,  
                     &rxe_hp1_a1, &rxe_hp1_a2, (float)AF_RX_EXC_PRE_HZ);  
    rx_exc_hp_design(&rxe_hp2_b0, &rxe_hp2_b1, &rxe_hp2_b2,  
                     &rxe_hp2_a1, &rxe_hp2_a2, (float)AF_RX_EXC_POST_HZ);  
    __dmb();  
}  
  
static inline int32_t __not_in_flash_func(rx_exc_push)(int32_t x) {  
    // pre-HP: выделяем полосу >AF_RX_EXC_PRE_HZ  
    int64_t acc = (int64_t)rxe_hp1_b0 * x + (int64_t)rxe_hp1_b1 * rxe_hp1_x1  
                + (int64_t)rxe_hp1_b2 * rxe_hp1_x2  
                - (int64_t)rxe_hp1_a1 * rxe_hp1_y1  
                - (int64_t)rxe_hp1_a2 * rxe_hp1_y2;  
    int32_t hp1 = (int32_t)(acc >> 15);  
    rxe_hp1_x2 = rxe_hp1_x1; rxe_hp1_x1 = x;  
    rxe_hp1_y2 = rxe_hp1_y1; rxe_hp1_y1 = hp1;  
  
    // x·|x| — чётная нелинейность, та же 2-я гармоника, но без DC:  
    // на тишине t2 = 0, нет транзиента включения и выбросов post-HP.  
    int32_t t2 = (hp1 * ((hp1 < 0) ? -hp1 : hp1)) >> 15;  
  
    // post-HP: отбираем 2-ю гармонику, отрезаем DC и исходную полосу  
    acc = (int64_t)rxe_hp2_b0 * t2 + (int64_t)rxe_hp2_b1 * rxe_hp2_x1  
        + (int64_t)rxe_hp2_b2 * rxe_hp2_x2  
        - (int64_t)rxe_hp2_a1 * rxe_hp2_y1  
        - (int64_t)rxe_hp2_a2 * rxe_hp2_y2;  
    int32_t hp2 = (int32_t)(acc >> 15);  
    rxe_hp2_x2 = rxe_hp2_x1; rxe_hp2_x1 = t2;  
    rxe_hp2_y2 = rxe_hp2_y1; rxe_hp2_y1 = hp2;  
  
    // Диагностика: EMA |hp2| k=1/64 (~4 мс) + пик — уровень гармоник  
    // до подмеса. Реальный вклад в выход = lvl * rx_exc_mix_q15 >> 15.  
    int32_t ah = (hp2 < 0) ? -hp2 : hp2;  
    rxe_harm_lvl += (ah - rxe_harm_lvl) >> 6;  
    if (ah > rxe_harm_peak) rxe_harm_peak = ah;  
  
    return x + (int32_t)(((int64_t)hp2 * rx_exc_mix_q15) >> 15);  
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
static volatile bool     cwdec_on = true;   // декодер морзянки активен при cw on
static volatile bool rtty_on = false; 
  
static volatile uint32_t cw_noise = 0;     // подмес шума выключен: на чистом    
                                           // тоне размывал спектр и мешал декодеру  
  
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
static volatile uint8_t cw_updating = 0;   // 1 = коэффициенты пересчитываются  
  
// Проектирование одной секции: Q = f0 / BW_sec, alpha = sin(w0)/(2Q).  
// Вызывается из af_dsp_set_cw_* (Core 0, редкая операция, float допустим).  
static void cw_design(void) {    
    cw_updating = 1;    
    __dmb();  
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
    cw_updating = 0;    
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
    if (cw_updating) return x;   // Core 0 пересчитывает — пропускаем отсчёт    
    x <<= 4;    
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
  
// Шаг NLMS — runtime-параметр (дефайн = значение по умолчанию, команда ntmu).  
static volatile int32_t notch_mu = NOTCH_MU_Q;  
  
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
        // 32-битный mul (1 такт) вместо __muldi3: w и v <= 2^15,      
        // произведение <= 2^30 — влезает в int32, сумма — в int64.      
        acc += (int32_t)(notch_w[k] * v);         // y в Q15      
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
        // квадраты <= 2^30 — 32-битные mul, разность накапливаем в int64    
        notch_energy += (int64_t)(v_new * v_new - v_old * v_old);      
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
    int64_t g = (((int64_t)e * notch_mu) << 15) >> k;  
    if (g >  NOTCH_G_MAX) g =  NOTCH_G_MAX;  
    if (g < -NOTCH_G_MAX) g = -NOTCH_G_MAX; 
  
    // g <= 2^20 (NOTCH_G_MAX): сдвигаем до int32 — g5 <= 2^15,      
    // g5*tap <= 2^30 влезает в int32, итоговый сдвиг 15-5=10.      
    int32_t g5 = (int32_t)(g >> 5);    
    for (int k = 0; k < NOTCH_N; k++) {    
        int32_t upd = (g5 * taps[k]) >> 10;    
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
  
// Runtime-параметры NR (дефайны = значения по умолчанию, команды nrmu/nrdepth/nrmakeup).  
static volatile int32_t nr_mu         = NR_MU_Q;  
static volatile int32_t nr_depth_q15  = NR_DEPTH_Q15;  
static volatile int32_t nr_makeup_q15 = NR_MAKEUP_Q15;  
  
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
        // 32-битные mul вместо __muldi3: произведения <= 2^30    
        acc    += (int32_t)(nr_w[k] * xk);    
        energy += (int32_t)(xk * xk);    
        if (--p < 0) p += NR_N + NR_DELAY;    
    }    
    int32_t y = (int32_t)(acc >> 15);  
    int32_t e = x - y;                         // ошибка = шум  
  
    nr_pow += (energy - nr_pow) >> NR_PWR_SHIFT;  
  
    int64_t den = nr_pow + NR_EPS;  
    int k = 63 - __builtin_clzll((uint64_t)den);  
    int64_t g = (((int64_t)e * nr_mu) << 15) >> k;  
    if (g >  NR_G_MAX) g =  NR_G_MAX;  
    if (g < -NR_G_MAX) g = -NR_G_MAX;  
  
    // g <= 2^20 (NR_G_MAX): g5 <= 2^15, произведение <= 2^30 — int32    
    int32_t g5 = (int32_t)(g >> 5);    
    for (int k = 0; k < NR_N; k++) {    
        int32_t upd = (g5 * nr_buf[taps[k]]) >> 10;    
        int32_t w   = (int32_t)nr_w[k];    
        w += upd - (w >> NR_LEAK_SHIFT);    
        if (w >  32767) w =  32767;    
        if (w < -32768) w = -32768;    
        nr_w[k] = (int16_t)w;    
    }    
  
    // dry/wet с настраиваемой глубиной: out = x − nr_depth_q15·e    
    int32_t out = x - (int32_t)(((int64_t)e * nr_depth_q15) >> 15);    
    // makeup-gain — компенсация просадки уровня    
    out = (int32_t)(((int64_t)out * nr_makeup_q15) >> 15);  
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
static volatile bool     dsp_reset_req = false; // запрос сброса состояния (после save)  
  
// Runtime-параметры АРУ/лимитера SSB-ветки (дефайны = значения по умолчанию).  
// CW-режим использует CW_AGC_* — команды его не трогают.  
static volatile int32_t agc_ref      = AGC_REF;        // целевой уровень АРУ  
static volatile float   agc_max_gain = AGC_MAX_GAIN;   // потолок усиления  
static volatile float   agc_min_gain = AGC_MIN_GAIN;   // нижний предел  
static volatile int32_t lim_ceil     = LIMIT_CEIL;     // потолок лимитера  

static volatile bool tx_on = false;   // false=RX, true=TX (PTT GP8 / "tx on")    
static volatile bool cwtx_started = false; // TX поднят командой `cw tx` —  
                                           // авто-возврат в RX по концу посылки  
  
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
        const float max_g = cw_on ? CW_AGC_MAX_GAIN : agc_max_gain;    
        const float min_g = cw_on ? CW_AGC_MIN_GAIN : agc_min_gain;    
        int32_t avg_lo = (int32_t)(agc_ref / max_g);    
        int32_t avg_hi = (int32_t)(agc_ref / min_g);  
        if (avg < avg_lo) avg = avg_lo;  
        if (avg > avg_hi) avg = avg_hi;  
        // Точное деление вместо >>k: сдвиг по старшему биту давал  
        // gain с ошибкой до 2x и СКАЧКИ gain при пересечении avg  
        // границ степени двойки -> AM-модуляция -> гребёнка палок.  
        // int64-деление ~100 циклов x 15.5 ksps = ~0.6% Core 1.  
        y1 = (int32_t)(((int64_t)x * agc_ref) / avg);  
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
    if (lpk > lim_ceil) {    
        y1 = (int32_t)(((int64_t)y1 * lim_ceil) / lpk);    
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
  
// Мягкий клиппер хвоста RX: кубический soft-clip (как tx_soft_clip).  
// |x| <= 32768: y = x − x³/(3·32768²) — плавное насыщение, без жёсткого  
// среза. |x| > 32768: асимптота ±(2/3·32768) = ±21845 — гарантированно  
// не клиппит, но и не даёт выйти за ~−3.5 dBFS. Применяется ОДИН раз  
// после shelf+exciter: эти каскады стоят за лимитером и могут поднять  
// пики выше int16 — мягкое насыщение вместо щелчков sat16.  
static inline int16_t __not_in_flash_func(rx_soft_clip)(int32_t x) {  
    if (x >  32768) return  21845;  
    if (x < -32768) return -21845;  
    int32_t x2 = (x * x) >> 15;              // x² Q15  
    return (int16_t)(x - ((x2 * x) >> 15) / 3);  // x − x³/3 (Q15)  
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
    volatile uint32_t empty_reads;                // опросы при пустой очереди (косметика, не пропуски)  
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
    // Пустая очередь — НЕ underrun: consumer (af_pwm_poll/af_usb_poll)  
    // опрашивает из loop() на сотнях кГц, производитель даёт ~242 блока/с —  
    // пустой ответ штатен в 99.9% вызовов. Реальный пропуск звука считает  
    // DMA-сторона: af_pwm_underruns() / af_usb underruns.  
    if (q->tail == q->head) { ++q->empty_reads; return NULL; }  
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
  
// Возврат: 0 = выключено, 1 = включено, 2 = включено и NR снят (приоритет NT),  
//          3 = отклонено (CW on — NT в узкой полосе бесполезен и перегружает Core 1).  
int af_dsp_set_notch(bool on) {    
    // Блокировка снята для теста: CW + NT и NT + NR допустимы.  
    // ВНИМАНИЕ: суммарная нагрузка может превысить окно блока Core 1 —  
    // контролировать max_block_us в stats.  
    // if (on && cw_on) return 3;      // запрет: CW + NT = перегрузка Core 1      
    int rc = on ? 1 : 0;      
    // if (on && nr_on) {              // NT имеет приоритет — снимаем NR      
    //     af_dsp_set_nr(false);      
    //     rc = 2;      
    // }
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
        for (int i = 0; i < 5; i++)    
            rx_notch[i].x1 = rx_notch[i].x2 = rx_notch[i].y1 = rx_notch[i].y2 = 0;    
    }    
    rx_notch_on = on;  
}  
bool af_dsp_rxnotch_on(void) { return rx_notch_on; }

void af_dsp_set_rxpres(bool on) {  
    if (on && !rx_pres_on) {              // чистый старт — без щелчка  
        rxp_hp_x1 = rxp_hp_x2 = rxp_hp_y1 = rxp_hp_y2 = 0;  
        rxp_pk_x1 = rxp_pk_x2 = rxp_pk_y1 = rxp_pk_y2 = 0;  
        rxp_env_q16 = rxp_nf_q16 = (int64_t)256 << 16;  
        rxp_k_q15 = rxp_k_tgt_q15 = 0;  
        // rx_pres_pk_design здесь НЕ вызываем: пишет rxp_pk_* с Core 0,  
        // а block_update пишет их же с Core 1 — гонка на один блок.  
        // k=0 → ближайший block_update сам опубликует байпас-коэффициенты.  
    }  
    rx_pres_on = on;  
}  
bool     af_dsp_rxpres_on(void)  { return rx_pres_on; }  
void     af_dsp_set_rxpresmax(int32_t db_x10) {  
    if (db_x10 < 0)  db_x10 = 0;  
    if (db_x10 > 60) db_x10 = 60;         // потолок +6 дБ  
    rx_presmax_db_x10 = db_x10;           // для геттера/stats/банка  
    // дБx10 -> Q15 приращения: gain_q15 = 32768*(10^(dB/20) - 1)  
    rx_pres_max_q15 = (int32_t)lrintf((powf(10.0f, db_x10 / 200.0f) - 1.0f) * 32768.0f);  
}  
void     af_dsp_set_rxpresf(uint32_t hz) {  
    if (hz < 1000) hz = 1000;  
    if (hz > 4000) hz = 4000;  
    rx_pres_freq_hz = hz;  
}  
void     af_dsp_set_rxpresq(uint32_t q_x100) {  
    if (q_x100 < 30)  q_x100 = 30;  
    if (q_x100 > 300) q_x100 = 300;  
    rx_pres_q_x100 = q_x100;  
}  
int32_t  af_dsp_rxpres_k(void)    { return rxp_k_q15; }        // для stats  
int32_t  af_dsp_rxpres_env(void)  { return (int32_t)(rxp_env_q16 >> 16); }  
int32_t  af_dsp_rxpres_nf(void)   { return (int32_t)(rxp_nf_q16 >> 16); }  
int32_t  af_dsp_rxpresmax(void)   { return rx_presmax_db_x10; }  // дБ x10, не Q15  
uint32_t af_dsp_rxpresf(void)     { return rx_pres_freq_hz; }  
uint32_t af_dsp_rxpresq(void)     { return rx_pres_q_x100; }  
  
void af_dsp_set_rxshelf(int32_t db_x10) {  
    if (db_x10 < 0)  db_x10 = 0;  
    if (db_x10 > 60) db_x10 = 60;         // потолок +6 дБ  
    rx_shelf_db_x10 = db_x10;  
    rx_shelf_design();  
}  
void af_dsp_set_rxshelff(uint32_t hz) {  
    if (hz < 200) hz = 200;  
    if (hz > 800) hz = 800;  
    rx_shelf_freq_hz = hz;  
    rx_shelf_design();  
}  
void af_dsp_set_rxshelfq(uint32_t q_x100) {  
    if (q_x100 < 30)  q_x100 = 30;  
    if (q_x100 > 200) q_x100 = 200;  
    rx_shelf_q_x100 = q_x100;  
    rx_shelf_design();  
}  
int32_t  af_dsp_rxshelf(void)  { return rx_shelf_db_x10; }  
uint32_t af_dsp_rxshelff(void) { return rx_shelf_freq_hz; }  
uint32_t af_dsp_rxshelfq(void) { return rx_shelf_q_x100; }  
  
void af_dsp_set_rxexc(bool on) {  
    if (on && !rx_exc_on) {              // чистый старт — без щелчка  
        rxe_hp1_x1 = rxe_hp1_x2 = rxe_hp1_y1 = rxe_hp1_y2 = 0;  
        rxe_hp2_x1 = rxe_hp2_x2 = rxe_hp2_y1 = rxe_hp2_y2 = 0;  
        rxe_harm_lvl = 0;  rxe_harm_peak = 0;  
    }  
    rx_exc_on = on;  
}  
bool    af_dsp_rxexc_on(void) { return rx_exc_on; }  
void    af_dsp_set_rxexcmix(int32_t q15) {  
    if (q15 < 0)     q15 = 0;  
    if (q15 > 16384) q15 = 16384;         // потолок 50%  
    rx_exc_mix_q15 = q15;  
}  
int32_t af_dsp_rxexcmix(void) { return rx_exc_mix_q15; }  
int32_t af_dsp_rxexc_harm(void) { return rxe_harm_lvl; }  
int32_t af_dsp_rxexc_hpeak(void) {  
    int32_t v = rxe_harm_peak;  
    rxe_harm_peak = 0;              // чтение сбрасывает — как rx_in_peak  
    return v;  
}  


void     af_dsp_set_nb(bool on) {      
    // NB ЗАГЛУШЕН: детектор вешал звук и дёргал шумовую полку (b46d15c).    
    // Команды/поля банка оставлены для совместимости формата (VER 6),    
    // но включение игнорируется — nb_on всегда false.    
    // TODO: вернуться к NB — вероятно, нужен детектор на 495 ksps до CIC    
    // и/или критерий ширины импульса, а не только превышение огибающей.    
    (void)on;    
    nb_on = false;    
    nb_hold_cnt = 0;    
}    
bool     af_dsp_nb_on(void) { return nb_on; }  
void     af_dsp_set_nbthr(int32_t x10) {  
    if (x10 < AF_NB_THR_MIN) x10 = AF_NB_THR_MIN;  
    if (x10 > AF_NB_THR_MAX) x10 = AF_NB_THR_MAX;  
    nb_thr_x10 = x10;  
}  
int32_t  af_dsp_nbthr(void) { return nb_thr_x10; }  
// Срабатываний с прошлого чтения (stats/dump). Чтение сбрасывает.  
uint32_t af_dsp_nb_hits(void) { uint32_t h = nb_hits; nb_hits = 0; return h; }  
int32_t  af_dsp_rx_in_lvl(void)   { return (int32_t)(rx_in_lvl_q16 >> 16); }  
// Пик |x| после FIR с момента прошлого чтения (stats/dump). Чтение сбрасывает.  
int32_t  af_dsp_rx_in_peak(void)  { int32_t p = rx_in_peak; rx_in_peak = 0; return p; }  
  
// Возвращает false, если команда отклонена (NT on — приоритет NT;  
// CW on — NR в узкой полосе бесполезен и перегружает Core 1).    
bool af_dsp_set_nr(bool on) {        
    // Блокировка снята для теста: NT + NR и CW + NR допустимы.  
    // if (on && notch_on) return false;   // NT и NR взаимоисключающие      
    // if (on && cw_on)    return false;   // запрет: CW + NR = перегрузка Core 1       
    if (on && !nr_on) {              // чистый старт → без щелчка    
        memset(nr_buf, 0, sizeof(nr_buf));    
        memset(nr_w,   0, sizeof(nr_w));    
        nr_idx = 0;    
        nr_pow = 0;    
    }    
    nr_on = on;    
    return true;  
}

void af_dsp_set_rtty(bool on) {  
    if (on && cw_on) af_dsp_set_cw(false);   // взаимоисключение  
    if (on && !rtty_on) rttydec_init();  
    if (!on && rtty_on) rttydec_stop();  
    rtty_on = on;  
}  
bool af_dsp_rtty_on(void) { return rtty_on; } 

void af_dsp_set_cw(bool on) {    
    if (on && !cw_on) {        
        // Блокировка снята для теста: NT/NR больше не снимаются при CW.  
        // if (notch_on) af_dsp_set_notch(false);      
        // if (nr_on)    af_dsp_set_nr(false);      
        cw_design();      
        agc_avg_q16 = (int64_t)4096 << 16;      
        cwdec_init((float)cw_f0);   // гетеродин декодера под текущий f0    
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
        cwdec_stop();               // добить последнюю букву, очистить очередь    
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
    if (cw_on) { cw_band_follow(); cwdec_init((float)cw_f0); }    
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
  
// --- Настройка АРУ/лимитера (SSB-ветка; CW использует CW_AGC_*) ---  
void af_dsp_set_agcref(int32_t v) {  
    if (v < 1000)  v = 1000;  
    if (v > 32767) v = 32767;  
    agc_ref = v;  
}  
void af_dsp_set_agcmax(float g) {  
    if (g < 1.0f)    g = 1.0f;  
    if (g > 1000.0f) g = 1000.0f;  
    agc_max_gain = g;  
}  
void af_dsp_set_agcmin(float g) {  
    if (g < 0.01f) g = 0.01f;  
    if (g > 1.0f)  g = 1.0f;  
    agc_min_gain = g;  
}  
void af_dsp_set_limceil(int32_t v) {  
    if (v < 1000)  v = 1000;  
    if (v > 32767) v = 32767;  
    lim_ceil = v;  
}  
int32_t af_dsp_agcref(void)  { return agc_ref; }  
float   af_dsp_agcmax(void)  { return agc_max_gain; }  
float   af_dsp_agcmin(void)  { return agc_min_gain; }  
int32_t af_dsp_limceil(void) { return lim_ceil; }  
  
// --- Настройка NT/NR (шаги адаптации и глубина — экспертные) ---  
void af_dsp_set_ntmu(int32_t v) {  
    if (v < 0)     v = 0;  
    if (v > 32768) v = 32768;  
    notch_mu = v;  
}  
void af_dsp_set_nrmu(int32_t v) {  
    if (v < 0)     v = 0;  
    if (v > 32768) v = 32768;  
    nr_mu = v;  
}  
void af_dsp_set_nrdepth(int32_t v) {    
    if (v < 0)     v = 0;    
    if (v > 65536) v = 65536;    
    nr_depth_q15 = v;  
    af_snr_set_depth(v > 32768 ? 32768 : v);   // spec: beta Q15  
}    
void af_dsp_set_nrmakeup(int32_t v) {    
    if (v < 0)      v = 0;    
    if (v > 131072) v = 131072;   // до ×4    
    nr_makeup_q15 = v;    
    af_snr_set_makeup(v);         // spec: выходной gain Q15    
}    
int32_t af_dsp_ntmu(void)     { return notch_mu; }  
int32_t af_dsp_nrmu(void)     { return nr_mu; }  
int32_t af_dsp_nrdepth(void)  { return nr_depth_q15; }  
int32_t af_dsp_nrmakeup(void) { return nr_makeup_q15; }  
  
void af_dsp_set_bypass(bool on) { bypass_on = on; }  
// Запрос сброса состояния тракта — выполняется на Core 1 в af_dsp_poll.  
// Вызывать после af_capture_resume() в af_store_save.  
void af_dsp_reset_state(void) { dsp_reset_req = true; __dmb(); }  
void af_dsp_set_ramp(bool on) { ramp_on = on; ramp_val = 0; }  
bool af_dsp_ramp_on(void)     { return ramp_on; }  
bool     af_dsp_agc_on(void)     { return agc_on; }  
bool     af_dsp_bypass_on(void)  { return bypass_on; }
void af_dsp_set_cwdec(bool on) {    
    if (on && !cwdec_on && cw_on) cwdec_init((float)cw_f0);  // чистый старт    
    if (!on) cwdec_stop();    
    cwdec_on = on;    
}    
bool     af_dsp_cwdec_on(void) { return cwdec_on; }    
uint32_t af_dsp_cwdec_dropped(void) { return cwdec_dropped(); } 
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
    
// --- CW-передатчик (Core 0) ---    
// Постановка текста в очередь + подъём TX. Возврат в RX — автоматически    
// в loop() по !cwtx_busy() при cwtx_started.    
bool af_dsp_cwtx_start(const char* text) {      
    if (!cwtx_send(text)) return false;  // очередь занята — посылка не встала,  
                                         // TX не поднимаем, флаг не взводим  
    cwtx_set_tone((float)cw_f0);   // тон следует за текущим cw f0      
    cwtx_started = true;      
    af_dsp_set_tx(true);      
    return true;  
}      
bool af_dsp_cwtx_started(void) { return cwtx_started; }    
void af_dsp_cwtx_clear_started(void) { cwtx_started = false; }    
bool af_dsp_cwtx_busy(void) { return cwtx_busy(); }    
  
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
    rx_pres_hp_design();      // HP-детектор presence    
    rx_pres_pk_design(32768); // peaking = байпас при старте      
    rx_shelf_design();        // коэффициенты RX low-shelf  
    rx_exc_design();          // ФВЧ-биквады эксайтера  
    af_snr_init();            // таблицы окна/twiddles/bit-reverse спектрального NR    
    af_tx_init();         // TX: FIR-компрессор-клиппер        
    cwtx_init((float)cw_f0);  // NCO + LUT синуса CW-передатчика (Core 0, float)  
  
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
static inline void __not_in_flash_func(core1_update_load)(uint32_t now_us) {    
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
  
    // Сброс состояния тракта по запросу Core 0 (после af_store_save):  
    // пауза capture + парковка на flash-erase оставляла протухшие отсчёты  
    // в линиях задержки и сбитые EMA — на выходе гребёнка до ребута.  
    if (dsp_reset_req) {  
        dsp_reset_req = false;  
        dc_est = 0;  
        memset(cic_int, 0, sizeof(cic_int));  
        memset(cic_comb, 0, sizeof(cic_comb));  
        cic_cnt = 0;  
        for (int s = 0; s < AF_HB_STAGES; s++) {  
            memset(hb[s].d, 0, sizeof(hb[s].d));  
            hb[s].w = 0; hb[s].phase = 0;  
        }  
        memset(af_fir_buf, 0, sizeof(af_fir_buf));    
        af_fir_idx = 0;  
        memset(af_fir_buf2, 0, sizeof(af_fir_buf2));  
        af_fir_idx2 = 0;  
        memset(notch_buf, 0, sizeof(notch_buf));  
        memset(notch_w,   0, sizeof(notch_w));  
        memset(notch_v,   0, sizeof(notch_v));  
        notch_idx = notch_vidx = 0;  
        notch_energy = notch_pow = 0;  
        memset(nr_buf, 0, sizeof(nr_buf));  
        memset(nr_w,   0, sizeof(nr_w));  
        nr_idx = 0; nr_pow = 0;  
        for (int s = 0; s < AF_CW_SECTIONS; s++)  
            cw_sec[s].x1 = cw_sec[s].x2 = cw_sec[s].y1 = cw_sec[s].y2 = 0;  
        for (int i = 0; i < 5; i++)  
            rx_notch[i].x1 = rx_notch[i].x2 = rx_notch[i].y1 = rx_notch[i].y2 = 0;  
        rxp_hp_x1 = rxp_hp_x2 = rxp_hp_y1 = rxp_hp_y2 = 0;  
        rxp_pk_x1 = rxp_pk_x2 = rxp_pk_y1 = rxp_pk_y2 = 0;  
        rxp_env_q16 = rxp_nf_q16 = (int64_t)256 << 16;    
        rxp_k_q15 = rxp_k_tgt_q15 = 0;  
        rxs_x1 = rxs_x2 = rxs_y1 = rxs_y2 = 0;  
        rxe_hp1_x1 = rxe_hp1_x2 = rxe_hp1_y1 = rxe_hp1_y2 = 0;  
        rxe_hp2_x1 = rxe_hp2_x2 = rxe_hp2_y1 = rxe_hp2_y2 = 0;  
        rxe_harm_lvl = 0;  rxe_harm_peak = 0;  
        nb_env = 8192;  nb_last = 0;  nb_hold_cnt = 0;            
        agc_avg_q16  = (int64_t)4096 << 16;  
        lim_peak_q16 = (int64_t)2048 << 16;  
        rx_in_lvl_q16 = (int64_t)256 << 16;    
        rx_in_peak = 0;    
        out_fill_pos = 0;    
        af_snr_reset();          // кадры/OLA/шумовая оценка спектрального NR      
        __dmb();      
    }      
  
    // Отложенный сброс spec-NR по запросу Core 0 (nr spec|lms, load банка):  
    // выполняется здесь, на Core 1 — af_snr_push в этот момент не работает.  
    if (snr_reset_req) {  
        snr_reset_req = false;  
        af_snr_reset();  
        __dmb();  
    }  
        
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
#if 0   // NB ЗАГЛУШЕН — см. af_dsp_set_nb. Оставлено для будущей доработки.  
            // Noise Blanker: импульсные помехи до HB/FIR (125 ksps).    
            // |y| > env*thr -> hold последнего чистого отсчёта + AF_NB_HOLD    
            // на хвост. env — EMA |y| k=1/1024, обновляется значением,      
            // ограниченным порогом: импульс не задирает собственный порог.      
            if (nb_on) {    
                int32_t ay = (y < 0) ? -y : y;    
                int32_t thr = (nb_env * nb_thr_x10) / 10;    
                int32_t ev = (ay > thr) ? thr : ay;    
                nb_env += (ev - nb_env) >> 10;    
                if (nb_hold_cnt > 0 || ay > thr) {    
                    if (nb_hold_cnt == 0) nb_hits++;    
                    nb_hold_cnt = AF_NB_HOLD;    
                    y = nb_last;    
                } else {    
                    nb_last = y;    
                }    
                if (nb_hold_cnt > 0) nb_hold_cnt--;    
            }    
#endif    
            if (!hb_push(&hb[0], y, &y)) continue;    
            if (!hb_push(&hb[1], y, &y)) continue;  
            if (!hb_push(&hb[2], y, &y)) continue;  
            y >>= 3;    // компенсация DC-gain трёх HB (2^3 = 8)
  
            // НЧ-тракт на 15.625 ksps.    
            if (bypass_on) {      
                // Обход: децимированный вход напрямую на выход,      
                // без FIR/CW/NT/NR/АРУ/лимитера.  
                // голос должен быть слышен и в обходе  
                if (af_rec_active == 1) af_rec_push(sat16(y));  
                int16_t sb = af_voice_mix(sat16(y));  
                if (af_rec_active == 2) sb = af_rec_mix(sb);  
                out_emit(sb);  
                //out_emit(sat16(y));      
                continue;      
            }      
            if (tx_on) {        
                // TX-тракт: FIR -> компрессор -> клиппер -> постфильтр.        
                // RX-цепочка (FIR/CW/NT/NR/AGC) в TX не работает.        
                int32_t tx_in = (af_rec_active == 2) ? af_rec_src(y) : y;      
                if (af_rec_active == 1) af_rec_push(sat16(tx_in));  // запись: микрофон ДО TX-обработки      
                int16_t t;  
                if (cwtx_busy()) {  
                    // CW-посылка: тон с колокольной огибающей В ОБХОД  
                    // af_tx_process — компрессор/клиппер/гейт речевого  
                    // тракта съели бы фронты и добавили гармоники.  
                    // af_tx_process не вызывается — его FIR/EMA простаивают,  
                    // при возврате к речи продолжат без щелчка.  
                    t = cwtx_sample();  
                } else {  
                    t = af_tx_process(tx_in);            // play: запись вместо микрофона      
                }  
                out_emit(t);      
                continue;        
            }        
            y = af_fir_push(y);  
            {   // S-meter: EMA |y| Q16, ~10 мс — уровень в полосе до АРУ  
                int32_t ay = (y < 0) ? -y : y;  
                rx_in_lvl_q16 += (((int64_t)ay << 16) - rx_in_lvl_q16) >> 6;  
                if (ay > rx_in_peak) rx_in_peak = ay;   // пик за интервал чтения  
            }  
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
                if (cwdec_on) cwdec_push((int16_t)(y > 32767 ? 32767 : (y < -32768 ? -32768 : y)));
                if (rtty_on && !bypass_on && !tx_on) rttydec_push(x);  
            }    
            if (notch_on) y = notch_lms_push(y);    
            if (nr_on)    y = (nr_mode == 1) ? af_snr_push(y) : nr_lms_push(y); 
            int16_t s = agc_limit(y);  
            // Гребёнка — после АРУ (контур не «отменяет» нотчи), но ДО пост-FIR:  
            // предельный цикл Q15-нотчей звенит на частотах 3/4/5 кГц вне полосы —  
            // постфильтр этот звон отрезает. Нотчи 1/2 кГц внутри полосы работают.  
            if (rx_notch_on) s = (int16_t)rx_notch_push(s);  
            s = sat16(af_fir_push2(s));   // пост-FIR: срез внеполосного мусора после АРУ/NR  
            if (rx_pres_on)  s = sat16(rx_pres_push(s));     // presence: sat16, не cast —    
                                                           // подъём до +6 дБ может превысить int16    
            // Хвост RX: shelf + exciter стоят ПОСЛЕ лимитера — их подъём  
            // не контролируется контуром B. Вместо жёсткого sat16 на каждом  
            // каскаде — один мягкий клиппер в конце: пики плавно прижимаются  
            // к ~−3.5 dBFS, clip_count не растёт, щелчков нет.  
            if (rx_shelf_db_x10 || (rx_exc_on && !cw_on)) {  
                int32_t s32 = s;  
                if (rx_shelf_db_x10)       s32 = rx_shelf_push(s32); // low-shelf: «бас»  
                if (rx_exc_on && !cw_on)   s32 = rx_exc_push(s32);   // эксайтер: только речь  
                s = rx_soft_clip(s32);  
            }  
            if (af_rec_active == 1) af_rec_push(s);    // запись: чистый тракт, до голоса      
            s = af_voice_mix(s);                       // голосовые сообщения          
            if (af_rec_active == 2) s = af_rec_mix(s); // подмес воспроизведения слота
            
            if (ramp_on) {    
                // Пила: +100 на отсчёт при 15.625 ksps → период ~21 мс,    
                // амплитуда полная. Скачок ≠ +100 (mod 65536) = потеря/дубль.    
                s = ramp_val;    
                ramp_val += 100;    
            }    
            out_emit(sat16(s));  
        }  // конец for по отсчётам блока  
        if (rx_pres_on && !tx_on && !bypass_on) rx_pres_block_update();  

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
// empty_reads — пустые опросы очереди (consumer быстрее производителя,  
// штатная ситуация). НЕ путать с реальными underrun DMA — те считают  
// af_pwm_underruns()/af_usb_* на стороне потребителя.  
uint32_t af_dsp_out_empty_reads_usb(void) { return out_usb.empty_reads; }      
uint32_t af_dsp_out_empty_reads_pwm(void) { return out_pwm.empty_reads; }        
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