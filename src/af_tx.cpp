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
// Весь горячий путь — целочисленный. int64-деления — только      
// per-block (раз в 64 отсчёта): gain АРУ, компрессора, k_dyn.      
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
#define TX_FIR_N       127    // пред-фильтр: крутая речевая полоса    
#define TX_POST_FIR_N  63     // постфильтр: срез гармоник клиппинга    
    
static volatile uint32_t tx_lo_hz = AF_TX_LO_HZ_DEFAULT;    
static volatile uint32_t tx_hi_hz = AF_TX_HI_HZ_DEFAULT;    
    
static int16_t tx_fir_taps[TX_FIR_N];      
static int16_t tx_post_taps[TX_POST_FIR_N];      

static void tx_fir_design_n(int16_t* taps, int n) {      
    const int   M     = (n - 1) / 2;      
    const float fc_hi = (float)tx_hi_hz / AF_FS_OUT_HZ;      
    const float fc_lo = (float)tx_lo_hz / AF_FS_OUT_HZ;      
    float h[TX_FIR_N];      
    float sum = 0.0f;      
    for (int k = 0; k < n; k++) {      
        const int m = k - M;      
        float s_hi = (m == 0) ? (2.0f * fc_hi)      
                              : sinf(2.0f * PI * fc_hi * m) / (PI * m);      
        float s_lo = (m == 0) ? (2.0f * fc_lo)      
                              : sinf(2.0f * PI * fc_lo * m) / (PI * m);      
        const float a = 2.0f * PI * k / (n - 1);      
        float w = 0.35875f - 0.48829f * cosf(a)      
                         + 0.14128f * cosf(2.0f * a)      
                         - 0.01168f * cosf(3.0f * a);      
        h[k] = (s_hi - s_lo) * w;      
        sum += fabsf(h[k]);      
    }      
    const float norm = (sum > 0.0f) ? (2.0f / sum) : 1.0f;      
    for (int k = 0; k < n; k++)      
        taps[k] = (int16_t)lroundf(h[k] * norm * 32768.0f);      
}      

typedef struct {      
    int32_t buf[TX_FIR_N];      
    int     idx;      
    const int16_t* taps;      
    int     n;      
} tx_fir_t;      
    
static tx_fir_t tx_pre, tx_post;      
  
// --- RTTY TX: отдельный полосовой FIR 700..1400 Гц Гц (как RX-band при rtty on) ---  
// Свой экземпляр: речевой tx_pre трогать нельзя — он настроен под txband.  
#define TX_RTTY_FIR_N    127     // скаты ~±100 Гц, подавление ~-90 дБ 
#define TX_RTTY_LO_HZ    700u    // нижний срез фильтра
#define TX_RTTY_HI_HZ    1400u   // верхний срез фильтра
static int16_t tx_rtty_taps[TX_RTTY_FIR_N];      
static tx_fir_t tx_rtty_fir1, tx_rtty_fir2;   // каскад: два одинаковых FIR подряд      
  
// --- Двухтональный генератор: свой FIR 400..3200 Гц ---  
// Классический IMD-тест 700/1900: верхний тон и продукт IMD3 на 3100 Гц  
// должны пройти — речевой 700..1400 их срезал бы. Свой каскад 2x127.  
#define TX_TT_FIR_N    127  
#define TX_TT_LO_HZ    400u  
#define TX_TT_HI_HZ    3200u  
static int16_t tx_tt_taps[TX_TT_FIR_N];  
static tx_fir_t tx_tt_fir1, tx_tt_fir2;    
      
static void tx_fir_design(void) {        
    tx_fir_design_n(tx_fir_taps,  TX_FIR_N);       // пред-фильтр, 127      
    tx_fir_design_n(tx_post_taps, TX_POST_FIR_N);  // постфильтр, 63      
}        
  
// Дизайн RTTY-фильтра: та же sinc+Блэкман-Харрис, но полоса своя —  
// tx_fir_design_n читает tx_lo/hi_hz, поэтому подменяем на время вызова.  
static void tx_rtty_fir_design(void) {  
    const uint32_t save_lo = tx_lo_hz, save_hi = tx_hi_hz;  
    tx_lo_hz = TX_RTTY_LO_HZ;  
    tx_hi_hz = TX_RTTY_HI_HZ;  
    tx_fir_design_n(tx_rtty_taps, TX_RTTY_FIR_N);  
    tx_lo_hz = save_lo;  
    tx_hi_hz = save_hi;  
}

// Дизайн TT-фильтра: та же sinc+Блэкман-Харрис, полоса 400..3200.  
static void tx_tt_fir_design(void) {  
    const uint32_t save_lo = tx_lo_hz, save_hi = tx_hi_hz;  
    tx_lo_hz = TX_TT_LO_HZ;  
    tx_hi_hz = TX_TT_HI_HZ;  
    tx_fir_design_n(tx_tt_taps, TX_TT_FIR_N);  
    tx_lo_hz = save_lo;  
    tx_hi_hz = save_hi;  
}  
    
static inline int32_t __not_in_flash_func(tx_fir_push)(tx_fir_t* f, int32_t x) {      
    const int n = f->n;      
    f->buf[f->idx] = x;      
    if (++f->idx >= n) f->idx = 0;      
      
    int64_t acc = 0;      
    int lo = f->idx;      
    int hi = f->idx + n - 1;      
    if (hi >= n) hi -= n;      
    for (int k = 0; k < n / 2; k++) {      
        acc += (int32_t)f->taps[k] * (f->buf[lo] + f->buf[hi]);      
        if (++lo >= n) lo = 0;      
        if (--hi < 0)  hi = n - 1;      
    }      
    acc += (int32_t)f->taps[n / 2] * f->buf[lo];      
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
static volatile int32_t  tx_inv_3t2_q31 =                         // 1/(3t^2), Q31    
    (int32_t)(((int64_t)1 << 31) / (3LL * AF_TX_CLIP_DEFAULT * AF_TX_CLIP_DEFAULT));    
static uint32_t tx_agc_target = AF_TX_AGC_TARGET;  // целевой уровень АРУ    
static uint32_t tx_gate_thr   = AF_TX_GATE_THR;    // порог гейта (0 = off)      
static volatile int32_t tx_gate_floor = AF_TX_GATE_FLOOR_Q16; // нижняя ступень, Q16    
static volatile uint32_t tx_agc_max_x10   = AF_TX_AGC_MAX_X10;    // потолок АРУ, x10    
static volatile uint32_t tx_gate_hold_ms  = AF_TX_GATE_HOLD_MS;   // hangover, мс    
static volatile uint32_t tx_gate_hyst_x10 = AF_TX_GATE_HYST_X10;  // гистерезис, x10    
static uint32_t agc_env       = 0;                 // EMA огибающей АРУ, Q16
static int32_t  tx_agc_gain_q15 = 32768;           // последний gain АРУ (debug)  
static uint32_t gate_gain     = 0;                 // gain гейта, Q16 (0..65536)  
static int64_t tx_env_q16 = (int64_t)256 << 16;   // EMA огибающей, Q16  
static int32_t tx_gain_1band_q15 = 32768;         // кеш gain 1-полосного, Q15  
static int32_t tx_gain_lo_q15    = 32768;         // кеш gain НЧ-полосы, Q15  
static int32_t tx_gain_hi_q15    = 32768;         // кеш gain ВЧ-полосы, Q15  
static int32_t tx_agc_gain_blk   = 32768;         // кеш gain АРУ, Q15 (per-block)    
static uint8_t agc_blk_cnt       = 0;             // счётчик пересчёта gain АРУ    
static uint8_t comp_blk_cnt      = 0;             // счётчик пересчёта gain компрессора    
static volatile uint8_t  tx_ap_on    = AF_TX_AP_ON_DEFAULT;  
static volatile uint8_t  tx_softclip = AF_TX_SOFTCLIP_ON_DEFAULT;  
static int32_t ap_x1 = 0, ap_y1 = 0;   // состояние all-pass  
static volatile uint8_t  tx_2band_on = AF_TX_2BAND_ON_DEFAULT;  
static volatile uint8_t  tx_notch_on = AF_TX_NOTCH_ON_DEFAULT;    
static volatile uint32_t tx_split_hz    = AF_TX_SPLIT_HZ;      // частота кроссовера    
static volatile int32_t  tx_split_k_q15 = AF_TX_SPLIT_K_Q15;   // k старого split, Q15    
static volatile uint32_t tx_notch1_hz   = AF_TX_NOTCH1_HZ;     // частота 1-го нотча    
static volatile uint32_t tx_notch2_hz   = AF_TX_NOTCH2_HZ;     // частота 2-го нотча    
// Коэффициенты нотчей Q15: b = {1, -2cos(w), 1}, a = {1, -2r cos(w), r^2}    
static int32_t n1_b1 = 0, n1_a1 = 0, n1_a2 = 0;    
static int32_t n2_b1 = 0, n2_a1 = 0, n2_a2 = 0;    
static int32_t n1_x1 = 0, n1_x2 = 0, n1_y1 = 0, n1_y2 = 0;  
static int32_t n2_x1 = 0, n2_x2 = 0, n2_y1 = 0, n2_y2 = 0;  
static uint16_t tx_gate_hold = 0;   // счётчик hangover, в отсчётах @AF_FS_OUT_HZ

// --- Этап 2: Dynamic Presence EQ (SNR-адаптивный) ---  
static volatile uint8_t  tx_pres_on    = AF_TX_PRES_ON_DEFAULT;    
static volatile int32_t  tx_presmax_q15 = AF_TX_PRES_MAX_Q15;  // потолок подъёма, Q15    
static volatile uint32_t tx_pres_freq_hz = AF_TX_PRES_FREQ_HZ; // центр пика, Гц    
static volatile uint32_t tx_pres_q_x100  = AF_TX_PRES_Q_X100;  // добротность x100    
  
// Отвод env_hi_pre: HP ~1.5 кГц (биквад) + EMA — после АРУ, ДО гейта.  
// Источник для k_dyn и noise floor (env_hi компрессора в паузах = 0).  
static int32_t hp_x1 = 0, hp_x2 = 0, hp_y1 = 0, hp_y2 = 0;   // HP-биквад  
static int32_t hp_b0 = 0, hp_b1 = 0, hp_b2 = 0, hp_a1 = 0, hp_a2 = 0;  
static int64_t env_hi_pre_q16 = (int64_t)256 << 16;          // EMA |hp|, Q16  
static int64_t pres_nf_q16    = (int64_t)256 << 16;          // noise floor, Q16    
static int32_t k_dyn_q15      = 0;                           // сглаженный k_dyn, Q15    
static int32_t k_dyn_tgt_q15  = 0;                           // цель k_dyn (per-block)    
static uint8_t pres_blk_cnt   = 0;                           // счётчик до пересчёта k    
static uint8_t pres_decim = 0;   // дециматор sidechain ÷2    
static uint32_t tx_presmax_db_x10 = AF_TX_PRES_MAX_DB_X10;   // из конфига
static uint32_t tx_duckmax_db_x10 = AF_TX_DUCK_MAX_DB_X10;  
  
// Peaking-биквад 2–2.7 кГц (после компрессора, до клиппера)  
static int32_t pe_x1 = 0, pe_x2 = 0, pe_y1 = 0, pe_y2 = 0;  
static int32_t pe_b0 = 0, pe_b1 = 0, pe_b2 = 0, pe_a1 = 0, pe_a2 = 0;  

static int32_t split_lp = 0;                    // состояние ФНЧ кроссовера (1-й порядок)    
static volatile uint8_t tx_lr2_on = AF_TX_LR2_ON_DEFAULT;    
// LR2-кроссовер: LP и HP биквады Баттерворта Q=0.7071 на AF_TX_SPLIT_HZ.    
// Коэффициенты Q15, считаются в af_tx_init (RBJ lowpass/highpass).    
static int32_t xo_lp_b0 = 0, xo_lp_b1 = 0, xo_lp_b2 = 0, xo_lp_a1 = 0, xo_lp_a2 = 0;    
static int32_t xo_hp_b0 = 0, xo_hp_b1 = 0, xo_hp_b2 = 0, xo_hp_a1 = 0, xo_hp_a2 = 0;    
static int32_t xo_lp_x1 = 0, xo_lp_x2 = 0, xo_lp_y1 = 0, xo_lp_y2 = 0;    
static int32_t xo_hp_x1 = 0, xo_hp_x2 = 0, xo_hp_y1 = 0, xo_hp_y2 = 0;    
static int64_t tx_env_lo_q16 = (int64_t)256 << 16;  // EMA огибающей НЧ-полосы  
static int64_t tx_env_hi_q16 = (int64_t)256 << 16;  // EMA огибающей ВЧ-полосы  

// Двухтональный генератор: прогон через каскадный 400..3200 + DC-blocker.  
static int32_t tx_tt_dc_x1 = 0, tx_tt_dc_y1 = 0;  

// --- Этап 5: HF ducking ---    
static volatile uint8_t  tx_duck_on     = AF_TX_DUCK_ON_DEFAULT;    
static volatile int32_t  tx_duckmax_q15 = AF_TX_DUCK_MAX_Q15;  // потолок, Q15    
static int32_t k_duck_q15     = 0;      // сглаженный k_duck, Q15    
static int32_t k_duck_tgt_q15 = 0;      // цель k_duck (per-block)    
static uint8_t duck_blk_cnt   = 0;      // счётчик пересчёта    
  
// --- Этап 6: ВЧ-эксайтер (динамический усилитель ВЧ-полосы) ---        
// Подмес env(|hi|)*hi*mix — AM по огибающей, НЕ гармонический        
// эксайтер: новых частотных компонент в установившемся режиме нет.        
// Субъективно подчёркивает согласные; для настоящих гармоник нужна        
// нелинейность (hi*|hi| и т.п.) — пока не вводим.        
static volatile uint8_t  tx_exc_on   = AF_TX_EXC_ON_DEFAULT;   // дефолт on          
static volatile int32_t  tx_exc_mix  = AF_TX_EXC_MIX_Q15;      // подмес, Q15        
static int64_t exc_env_q16 = (int64_t)256 << 16;               // EMA |hi|, Q16          
static uint8_t exc_decim   = 0;                                // дециматор EMA ÷2             
  
// --- Этап 7: k_intelligibility (автобаланс НЧ/ВЧ) ---    
static volatile uint8_t tx_intel_on = AF_TX_INTEL_ON_DEFAULT;      
static volatile int32_t tx_intel_tgt_q15   = AF_TX_INTEL_TGT_Q15;    // цель ratio, Q15    
static volatile int32_t tx_intel_cut_q15   = AF_TX_INTEL_CUT_Q15;    // потолок ослабления    
static volatile int32_t tx_intel_boost_q15 = AF_TX_INTEL_BOOST_Q15;  // потолок подъёма    
static int32_t g_intel_q15    = 32768;   // сглаженный gain ВЧ-полосы, Q15      
static int32_t g_intel_tgt_q15 = 32768;  // цель gain (per-block)      
static uint8_t intel_blk_cnt  = 0;       // счётчик пересчёта           
  
// --- High-shelf полка (вариант A): статический подъём ВЧ ---  
static uint32_t tx_shelf_db_x10 = AF_TX_SHELF_DB_X10_DEFAULT;  // 0 = off    
static volatile uint32_t tx_shelf_freq_hz = AF_TX_SHELF_FREQ_HZ;  // частота полки    
static volatile uint32_t tx_shelf_q_x100  = AF_TX_SHELF_Q_X100;   // добротность x100    
static int32_t sh_x1 = 0, sh_x2 = 0, sh_y1 = 0, sh_y2 = 0;    
static int32_t sh_b0 = 0, sh_b1 = 0, sh_b2 = 0, sh_a1 = 0, sh_a2 = 0;       
  
// Пороги компрессора — runtime (дефайны = значения по умолчанию).    
static volatile int32_t tx_comp_thr    = 8192;               // 1-полосный, ~25% шкалы    
static volatile int32_t tx_comp_thr_lo = AF_TX_COMP_THR_LO;  // НЧ-полоса    
static volatile int32_t tx_comp_thr_hi = AF_TX_COMP_THR_HI;  // ВЧ-полоса    

// All-pass 1-го порядка (phase rotator): y = a*(x - y1) + x1.  
// Амплитуда не меняется, фаза крутится — пики речи симметрируются,  
// клипперу достаётся более «квадратный» сигнал.  
static inline int32_t __not_in_flash_func(tx_allpass)(int32_t x) {  
    int32_t y = (int32_t)(((int64_t)AF_TX_AP_COEFF_Q15 * (x - ap_y1)) >> 15) + ap_x1;  
    ap_x1 = x;  
    ap_y1 = y;  
    return y;  
}  

// Компрессия одной полосы: своя EMA-огибающая, свой порог.  
// gain пересчитывается раз в 64 отсчёта (per-block): огибающая  
// медленная (EMA ~10/300 мс), per-sample деление thr/env не нужно.  
// Кеш gain хранится снаружи — у каждой полосы свой.  
static inline int32_t __not_in_flash_func(tx_comp_band)(int32_t x,  
                                                        int64_t* env_q16,  
                                                        int32_t* gain_q15,  
                                                        int32_t thr) {  
    int64_t ax  = (x < 0) ? -x : x;    
    int64_t tgt = ax << 16;    
    int64_t da  = tgt - *env_q16;    
    // Этап 3: program-dependent release. Спад после громкой фразы    
    // (env > txgate) — быстрый: gain возвращается к 1 до того, как    
    // гейт закроется, шум паузы не «подтягивается». На тихой речи    
    // (env <= txgate) — медленный, без «дыхания».    
    int64_t c;    
    if (da > 0) {    
        c = (int64_t)AF_TX_ENV_ATT_Q24;    
    } else {    
        c = (*env_q16 > ((int64_t)tx_gate_thr << 16))    
                ? (int64_t)AF_TX_ENV_DEC_FAST_Q24    
                : (int64_t)AF_TX_ENV_DEC_Q24;    
    }    
    *env_q16 += (da * c) >> 24;      
    if (*env_q16 < ((int64_t)1 << 16)) *env_q16 = (int64_t)1 << 16;    
    // gain пересчитывается в tx_compress раз в 64 ОТСЧЁТА для обеих    
    // полос одновременно — раньше счётчик жил здесь и в 2-полосном    
    // режиме срабатывал только на вызове hi: lo оставался без    
    // компрессии (gain=1 навсегда), hi обновлялся вдвое чаще.    
    return (int32_t)(((int64_t)x * *gain_q15) >> 15);    
}    
  
// Пересчёт gain одной полосы по её огибающей — вызывается из    
// tx_compress раз в 64 отсчёта.    
static inline void __not_in_flash_func(tx_comp_gain_update)(int64_t env_q16,  
                                       int32_t* gain_q15, int32_t thr) {  
    int32_t env = (int32_t)(env_q16 >> 16);    
    if (env > thr) {    
        int32_t inv = (int32_t)(((int64_t)thr << 15) / env);   // thr/env Q15    
        *gain_q15   = 32768 - (((32768 - inv) * tx_depth_q15) >> 15);    
    } else {    
        *gain_q15 = 32768;    
    }    
}    

// Биквад Direct Form I, коэффициенты Q15 — для LR2-кроссовера.    
// int64-аккумулятор: a1 ~1.9*Q15, сумма 5 произведений > int32.    
static inline int32_t __not_in_flash_func(tx_biquad)(int32_t x,      
        int32_t b0, int32_t b1, int32_t b2, int32_t a1, int32_t a2,      
        int32_t* x1, int32_t* x2, int32_t* y1, int32_t* y2) {      
    // Каждое произведение — int32 (1 такт на M0+): |coef| < 2 и      
    // |x|,|y| <= 32767 -> |произведение| < 2.1e9, в int32 влезает      
    // впритык. Сумма 5 членов может превысить int32 -> накопление      
    // в int64 (сложение 64-бит ~2 такта, дёшево).      
    int64_t acc = (int32_t)(b0 * x) + (int32_t)(b1 * *x1) + (int32_t)(b2 * *x2)      
                - (int32_t)(a1 * *y1) - (int32_t)(a2 * *y2);      
    int32_t y = (int32_t)(acc >> 15);      
    *x2 = *x1; *x1 = x;    
    *y2 = *y1; *y1 = y;    
    return y;    
}    

// 2-полосная компрессия: комплементарный split на ~900 Гц.  
// lo = ФНЧ(x), hi = x - lo — сумма тождественно равна x (фазово согласованы).  
// НЧ (гласные) и ВЧ (согласные) сжимаются раздельно -> нет pumping.  
static inline int32_t __not_in_flash_func(tx_compress)(int32_t x, int32_t* hi_out) {      
    if (!tx_2band_on) {        
        *hi_out = 0;                    // 1-полосный режим: эксайтер молчит    
        int32_t r = tx_comp_band(x, &tx_env_q16, &tx_gain_1band_q15, tx_comp_thr);      
        if (++comp_blk_cnt >= 64) {     // тот же счётчик, раз в 64 отсчёта      
            comp_blk_cnt = 0;      
            tx_comp_gain_update(tx_env_q16, &tx_gain_1band_q15, tx_comp_thr);      
        }    
        return r;    
    }        
      
    int32_t lo, hi;      
    if (tx_lr2_on) {    
        // Этап 4: кроссовер Баттерворта 2-го порядка на AF_TX_SPLIT_HZ —  
        // RBJ LP+HP, Q=0.7071; HP инвертирован -> сумма all-pass, -3 дБ на  
        // разделе. Не LR2: у LR2 Q=0.5 и -6 дБ на разделе.  
        lo = tx_biquad(x, xo_lp_b0, xo_lp_b1, xo_lp_b2, xo_lp_a1, xo_lp_a2,    
                       &xo_lp_x1, &xo_lp_x2, &xo_lp_y1, &xo_lp_y2);    
        hi = tx_biquad(x, xo_hp_b0, xo_hp_b1, xo_hp_b2, xo_hp_a1, xo_hp_a2,    
                       &xo_hp_x1, &xo_hp_x2, &xo_hp_y1, &xo_hp_y2);    
    } else {    
        // Старый комплементарный split: ФНЧ 1-го порядка, hi = x - lo    
        split_lp += (int32_t)(((int64_t)(x - split_lp) * tx_split_k_q15) >> 15);      
        lo = split_lp;    
        hi = x - lo;    
    }    
    
    lo = tx_comp_band(lo, &tx_env_lo_q16, &tx_gain_lo_q15, tx_comp_thr_lo);          
    hi = tx_comp_band(hi, &tx_env_hi_q16, &tx_gain_hi_q15, tx_comp_thr_hi);          
    
    // Per-block пересчёт gain ОБЕИХ полос раз в 64 отсчёта (~4 мс).      
    if (++comp_blk_cnt >= 64) {      
        comp_blk_cnt = 0;      
        tx_comp_gain_update(tx_env_lo_q16, &tx_gain_lo_q15, tx_comp_thr_lo);      
        tx_comp_gain_update(tx_env_hi_q16, &tx_gain_hi_q15, tx_comp_thr_hi);      
    }       
      
    // Этап 5: HF ducking — НЧ приглушается, когда ВЧ доминирует.    
    // k_duck = clamp((env_hi - env_lo/2) / (env_lo + eps), 0..1):    
    // env_hi > env_lo/2 -> согласные сильнее половины гласных ->    
    // НЧ давится до duck_max. Деление per-block (~4 мс).    
    if (tx_duck_on) {    
        if (++duck_blk_cnt >= 64) {      
            duck_blk_cnt = 0;      
            int64_t e_lo = tx_env_lo_q16 >> 16;      
            int64_t e_hi = tx_env_hi_q16 >> 16;      
            int64_t num  = e_hi - (e_lo >> 1);      
            int32_t k = 0;      
            // duck только при живой речи: env_lo выше порога гейта.      
            // Иначе на шуме (env~1) num>0 и kd залипает ~500.      
            if (num > 0 && e_lo > (int64_t)tx_gate_thr) {      
                int64_t den = e_lo + 64;              // eps от деления на ~0      
                k = (int32_t)((num << 15) / den);      
                if (k > 32768) k = 32768;      
            }      
            k_duck_tgt_q15 = k;      
        }      
        // EMA-сглаживание: атака ~5 мс, спад ~80 мс — без щелчков    
        int32_t c = (k_duck_tgt_q15 > k_duck_q15) ? AF_TX_DUCK_ATT_Q24    
                                                : AF_TX_DUCK_REL_Q24;    
        k_duck_q15 += (int32_t)(((int64_t)(k_duck_tgt_q15 - k_duck_q15) * c) >> 24);    
        // lo *= (1 - k_duck*duck_max): gain_duck = 32768 - k_duck*duck_max>>15    
        int32_t g_duck = 32768 - ((k_duck_q15 * tx_duckmax_q15) >> 15);    
        lo = (int32_t)(((int64_t)lo * g_duck) >> 15);    
    } else {      
        k_duck_q15 = 0;      
        k_duck_tgt_q15 = 0;      
    }      
  
    // Этап 7: k_intelligibility — автобаланс ВЧ-полосы.  
    // ratio = env_hi/env_lo; цель AF_TX_INTEL_TGT_Q15 (~0.47).  
    // ratio < tgt -> g_hi > 1 (подъём согласных), ratio > tgt -> g_hi < 1.  
    // g = clamp(tgt/ratio, 1-cut, 1+boost). Деление per-block.  
    // Только при живой речи (env_lo > gate) — на шуме g=1.  
    if (tx_intel_on) {  
        if (++intel_blk_cnt >= 64) {  
            intel_blk_cnt = 0;  
            int64_t e_lo = tx_env_lo_q16 >> 16;  
            int64_t e_hi = tx_env_hi_q16 >> 16;  
            int32_t g = 32768;  
            if (e_lo > (int64_t)tx_gate_thr && e_hi > 0) {  
                // g = tgt * e_lo / e_hi  (ratio = e_hi/e_lo -> g = tgt/ratio)  
                int64_t gg = ((int64_t)tx_intel_tgt_q15 * e_lo) / e_hi;    
                int64_t gmin = 32768 - tx_intel_cut_q15;    // 0.5    
                int64_t gmax = 32768 + tx_intel_boost_q15;  // 2.0    
                if (gg < gmin) gg = gmin;  
                if (gg > gmax) gg = gmax;  
                g = (int32_t)gg;  
            }  
            g_intel_tgt_q15 = g;  
        }  
        g_intel_q15 += (int32_t)(((int64_t)(g_intel_tgt_q15 - g_intel_q15)  
                                 * AF_TX_INTEL_SMOOTH_Q24) >> 24);  
        hi = (int32_t)(((int64_t)hi * g_intel_q15) >> 15);  
    } else {  
        g_intel_q15 = 32768;  
        g_intel_tgt_q15 = 32768;  
    }  
        
    if (tx_lr2_on) {        
        // Раздельный клиппинг полос до суммирования: НЧ-пики гласных    
        // не модулируют согласные. Порог общий tx_clip_thr, без makeup —    
        // makeup применяется в основном клиппере дальше по цепочке.    
        int32_t t = (int32_t)tx_clip_thr;    
        if (lo >  t) lo =  t;    
        if (lo < -t) lo = -t;    
        if (hi >  t) hi =  t;    
        if (hi < -t) hi = -t;    
    }    
    *hi_out = hi;                       // ВЧ-полоса для эксайтера (этап 6)      
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
    // y = x - x^3/(3t^2): деление заменено умножением на кешированный    
    // inv_3t2 (Q31) — int64-mul вместо int64-div, ~35 тактов -> ~5.    
    int64_t x3 = (int64_t)x * x * x;                        // <= 2^45    
    int32_t y = x - (int32_t)((x3 * tx_inv_3t2_q31) >> 31);    
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
// int64-аккумулятор: сумма 5 произведений Q15*int16 может кратковременно      
// превысить int32 на пиках (b0~0.8*Q15, но a1~1.9*Q15 для HP 1.5к/15.4к)      
// -> wrap дал бы выброс в env_hi_pre и ложный скачок k_dyn.      
// Вызывается на 7.7 ksps (децимация /2) — цена ~0.5% L1.      
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
    // Огибающая 50 мс не нуждается в 15.4 ksps — считаем на 7.7 ksps.    
    // Экономит половину стоимости HP-биквада + EMA.    
    if (++pres_decim & 1) return;    
    int32_t hp = tx_hp_push(x_agc);    
    int32_t ax = (hp < 0) ? -hp : hp;    
    
    // EMA огибающей ВЧ — int32: da <= 2^31, c <= 2^24,    
    // произведение <= 2^55 — нужен int64 только для сдвига,    
    // но da*c в int64 один раз дешевле, чем весь блок в int64.    
    // Компромисс: da в int32, умножение через int64 один раз.    
    int32_t tgt = ax << 16;    
    int32_t da  = tgt - (int32_t)env_hi_pre_q16;   // env < 2^31, усечение безопасно  
    env_hi_pre_q16 += ((int64_t)da * (da > 0 ? (int64_t)AF_TX_ENV_ATT_Q24    
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
    // Деление — раз в 128 отсчётов (~8.3 мс): функция децимирована      
    // ÷2 (pres_decim), 64 вызова = 128 входных отсчётов. Огибающая      
    // медленная, per-sample пересчёт не нужен и стоил +17% L1.      
    // ВНИМАНИЕ: EMA-константы ATT/DEC и K_ATT/K_REL применяются на      
    // 7.7 ksps — фактические постоянные времени вдвое больше номинала.      
    if (++pres_blk_cnt >= 64) {      
        pres_blk_cnt = 0;    
        int64_t nf  = pres_nf_q16 >> 16;    
        int64_t env = env_hi_pre_q16 >> 16;    
        int64_t num = env - 2 * nf;    
        int32_t k = 0;    
        if (num > 0) {    
            k = (int32_t)((num << 15) / (4 * nf));    
            if (k > 32768) k = 32768;    
        }    
        k_dyn_tgt_q15 = k;    
    }    
    
    // Сглаживание k_dyn к цели: атака ~10 мс, спад ~100 мс —    
    // без щелчков на границах слогов. Только EMA, без деления.    
    int32_t c = (k_dyn_tgt_q15 > k_dyn_q15) ? AF_TX_PRES_K_ATT_Q24    
                                          : AF_TX_PRES_K_REL_Q24;    
    k_dyn_q15 += (int32_t)(((int64_t)(k_dyn_tgt_q15 - k_dyn_q15) * c) >> 24);    
}    
  
// Peaking-биквад с переменным подъёмом: gain = 1 + k_dyn*(presmax-1).  
// Реализовано как dry + k_dyn*(wet - dry): при k_dyn=0 — прозрачный  
// байпас, при k_dyn=1 — полный подъём presmax. Коэффициенты биквада  
// посчитаны на максимальный подъём; промежуточные k — линейный подмес.  
static inline int32_t __not_in_flash_func(tx_presence)(int32_t x) {    
    if (!tx_pres_on) return x;    
    // Биквад работает всегда — иначе при возврате k_dyn>0 состояния    
    // pe_x1/pe_y1 устарели и дают щелчок. При k_dyn=0 выход = x    
    // (dry-подмес), цена — один биквад ~0.3% L1.    
    // int64-аккумулятор: при presmax=+12 дБ b0~1.4*Q15, произведение      
    // ~1.4*2^30, сумма 5 членов может превысить int32 -> wrap = треск.      
    // 5 int64-mul ~175 тактов/отсчёт ~+1% L1 — цена за отсутствие      
    // выбросов на пиках. Сатурация wet остаётся как страховка.      
    int64_t acc = (int64_t)pe_b0 * x + (int64_t)pe_b1 * pe_x1      
                + (int64_t)pe_b2 * pe_x2      
                - (int64_t)pe_a1 * pe_y1 - (int64_t)pe_a2 * pe_y2;      
    int32_t wet = (int32_t)(acc >> 15);      
    if (wet >  32767) wet =  32767;    
    if (wet < -32768) wet = -32768;    
    pe_x2 = pe_x1; pe_x1 = x;    
    pe_y2 = pe_y1; pe_y1 = wet;    
    // y = x + k_dyn*(wet - x): (wet-x) <= 2^16, k_dyn <= 2^15,    
    // произведение <= 2^31 — int32 безопасен.    
    return x + (((wet - x) * k_dyn_q15) >> 15);     
} 

// High-shelf биквад (вариант A): статический подъём ВЧ выше  
// AF_TX_SHELF_FREQ_HZ. Компенсирует «Hi-Fi» плоскость тракта —  
// для разборчивости SSB-речи нужен подъём +3..+6 дБ на согласных.  
// Коэффициенты RBJ high-shelf, пересчёт в af_tx_set_shelf.  
// При shelf=0 — байпас, ноль нагрузки.  
static inline int32_t __not_in_flash_func(tx_shelf)(int32_t x) {    
    if (tx_shelf_db_x10 == 0) return x;    
    // tx_biquad: int32-произведения + int64-накопление.    
    // |b0|~1.3*Q15, |a1|~1.5*Q15 при +4 дБ -> произведения < 2^31.    
    int32_t y = tx_biquad(x, sh_b0, sh_b1, sh_b2, sh_a1, sh_a2,    
                          &sh_x1, &sh_x2, &sh_y1, &sh_y2);    
    if (y >  32767) y =  32767;    
    if (y < -32768) y = -32768;    
    return y;    
}    

// Этап 6: ВЧ-эксайтер. Берёт ВЧ-полосу (hi из кроссовера — уже      
// посчитана в tx_compress, передаём сюда), выпрямляет |hi| ->      
// EMA-огибающая -> подмес env*|hi|*sign(hi)*mix = env*hi*mix.      
// Гармоники в фазе с исходной ВЧ — «воздух» согласным без чужого      
// сигнала. В паузах hi~0 -> exc~0, нагрузки нет.      
// ~3 int64-mul per-sample при включённом ~+0.5% L1.      
static inline int32_t __not_in_flash_func(tx_exciter)(int32_t x, int32_t hi) {        
    if (!tx_exc_on) return x;        
    // EMA огибающей ~10 мс не нуждается в 15.4 ksps — считаем на      
    // 7.7 ksps (децимация /2, как в tx_pres_sidechain). Подмес      
    // env*hi*mix остаётся per-sample — он дешёвый (2 int64-mul).      
    if (!(++exc_decim & 1)) {      
        int32_t a = (hi < 0) ? -hi : hi;        
        int64_t tgt = (int64_t)a << 16;        
        int64_t da  = tgt - exc_env_q16;        
        // Своя константа AF_TX_EXC_ENV_Q24 (~10 мс): раньше бралась      
        // ATT_Q24=400 — огибающая была в ~7 раз быстрее задуманной.      
        exc_env_q16 += (da * (int64_t)AF_TX_EXC_ENV_Q24) >> 24;        
    }      
    // exc = env(|hi|) * hi * mix: env Q16 -> >>16, mix Q15 -> >>15        
    int32_t env = (int32_t)(exc_env_q16 >> 16);        
    int32_t exc = (int32_t)(((int64_t)env * hi) >> 15);        
    return x + (int32_t)(((int64_t)exc * tx_exc_mix) >> 15);        
}        

// ---------------------------------------------------------------------------    
// Точка входа из af_dsp_poll    
// ---------------------------------------------------------------------------

// АРУ: gain = target/env, кламп 1..max. EMA огибающей — общая с  
// компрессором нет, своя (агрессивнее атака, медленнее спад).  
static int32_t __not_in_flash_func(tx_agc)(int32_t x) {  
    uint32_t a = (uint32_t)(x >= 0 ? x : -x);    
    uint32_t c = (a > (agc_env >> 8)) ? AF_TX_AGC_ATT_Q24 : AF_TX_AGC_REL_Q24;    
    // ВАЖНО: вычитание в int64 — иначе при agc_env > a<<8 беззнаковый    
    // андерфлоу даёт +4 млрд и огибающая залипает на UINT32_MAX    
    // (гейт permanently открыт, АРУ застревает на gain=1).    
    agc_env += (uint32_t)(((int64_t)((int64_t)(a << 8) - (int64_t)agc_env) * c) >> 24);      
  
    // gain АРУ — per-block: огибающая медленная, деление target/env  
    // раз в 64 отсчёта (~4 мс) неотличимо от per-sample на слух.  
    if (++agc_blk_cnt >= 64) {    
        agc_blk_cnt = 0;    
        uint32_t env = agc_env >> 8;  
        if (env < 64) env = 64;                        // защита от деления на ~0  
        int64_t g = ((int64_t)tx_agc_target << 15) / (int64_t)env; // Q15  
        int64_t gmax = (int64_t)tx_agc_max_x10 * 3277;             // x10 -> Q15    
        if (g > gmax) g = gmax;  
        if (g < 32768) g = 32768;                      // АРУ только усиливает/держит  
        // Гейт закрыт -> не разгонять фон: gain <= 1.  
        if (tx_gate_thr != 0 && env < tx_gate_thr && g > 32768) g = 32768;  
        tx_agc_gain_blk = (int32_t)g;  
        tx_agc_gain_q15 = (int32_t)g;                  // для stats (этап 0)  
    }  
    return (int32_t)(((int64_t)x * tx_agc_gain_blk) >> 15);  
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
static int32_t __not_in_flash_func(tx_gate)(int32_t x) {  
    if (tx_gate_thr == 0) return x;                  // гейт выключен  
    uint32_t env = agc_env >> 8;                     // та же огибающая  
    const uint32_t thr_close = (tx_gate_thr * tx_gate_hyst_x10) / 10;    
    const uint16_t hold_n = (uint16_t)((tx_gate_hold_ms * AF_FS_OUT_HZ) / 1000u);    
  
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

// DC-блокер: однополюсный ФВЧ ~20 Гц между клиппером и пост-FIR.  
// y[n] = x[n] - x[n-1] + R*y[n-1], R = exp(-2*pi*20/15464) ~ 0.9919  
// Q15: R = 32510. Страховка от DC после клиппера/эксайтера.  
static int32_t dc_x1 = 0, dc_y1 = 0;  
  
static inline int32_t __not_in_flash_func(tx_dc_block)(int32_t x) {    
    int32_t y = x - dc_x1 + (int32_t)(((int64_t)32510 * dc_y1) >> 15);    
    // Кламп состояния: без него при большом DC на входе dc_y1      
    // накапливается и переполняет int32 -> wrap = щелчок.      
    if (y >  32767) y =  32767;    
    if (y < -32768) y = -32768;    
    dc_x1 = x;    
    dc_y1 = y;    
    return y;    
}    

// RTTY/CW TX: прогон отсчёта через каскадный полосовой 700..1400.    
// Вызывается из af_dsp_poll вместо прямого rttytx_sample()/cwtx_sample().    
// DC-blocker на выходе: ключевой тон имеет ненулевое среднее (NCO    
// останавливается на произвольной фазе) — EMA вычитает DC, срез ~25 Гц.    
static int32_t tx_rtty_dc_x1 = 0, tx_rtty_dc_y1 = 0;    
    
int16_t __not_in_flash_func(af_tx_rtty_fir_push)(int32_t x) {        
    int32_t y = tx_fir_push(&tx_rtty_fir1, x);        
    y = tx_fir_push(&tx_rtty_fir2, y);        // каскад: скаты и подавление удваиваются        
    int32_t d = y - tx_rtty_dc_x1 + (int32_t)(((int64_t)32510 * tx_rtty_dc_y1) >> 15);    
    if (d >  32767) d =  32767;    
    if (d < -32768) d = -32768;    
    tx_rtty_dc_x1 = y;    
    tx_rtty_dc_y1 = d;    
    if (d >  32767) d =  32767;        
    if (d < -32768) d = -32768;        
    return (int16_t)d;        
}
  
int16_t __not_in_flash_func(af_tx_tt_fir_push)(int32_t x) {  
    int32_t y = tx_fir_push(&tx_tt_fir1, x);  
    y = tx_fir_push(&tx_tt_fir2, y);  
    int32_t d = y - tx_tt_dc_x1 + (int32_t)(((int64_t)32510 * tx_tt_dc_y1) >> 15);  
    if (d >  32767) d =  32767;  
    if (d < -32768) d = -32768;  
    tx_tt_dc_x1 = y;  
    tx_tt_dc_y1 = d;  
    return (int16_t)d;  
}  
  
int16_t __not_in_flash_func(af_tx_process)(int32_t y) {        
    y = tx_fir_push(&tx_pre, y);      // речевая полоса        
    if (tx_ap_on) y = tx_allpass(y);  // симметризатор пиков речи  
    y = tx_agc(y);                    // АРУ: выравнивание уровня к target      
    tx_pres_sidechain(y);             // env_hi_pre/nf/k_dyn — ДО гейта  
    y = tx_gate(y);                   // шумовой гейт: паузы -> тишина      
    int32_t hi_band = 0;      
    y = tx_compress(y, &hi_band);     // компрессия + отвод ВЧ-полосы      
    y = tx_presence(y);               // dyn-presence EQ (этап 2)      
    // вместо y = tx_shelf(y):  
    const int32_t sh = tx_shelf(y);  
    y += (int32_t)(((int64_t)(sh - y) * k_dyn_q15) >> 15);  // подмес полки по k_dyn  
    y = tx_exciter(y, hi_band);       // ВЧ-эксайтер (этап 6)      
    y = tx_softclip ? tx_soft_clip(y) // кубический клиппер (мягкий)        
                    : tx_clip(y);     // жёсткий клиппер      
    y = tx_dc_block(y);               // ФВЧ ~20 Гц: страховка от DC    
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

// Расчёт коэффициентов нотчей по tx_notch1/2_hz — вызывается из    
// af_tx_init и af_tx_set_notchf. w = 2*pi*f0/Fs, b1 = -2cos(w),    
// a1 = -2r*cos(w), a2 = r^2.    
static void tx_notch_design(void) {    
    const float r  = (float)AF_TX_NOTCH_R_Q15 / 32768.0f;    
    const float w1 = 2.0f * PI * (float)tx_notch1_hz / AF_FS_OUT_HZ;    
    const float w2 = 2.0f * PI * (float)tx_notch2_hz / AF_FS_OUT_HZ;    
    n1_b1 = (int32_t)lroundf(-2.0f * cosf(w1) * 32768.0f);    
    n1_a1 = (int32_t)lroundf(-2.0f * r * cosf(w1) * 32768.0f);    
    n1_a2 = (int32_t)lroundf(r * r * 32768.0f);    
    n2_b1 = (int32_t)lroundf(-2.0f * cosf(w2) * 32768.0f);    
    n2_a1 = (int32_t)lroundf(-2.0f * r * cosf(w2) * 32768.0f);    
    n2_a2 = n1_a2;    
    n1_x1 = n1_x2 = n1_y1 = n1_y2 = 0;    
    n2_x1 = n2_x2 = n2_y1 = n2_y2 = 0;    
}

// Расчёт peaking-биквада presence по tx_pres_freq_hz / tx_pres_q_x100 /    
// tx_presmax_q15 (RBJ). Вызывается из af_tx_init, af_tx_set_presmax,    
// af_tx_set_presfreq, af_tx_set_presq. Состояния pe_* не сбрасывает —    
// сброс только в af_tx_init (в сеттерах сброс дал бы щелчок).    
static void tx_pres_design(void) {    
    const float A  = sqrtf((float)tx_presmax_q15 / 32768.0f); // лин. амплитуда    
    const float w  = 2.0f * PI * (float)tx_pres_freq_hz / AF_FS_OUT_HZ;    
    const float cw = cosf(w), sw = sinf(w);    
    const float al = sw / (2.0f * tx_pres_q_x100 / 100.0f);    
    const float a0 = 1.0f + al / A;    
    pe_b0 = (int32_t)lroundf(((1.0f + al * A) / a0) * 32768.0f);    
    pe_b1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);    
    pe_b2 = (int32_t)lroundf(((1.0f - al * A) / a0) * 32768.0f);    
    pe_a1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);    
    pe_a2 = (int32_t)lroundf(((1.0f - al / A) / a0) * 32768.0f);    
}

// Расчёт кроссовера по tx_split_hz: RBJ LP+HP, Q=0.7071; HP инвертирован    
// -> сумма all-pass, -3 дБ каждой ветви на разделе. Заодно пересчитывает    
// tx_split_k_q15 — коэффициент старого комплементарного split (ФНЧ 1-го    
// порядка, режим txlr2 off): k = 2*pi*f/Fs, Q15.    
// Вызывается из af_tx_init и af_tx_set_split. Состояния xo_* не сбрасывает.    
static void tx_xo_design(void) {    
    const float w  = 2.0f * PI * (float)tx_split_hz / AF_FS_OUT_HZ;      
    const float cw = cosf(w), sw = sinf(w);      
    const float al = sw / (2.0f * 0.7071f);      
    const float a0 = 1.0f + al;      
    // LP      
    xo_lp_b0 = (int32_t)lroundf(((1.0f - cw) * 0.5f / a0) * 32768.0f);      
    xo_lp_b1 = (int32_t)lroundf(((1.0f - cw) / a0) * 32768.0f);      
    xo_lp_b2 = xo_lp_b0;      
    xo_lp_a1 = (int32_t)lroundf((-2.0f * cw / a0) * 32768.0f);      
    xo_lp_a2 = (int32_t)lroundf(((1.0f - al) / a0) * 32768.0f);      
    // HP — b-коэффициенты инвертированы: у RBJ-пары LP/HP выходы      
    // на fc в противофазе (180°), сумма lo+hi дала бы провал на      
    // кроссовере. С инверсией HP сумма = all-pass (плоская АЧХ).      
    xo_hp_b0 = -(int32_t)lroundf(((1.0f + cw) * 0.5f / a0) * 32768.0f);        
    xo_hp_b1 = -(int32_t)lroundf((-(1.0f + cw) / a0) * 32768.0f);        
    xo_hp_b2 = xo_hp_b0;        
    xo_hp_a1 = xo_lp_a1;      
    xo_hp_a2 = xo_lp_a2;      
    // Коэффициент старого split: k = 2*pi*f/Fs, Q15 (кламп 0..32768)    
    int32_t k = (int32_t)lroundf(w * 32768.0f);    
    if (k < 0)     k = 0;    
    if (k > 32768) k = 32768;    
    tx_split_k_q15 = k;    
}    

void af_tx_init(void) {
    agc_env   = 0;  
    gate_gain = 0;
    agc_blk_cnt = 0; comp_blk_cnt = 0;
    pres_blk_cnt   = 0;    
    pres_decim     = 0;    
    tx_gate_hold = 0;
    k_duck_q15     = 0;      
    k_duck_tgt_q15 = 0;      
    duck_blk_cnt   = 0;      
    exc_env_q16    = (int64_t)256 << 16;          
    exc_decim      = 0;          
    g_intel_q15     = 32768;  
    g_intel_tgt_q15 = 32768;  
    intel_blk_cnt   = 0;  
    tx_fir_design();  
    memset(&tx_pre,  0, sizeof(tx_pre));      
    memset(&tx_post, 0, sizeof(tx_post));      
    tx_pre.taps  = tx_fir_taps;  tx_pre.n  = TX_FIR_N;          
    tx_post.taps = tx_post_taps; tx_post.n = TX_POST_FIR_N;           
    tx_rtty_fir_design();    
    memset(&tx_rtty_fir1, 0, sizeof(tx_rtty_fir1));    
    memset(&tx_rtty_fir2, 0, sizeof(tx_rtty_fir2));  
    tx_rtty_dc_x1 = 0;    
    tx_rtty_dc_y1 = 0;    
    tx_rtty_fir1.taps = tx_rtty_taps;    
    tx_rtty_fir1.n    = TX_RTTY_FIR_N;    
    tx_rtty_fir2.taps = tx_rtty_taps;    
    tx_rtty_fir2.n    = TX_RTTY_FIR_N;
    tx_tt_fir_design();  
    memset(&tx_tt_fir1, 0, sizeof(tx_tt_fir1));  
    memset(&tx_tt_fir2, 0, sizeof(tx_tt_fir2));  
    tx_tt_fir1.taps = tx_tt_taps;  tx_tt_fir1.n = TX_TT_FIR_N;  
    tx_tt_fir2.taps = tx_tt_taps;  tx_tt_fir2.n = TX_TT_FIR_N;  
    tx_tt_dc_x1 = 0;  
    tx_tt_dc_y1 = 0;  
    tx_env_q16    = (int64_t)256 << 16;  
    tx_env_lo_q16 = (int64_t)256 << 16;  
    tx_env_hi_q16 = (int64_t)256 << 16;  
    split_lp = 0;  
    ap_x1 = 0;  
    ap_y1 = 0;
    tx_notch_design();
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
    // Этап 2: peaking-биквад presence (RBJ) — расчёт вынесен в функцию    
    tx_pres_design();    
    pe_x1 = pe_x2 = pe_y1 = pe_y2 = 0;    

    // Этап 4: кроссовер Баттерворта 2-го порядка — расчёт вынесен в функцию    
    tx_xo_design();    
    xo_lp_x1 = xo_lp_x2 = xo_lp_y1 = xo_lp_y2 = 0;      
    xo_hp_x1 = xo_hp_x2 = xo_hp_y1 = xo_hp_y2 = 0;      

    env_hi_pre_q16 = (int64_t)256 << 16;    
    pres_nf_q16    = (int64_t)256 << 16;    
    k_dyn_q15      = 0;    
    k_dyn_tgt_q15  = 0;
    // High-shelf: коэффициенты и состояния    
    af_tx_set_shelf(tx_shelf_db_x10);    
    sh_x1 = sh_x2 = sh_y1 = sh_y2 = 0;    
    dc_x1 = 0;    
    dc_y1 = 0;    
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
    tx_inv_3t2_q31 = (int32_t)(((int64_t)1 << 31) / (3LL * (int64_t)thr * thr));    
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

void af_tx_set_lr2(bool on)       { tx_lr2_on = on ? 1 : 0; }    
bool af_tx_lr2(void)              { return tx_lr2_on != 0; }    

void af_tx_set_duck(bool on)    { tx_duck_on = on ? 1 : 0; }    
bool af_tx_duck(void)           { return tx_duck_on != 0; }    
    
void af_tx_set_duckmax(uint32_t db_x10) {   // потолок ослабления, дБ x10    
    if (db_x10 > 120) db_x10 = 120;         // max -12 дБ    
    tx_duckmax_db_x10 = db_x10;    
    // Q15 = 1 - 10^(-dB/20): ослабление, не усиление    
    tx_duckmax_q15 = (int32_t)lroundf((1.0f - powf(10.0f, -(float)db_x10 / 200.0f)) * 32768.0f);    
}    
uint32_t af_tx_duckmax(void) { return tx_duckmax_db_x10; }   // дБ x10    
uint32_t af_tx_k_duck(void)  { return (uint32_t)k_duck_q15; }    

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
    tx_presmax_db_x10 = db_x10;             // для stats      
    // Q15 = 10^(dB/20): float только в сеттере, не в горячем пути      
    tx_presmax_q15 = (int32_t)lroundf(powf(10.0f, (float)db_x10 / 200.0f) * 32768.0f);      
    tx_pres_design();                       // пересчёт pe-биквада    
}    
uint32_t af_tx_presmax(void) { return tx_presmax_db_x10; }   // дБ x10      
  
void af_tx_set_presfreq(uint32_t hz) {      // центр presence-пика, Гц    
    if (hz < 500)  hz = 500;    
    if (hz > 6000) hz = 6000;    
    tx_pres_freq_hz = hz;    
    tx_pres_design();    
}    
uint32_t af_tx_presfreq(void) { return tx_pres_freq_hz; }    
  
void af_tx_set_presq(uint32_t q_x100) {     // добротность x100 (100 = 1.00)    
    if (q_x100 < 10)   q_x100 = 10;    
    if (q_x100 > 500)  q_x100 = 500;    
    tx_pres_q_x100 = q_x100;    
    tx_pres_design();    
}    
uint32_t af_tx_presq(void) { return tx_pres_q_x100; }    
  
// --- debug getters (этап 2) ---  
uint32_t af_tx_env_hi_pre(void) { return (uint32_t)(env_hi_pre_q16 >> 16); }  
uint32_t af_tx_pres_nf(void)    { return (uint32_t)(pres_nf_q16 >> 16); }  
uint32_t af_tx_k_dyn(void)      { return (uint32_t)k_dyn_q15; }  

// --- Этап 6: ВЧ-эксайтер ---      
void af_tx_set_exc(bool on)  { tx_exc_on = on ? 1 : 0; }      
bool af_tx_exc(void)         { return tx_exc_on != 0; }      
      
void af_tx_set_excmix(uint32_t q15) {   // подмес, Q15 (3277 ~ -20 дБ)      
    if (q15 > 16384u) q15 = 16384u;     // потолок -6 дБ — защита от «металла»      
    tx_exc_mix = (int32_t)q15;      
}      
uint32_t af_tx_excmix(void) { return (uint32_t)tx_exc_mix; }      

// --- Этап 7: k_intelligibility ---  
void af_tx_set_intel(bool on) { tx_intel_on = on ? 1 : 0; }  
bool af_tx_intel(void)        { return tx_intel_on != 0; }  
uint32_t af_tx_g_intel(void)  { return (uint32_t)g_intel_q15; }   // debug, Q15  

// --- High-shelf полка (вариант A) ---  
void af_tx_set_shelf(uint32_t db_x10) {   // подъём ВЧ, дБ x10 (0 = off)    
    if (db_x10 > 120) db_x10 = 120;       // max +12 дБ    
    tx_shelf_db_x10 = db_x10;    
    if (db_x10 == 0) return;              // байпас, коэффициенты не нужны    
    // RBJ high-shelf: A = 10^(dB/40), w = 2*pi*f0/Fs, alpha = sin(w)/(2Q)    
    const float A  = powf(10.0f, (float)db_x10 / 400.0f);    
    const float w  = 2.0f * PI * (float)tx_shelf_freq_hz / AF_FS_OUT_HZ;    
    const float cw = cosf(w), sw = sinf(w);    
    const float al = sw / (2.0f * tx_shelf_q_x100 / 100.0f);    
    const float sq = 2.0f * sqrtf(A) * al;  
    const float a0 = (A + 1.0f) - (A - 1.0f) * cw + sq;  
    sh_b0 = (int32_t)lroundf((A * ((A + 1.0f) + (A - 1.0f) * cw + sq) / a0) * 32768.0f);  
    sh_b1 = (int32_t)lroundf((-2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw) / a0) * 32768.0f);  
    sh_b2 = (int32_t)lroundf((A * ((A + 1.0f) + (A - 1.0f) * cw - sq) / a0) * 32768.0f);  
    sh_a1 = (int32_t)lroundf((-2.0f * ((A - 1.0f) - (A + 1.0f) * cw) / a0) * 32768.0f);  
    sh_a2 = (int32_t)lroundf(((A + 1.0f) - (A - 1.0f) * cw - sq) / a0 * 32768.0f);  
}  
uint32_t af_tx_shelf(void) { return tx_shelf_db_x10; }   // дБ x10  

void af_tx_set_shelffreq(uint32_t hz) {   // частота полки, Гц    
    if (hz < 500)  hz = 500;    
    if (hz > 6000) hz = 6000;    
    tx_shelf_freq_hz = hz;    
    if (tx_shelf_db_x10 != 0) af_tx_set_shelf(tx_shelf_db_x10);  // пересчёт    
}    
uint32_t af_tx_shelffreq(void) { return tx_shelf_freq_hz; }    
  
void af_tx_set_shelfq(uint32_t q_x100) {  // добротность x100    
    if (q_x100 < 10)   q_x100 = 10;    
    if (q_x100 > 500)  q_x100 = 500;    
    tx_shelf_q_x100 = q_x100;    
    if (tx_shelf_db_x10 != 0) af_tx_set_shelf(tx_shelf_db_x10);    
}    
uint32_t af_tx_shelfq(void) { return tx_shelf_q_x100; }    
  
// --- Пороги компрессора ---    
void af_tx_set_compthr(int32_t v) {    
    if (v < 256)   v = 256;    
    if (v > 32767) v = 32767;    
    tx_comp_thr = v;    
}    
void af_tx_set_compthrlo(int32_t v) {    
    if (v < 256)   v = 256;    
    if (v > 32767) v = 32767;    
    tx_comp_thr_lo = v;    
}    
void af_tx_set_compthrhi(int32_t v) {    
    if (v < 256)   v = 256;    
    if (v > 32767) v = 32767;    
    tx_comp_thr_hi = v;    
}    
int32_t af_tx_compthr(void)   { return tx_comp_thr; }    
int32_t af_tx_compthrlo(void) { return tx_comp_thr_lo; }    
int32_t af_tx_compthrhi(void) { return tx_comp_thr_hi; }    
  
// --- Частота кроссовера (пересчёт LR2-биквадов и k старого split) ---    
void af_tx_set_split(uint32_t hz) {    
    if (hz < 300)  hz = 300;    
    if (hz > 3000) hz = 3000;    
    tx_split_hz = hz;    
    tx_xo_design();    
}    
uint32_t af_tx_split(void) { return tx_split_hz; }    
  
// --- Гейт: hangover и гистерезис ---    
void af_tx_set_gatehold(uint32_t ms) {    
    if (ms > 2000) ms = 2000;    
    tx_gate_hold_ms = ms;    
}    
void af_tx_set_gatehyst(uint32_t x10) {    
    if (x10 < 1)  x10 = 1;    
    if (x10 > 10) x10 = 10;    
    tx_gate_hyst_x10 = x10;    
}    
uint32_t af_tx_gatehold(void) { return tx_gate_hold_ms; }    
uint32_t af_tx_gatehyst(void) { return tx_gate_hyst_x10; }    
  
// --- Потолок АРУ, x10 ---    
void af_tx_set_agcmax(uint32_t x10) {    
    if (x10 < 10)   x10 = 10;    
    if (x10 > 1000) x10 = 1000;    
    tx_agc_max_x10 = x10;    
}    
uint32_t af_tx_agcmax(void) { return tx_agc_max_x10; }    
  
// --- k_intelligibility: цель и потолки, Q15 ---    
void af_tx_set_inteltgt(int32_t v) {    
    if (v < 0)     v = 0;    
    if (v > 32768) v = 32768;    
    tx_intel_tgt_q15 = v;    
}    
void af_tx_set_intelcut(int32_t v) {    
    if (v < 0)     v = 0;    
    if (v > 32768) v = 32768;    
    tx_intel_cut_q15 = v;    
}    
void af_tx_set_intelboost(int32_t v) {    
    if (v < 0)     v = 0;    
    if (v > 32768) v = 32768;    
    tx_intel_boost_q15 = v;    
}    
int32_t af_tx_inteltgt(void)   { return tx_intel_tgt_q15; }    
int32_t af_tx_intelcut(void)   { return tx_intel_cut_q15; }    
int32_t af_tx_intelboost(void) { return tx_intel_boost_q15; }    
  
// --- Частоты TX-нотчей (пересчёт коэффициентов) ---    
void af_tx_set_notchf(uint32_t f1, uint32_t f2) {    
    if (f1 < 200)  f1 = 200;    
    if (f1 > 7000) f1 = 7000;    
    if (f2 < 200)  f2 = 200;    
    if (f2 > 7000) f2 = 7000;    
    tx_notch1_hz = f1;    
    tx_notch2_hz = f2;    
    tx_notch_design();    
}    
uint32_t af_tx_notch1(void) { return tx_notch1_hz; }    
uint32_t af_tx_notch2(void) { return tx_notch2_hz; }    