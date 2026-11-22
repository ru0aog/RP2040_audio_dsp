// src/modes/rtty_rx.cpp — RTTY-декодер 45.45/170, ITA-2 + МТК-2.  
// Ядро — порт rx_decoders.cpp (RP2040_beacon) -> fldigi rtty.cxx:  
//   x -> mix(mark)  -> boxcar(Tбит) -> env_m  
//     -> mix(space) -> boxcar(Tбит) -> env_s -> clipped ATC v3 -> бит-автомат  
// AFC: dphi между boxcar-суммами на стробах битов, кламп +-22.7 Гц, k=0.25.  
// Захват: RSID (rsid_rx.cpp) или сканер Гёрцеля (пара пиков со сдвигом 170).  
// Обвязка rttydec_*: Core 1 push -> SPSC -> Core 0 poll (как cwdec_*).  
#include "rtty_rx.h"  
#include "rsid_rx.h"  
#include "../af_config.h"  
#include <math.h>  
#include <string.h>  
  
#define RTTY_FS        ((float)AF_FS_OUT_HZ)   // 15464  
#define RTTY_SYMLEN    ((int)(RTTY_FS / RTTY_BAUD + 0.5f))   // ~340  
#define RTTY_MAX_SYMLEN 400  
  
// ---------------------------------------------------------------------------  
// Состояние декодера (порт rtty_t из rx_decoders.cpp)  
// ---------------------------------------------------------------------------  
typedef struct {  
    float fs;  
    int   symbollen;  
    int   nbits;              // 5  
  
    float mark_phase, space_phase;  
    float f_mark, f_space;    // AFC сдвигает  
  
    float buf_mi[RTTY_MAX_SYMLEN], buf_mq[RTTY_MAX_SYMLEN];  
    float buf_si[RTTY_MAX_SYMLEN], buf_sq[RTTY_MAX_SYMLEN];  
    int   buf_idx;  
    float sum_mi, sum_mq, sum_si, sum_sq;  
  
    float mark_env, space_env;  
    float mark_mag, space_mag;  
    float mark_noise, space_noise;  
    float noise_floor;  
    float v3;  
  
    bool  bit_buf[RTTY_MAX_SYMLEN];  
    int   rxstate;            // 0=IDLE 1=START 2=DATA 3=STOP  
    int   counter, bitcntr, rxdata;  
    int   figs;               // 0=LTRS 1=FIGS 2=RUS  
  
    float h_m_re, h_m_im;     // mark-сумма на прошлом стробе (AFC)  
    bool  h_m_valid;  
    float freqerr;  
    int   sym_since_char;  
  
    float dc_i, dc_q;  
    bool  lastbit;  
} rtty_t;  
  
static rtty_t rrx;  
static bool   rtty_active = false;  
static bool   rtty_rev    = false;  
static float  sq_level    = 0.15f;   // сквелч по SNR огибающих, 0=off  
static bool   sq_open     = true;  
  
static rttydec_state_t state = RTTY_ST_SCAN;  
static bool   scan_en  = true;  
static bool   rsid_en  = true;  
static uint32_t track_lost_ms = 0;   // таймаут потери сигнала в TRACK  
  
// ---------------------------------------------------------------------------  
// Таблицы Бодо / МТК-2 (verbatim из rx_decoders.cpp)  
// ---------------------------------------------------------------------------  
static const char baudot_ltrs[32] = {  
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0 };  
static const char baudot_figs[32] = {  
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',  
    '5','+',')','2','#','6','0','1','9','?','&',0,'.','/',';',0 };  
static const char *baudot_rus[32] = {  
    "",  "Е", "\n", "А", " ", "С", "И", "У",  
    "\r","Д", "Р", "Й", "Н", "Ф", "Ц", "К",  
    "Т", "З", "Л", "В", "Х", "Ы", "П", "Я",  
    "О", "Б", "Г", "",  "М", "Ь", "Ш", "" };  
  
#define MTK2_RUS  0x00  
#define MTK2_LAT  0x1F  
#define MTK2_FIG  0x1B  
  
static void rtty_emit(uint8_t c) {  
    if (c == MTK2_FIG) { rrx.figs = 1; return; }  
    if (c == MTK2_LAT) { rrx.figs = 0; return; }  
    if (c == MTK2_RUS) { rrx.figs = 2; return; }   // blank -> RUS-регистр МТК-2  
    if (!sq_open) return;  
    if (rrx.figs == 2) {  
        const char *s = baudot_rus[c];  
        if (s[0]) Serial.print(s);  
    } else {  
        char ch = rrx.figs ? baudot_figs[c] : baudot_ltrs[c];  
        if (ch) Serial.write(ch);  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Ядро: миксер, boxcar, ATC v3, бит-автомат (порт rtty.cxx)  
// ---------------------------------------------------------------------------  
static inline void rtty_mixer(float &phase, float f, float xr, float xq,  
                              float &or_, float &oi) {  
    float c = cosf(phase), s = sinf(phase);  
    or_ = xr * c + xq * s;  
    oi  = xq * c - xr * s;  
    phase += 2.0f * (float)M_PI * f / rrx.fs;  
    if (phase >  2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;  
    if (phase < -2.0f * (float)M_PI) phase += 2.0f * (float)M_PI;  
}  
  
static inline float decayavg(float a, float in, float w) {  
    if (w <= 1.0f) return in;  
    return a + (in - a) / w;  
}  
  
static bool rtty_is_mark_space(int &correction) {  
    correction = 0;  
    if (rrx.bit_buf[0] && !rrx.bit_buf[rrx.symbollen - 1]) {  
        for (int i = 0; i < rrx.symbollen; i++)  
            correction += rrx.bit_buf[i];  
        if (abs(rrx.symbollen / 2 - correction) < 6)  
            return true;  
    }  
    return false;  
}  
  
static inline bool rtty_is_mark(void) {  
    return rrx.bit_buf[rrx.symbollen / 2];  
}  
  
static bool rtty_rx(bool bit) {  
    bool flag = false;  
    int  correction;  
    memmove(rrx.bit_buf, rrx.bit_buf + 1, rrx.symbollen - 1);  
    rrx.bit_buf[rrx.symbollen - 1] = bit;  
  
    switch (rrx.rxstate) {  
    case 0: // IDLE — фронт mark->space (старт-бит)  
        if (rtty_is_mark_space(correction)) {  
            rrx.rxstate = 1;  
            rrx.counter = correction;  
        }  
        break;  
    case 1: // START — подтверждение space в середине  
        if (--rrx.counter == 0) {  
            if (!rtty_is_mark()) {  
                rrx.rxstate = 2;  
                rrx.counter = rrx.symbollen;  
                rrx.bitcntr = 0;  
                rrx.rxdata  = 0;  
            } else rrx.rxstate = 0;  
        }  
        break;  
    case 2: // DATA — nbits стробов по середине бита  
        if (--rrx.counter == 0) {  
            bool mk = rtty_is_mark();  
            rrx.rxdata |= (mk ? 1u : 0u) << rrx.bitcntr++;  
            rrx.counter = rrx.symbollen;  
            if (rrx.bitcntr >= rrx.nbits) rrx.rxstate = 3;  
        }  
        break;  
    case 3: // STOP — ждём mark  
        if (--rrx.counter == 0) {  
            if (rtty_is_mark()) {  
                rtty_emit((uint8_t)(rrx.rxdata & 0x1F));  
                flag = true;  
            }  
            rrx.rxstate = 0;  
        }  
        break;  
    }  
    return flag;  
}  
  
static void rtty_rx_sample(float xr, float xq) {  
    // DC-трекер (LO-feedthrough лезет в каналы через боковые лепестки boxcar)  
    rrx.dc_i += (xr - rrx.dc_i) / 512.0f;  
    rrx.dc_q += (xq - rrx.dc_q) / 512.0f;  
    xr -= rrx.dc_i;  
    xq -= rrx.dc_q;  
  
    float m_r, m_i, s_r, s_i;  
    rtty_mixer(rrx.mark_phase,  rrx.f_mark,  xr, xq, m_r, m_i);  
    rtty_mixer(rrx.space_phase, rrx.f_space, xr, xq, s_r, s_i);  
  
    int L = rrx.symbollen, idx = rrx.buf_idx;  
    rrx.sum_mi += m_r - rrx.buf_mi[idx]; rrx.buf_mi[idx] = m_r;  
    rrx.sum_mq += m_i - rrx.buf_mq[idx]; rrx.buf_mq[idx] = m_i;  
    rrx.sum_si += s_r - rrx.buf_si[idx]; rrx.buf_si[idx] = s_r;  
    rrx.sum_sq += s_i - rrx.buf_sq[idx]; rrx.buf_sq[idx] = s_i;  
    if (++rrx.buf_idx >= L) rrx.buf_idx = 0;  
  
    rrx.mark_mag  = sqrtf(rrx.sum_mi*rrx.sum_mi + rrx.sum_mq*rrx.sum_mq) / L;  
    rrx.space_mag = sqrtf(rrx.sum_si*rrx.sum_si + rrx.sum_sq*rrx.sum_sq) / L;  
  
    float T = (float)rrx.symbollen;  
    rrx.mark_env    = decayavg(rrx.mark_env,   rrx.mark_mag,  
                      rrx.mark_mag  > rrx.mark_env   ? T/4 : T*16);  
    rrx.mark_noise  = decayavg(rrx.mark_noise, rrx.mark_mag,  
                      rrx.mark_mag  < rrx.mark_noise ? T/4 : T*48);  
    rrx.space_env   = decayavg(rrx.space_env,  rrx.space_mag,  
                      rrx.space_mag > rrx.space_env  ? T/4 : T*16);  
    rrx.space_noise = decayavg(rrx.space_noise,rrx.space_mag,  
                      rrx.space_mag < rrx.space_noise? T/4 : T*48);  
    rrx.noise_floor = fminf(rrx.space_noise, rrx.mark_noise);  
  
    // clipped ATC + optimal v3  
    float mclipped = fminf(rrx.mark_mag,  rrx.mark_env);  
    float sclipped = fminf(rrx.space_mag, rrx.space_env);  
    if (mclipped < rrx.noise_floor) mclipped = rrx.noise_floor;  
    if (sclipped < rrx.noise_floor) sclipped = rrx.noise_floor;  
    float mn = rrx.mark_env  - rrx.noise_floor;  
    float sn = rrx.space_env - rrx.noise_floor;  
    rrx.v3 = (mclipped - rrx.noise_floor) * mn  
           - (sclipped - rrx.noise_floor) * sn  
           - 0.25f * (mn*mn - sn*sn);  
  
    bool bit = (rrx.v3 > 0);  
    if (rtty_rev) bit = !bit;              // инверсия полярности  
  
    // Сквелч: отношение огибающей сильнейшего тона к шуму  
    float env = fmaxf(rrx.mark_env, rrx.space_env);  
    float snr = (rrx.noise_floor > 1e-6f) ? env / rrx.noise_floor : 0.0f;  
    if (sq_level == 0.0f)            sq_open = true;  
    else if (snr > 1.0f + sq_level * 9.0f)  sq_open = true;   // ~x(1+9*lvl)  
    else if (snr < 1.0f + sq_level * 4.0f)  sq_open = false;  
  
    bool valid = rtty_rx(bit);  
    rrx.lastbit = bit;  
    rrx.sym_since_char++;  
  
    // AFC: dphi между стробами, применение на валидном символе  
    if (valid) {  
        if (rrx.h_m_valid) {  
            float dphi = atan2f(  
                rrx.sum_mi * rrx.h_m_im - rrx.sum_mq * rrx.h_m_re,  
                rrx.sum_mi * rrx.h_m_re + rrx.sum_mq * rrx.h_m_im);  
            int n = rrx.sym_since_char; if (n < 1) n = 1;  
            rrx.freqerr = dphi * rrx.fs / (2.0f * (float)M_PI * n);  
            float corr = rrx.freqerr;  
            if (corr >  22.7f) corr =  22.7f;  
            if (corr < -22.7f) corr = -22.7f;  
            rrx.f_mark  -= corr * 0.25f;  
            rrx.f_space -= corr * 0.25f;  
        }  
        rrx.h_m_re = rrx.sum_mi; rrx.h_m_im = rrx.sum_mq;  
        rrx.h_m_valid = true;  
        rrx.sym_since_char = 0;  
        track_lost_ms = millis();          // сигнал жив  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Инициализация каналов на заданный центр (center = середина mark/space)  
// ---------------------------------------------------------------------------  
static void rtty_tune(float center_hz) {  
    float half = RTTY_SHIFT_HZ * 0.5f;  
    rrx.f_mark  = center_hz + half;    // mark — верхний тон (норма)  
    rrx.f_space = center_hz - half;  
    rrx.mark_phase = rrx.space_phase = 0.0f;  
    memset(rrx.buf_mi, 0, sizeof(rrx.buf_mi));  
    memset(rrx.buf_mq, 0, sizeof(rrx.buf_mq));  
    memset(rrx.buf_si, 0, sizeof(rrx.buf_si));  
    memset(rrx.buf_sq, 0, sizeof(rrx.buf_sq));  
    memset(rrx.bit_buf, 0, sizeof(rrx.bit_buf));  
    rrx.buf_idx = 0;  
    rrx.sum_mi = rrx.sum_mq = rrx.sum_si = rrx.sum_sq = 0.0f;  
    rrx.mark_env = rrx.space_env = 0.0f;  
    rrx.mark_noise = rrx.space_noise = rrx.noise_floor = 0.0f;  
    rrx.rxstate = 0; rrx.counter = 0; rrx.bitcntr = 0; rrx.rxdata = 0;  
    rrx.figs = 0;  
    rrx.h_m_valid = false; rrx.freqerr = 0.0f; rrx.sym_since_char = 0;  
    rrx.dc_i = rrx.dc_q = 0.0f;  
    track_lost_ms = millis();  
}  
  
// ---------------------------------------------------------------------------  
// Сканер Гёрцеля: банк тонов 300..3000 Гц, ищем пару пиков со сдвигом 170 Гц.  
// Работает на децимированном потоке /4 (3866 Гц) — 27 тонов, окно ~53 мс.  
// ---------------------------------------------------------------------------  
#define SCAN_FS       (RTTY_FS / 4.0f)          // 3866 Гц  
#define SCAN_NTONES   27                         // 300..3000 шаг ~104 Гц  
#define SCAN_F0       300.0f  
#define SCAN_STEP     ((3000.0f - 300.0f) / (SCAN_NTONES - 1))  
#define SCAN_WIN      205                        // отсчётов @3866 = ~53 мс  
#define SCAN_SHIFT_T  2                          // 170 Гц ≈ 1.63 тона -> ±1  
#define SCAN_NEED     3                          // окон подряд для захвата  
  
typedef struct { float cr, ci, ar, ai; } gtz_t;  
static gtz_t   gtz[SCAN_NTONES];  
static int     scan_decim = 0, scan_cnt = 0;  
static int     scan_hit_tone = -1, scan_hit_n = 0;  
  
static void scan_init(void) {  
    for (int t = 0; t < SCAN_NTONES; t++) {  
        float w = 2.0f * (float)M_PI * (SCAN_F0 + t * SCAN_STEP) / SCAN_FS;  
        gtz[t].cr = cosf(w); gtz[t].ci = sinf(w);  
        gtz[t].ar = gtz[t].ai = 0.0f;  
    }  
    scan_decim = scan_cnt = 0;  
    scan_hit_tone = -1; scan_hit_n = 0;  
}  
  
// Один децимированный отсчёт в банк; по окну — поиск пары.  
static void scan_sample(float x) {  
    for (int t = 0; t < SCAN_NTONES; t++) {  
        gtz_t *g = &gtz[t];  
        float tr = x + g->cr * g->ar - g->ci * g->ai;  
        float ti =     g->cr * g->ai + g->ci * g->ar;  
        g->ar = tr; g->ai = ti;  
    }  
    if (++scan_cnt < SCAN_WIN) return;  
    scan_cnt = 0;  
  
    // Мощности + общий шум (медиана грубо — среднее)  
    float p[SCAN_NTONES], avg = 0.0f;  
    for (int t = 0; t < SCAN_NTONES; t++) {  
        p[t] = gtz[t].ar * gtz[t].ar + gtz[t].ai * gtz[t].ai;  
        avg += p[t];  
        gtz[t].ar = gtz[t].ai = 0.0f;  
    }  
    avg /= SCAN_NTONES;  
    if (avg < 1.0f) avg = 1.0f;  
  
    // Пара (t, t+2): оба пика > 6x среднего — кандидат space/mark  
    int best = -1; float best_p = 0.0f;  
    for (int t = 0; t + SCAN_SHIFT_T < SCAN_NTONES; t++) {  
        float pp = fminf(p[t], p[t + SCAN_SHIFT_T]);  
        if (p[t] > avg * 6.0f && p[t + SCAN_SHIFT_T] > avg * 6.0f && pp > best_p) {  
            best_p = pp; best = t;  
        }  
    }  
    if (best >= 0 && best == scan_hit_tone) {  
        if (++scan_hit_n >= SCAN_NEED) {  
            // Захват: центр = середина пары  
            float f_lo = SCAN_F0 + best * SCAN_STEP;  
            float f_hi = SCAN_F0 + (best + SCAN_SHIFT_T) * SCAN_STEP;  
            rtty_tune((f_lo + f_hi) * 0.5f);  
            state = RTTY_ST_TRACK;  
            Serial.printf("# rtty lock: mark=%.0f space=%.0f\n",  
                          (double)rrx.f_mark, (double)rrx.f_space);  
            scan_hit_tone = -1; scan_hit_n = 0;  
            return;  
        }  
    } else {  
        scan_hit_tone = best; scan_hit_n = (best >= 0) ? 1 : 0;  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Обвязка rttydec_* — SPSC-очередь (как cwdec_*)  
// ---------------------------------------------------------------------------  
#define RTTYDEC_Q_LEN 512  
static volatile int16_t  rtty_q[RTTYDEC_Q_LEN];  
static volatile uint16_t rtty_q_head = 0, rtty_q_tail = 0;  
static volatile uint32_t rtty_dropped_cnt = 0;  
  
void rttydec_init(void) {  
    rtty_q_head = rtty_q_tail = 0;  
    rtty_dropped_cnt = 0;  
    memset(&rrx, 0, sizeof(rrx));  
    rrx.fs = RTTY_FS;  
    rrx.symbollen = RTTY_SYMLEN;  
    rrx.nbits = 5;  
    sq_open = (sq_level == 0.0f);  
    scan_init();  
    if (rsid_en) {  
        rsid_rx_init();  
        state = RTTY_ST_RSID;  
    } else {  
        state = scan_en ? RTTY_ST_SCAN : RTTY_ST_TRACK;  
        if (state == RTTY_ST_TRACK) rtty_tune(RTTY_DEF_CENTER);  
    }  
    rtty_active = true;  
    Serial.printf("# rtty on: %s\n",  
                  state == RTTY_ST_RSID ? "rsid" :  
                  state == RTTY_ST_SCAN ? "scan" : "track");  
}  
  
void rttydec_push(int16_t x) {          // Core 1, горячий путь  
    uint16_t h = rtty_q_head;  
    uint16_t n = (uint16_t)((h + 1) & (RTTYDEC_Q_LEN - 1));  
    if (n == rtty_q_tail) { ++rtty_dropped_cnt; return; }  
    rtty_q[h] = x;  
    __dmb();  
    rtty_q_head = n;  
}  
  
void rttydec_poll(void) {               // Core 0, из loop()  
    if (!rtty_active) return;  
  
    // RSID-результат (детект внутри rsid_rx_poll)  
    float rsid_f;  
    if (state == RTTY_ST_RSID && rsid_get_rtty_center(&rsid_f)) {  
        rtty_tune(rsid_f);  
        state = RTTY_ST_TRACK;  
        Serial.printf("# rtty rsid lock: center=%.1f\n", (double)rsid_f);  
    }  
    // RSID не пришёл за ~4 с -> fallback на сканер  
    static uint32_t rsid_wait_ms = 0;  
    if (state == RTTY_ST_RSID) {  
        if (rsid_wait_ms == 0) rsid_wait_ms = millis();  
        if (scan_en && millis() - rsid_wait_ms > 4000) {  
            state = RTTY_ST_SCAN;  
            Serial.println("# rtty: rsid timeout -> scan");  
        }  
    } else rsid_wait_ms = 0;  
  
    // Потеря сигнала в TRACK: 5 с без валидных символов -> перезахват  
    if (state == RTTY_ST_TRACK && millis() - track_lost_ms > 5000) {  
        state = rsid_en ? RTTY_ST_SCAN : RTTY_ST_SCAN;  // сканер быстрее  
        scan_init();  
        Serial.println("# rtty: lost -> scan");  
    }  
  
    while (rtty_q_tail != rtty_q_head) {  
        int16_t x = rtty_q[rtty_q_tail];  
        rtty_q_tail = (uint16_t)((rtty_q_tail + 1) & (RTTYDEC_Q_LEN - 1));  
  
        // RSID слушает всегда в состоянии RSID (ресемплер внутри)  
        if (state == RTTY_ST_RSID) rsid_rx_push15464(x);  
  
        if (state == RTTY_ST_SCAN) {  
            if (++scan_decim >= 4) { scan_decim = 0; scan_sample((float)x); }  
            continue;                              // в SCAN не декодируем  
        }  
        rtty_rx_sample((float)x, 0.0f);            // моно: xq=0  
    }  
  
    if (state == RTTY_ST_RSID) rsid_rx_poll();     // квант FFT/поиска  
}  
  
void rttydec_stop(void) {  
    rtty_q_head = rtty_q_tail = 0;  
    rtty_active = false;  
    rsid_rx_stop();  
}  
  
// ---- настройки ----  
void rttydec_set_scan(bool on)  { scan_en = on; }  
void rttydec_set_rsid(bool on)  { rsid_en = on; }  
void rttydec_set_rev(bool on)   { rtty_rev = on; }  
void rttydec_set_sql(float lvl) {  
    if (lvl < 0.0f || lvl > 0.9f) return;  
    sq_level = lvl;  
    if (lvl == 0.0f) sq_open = true;  
}  
void rttydec_set_center(float hz) {  
    if (hz < 200.0f) hz = 200.0f;  
    if (hz > 3000.0f) hz = 3000.0f;  
    rtty_tune(hz);  
    state = RTTY_ST_TRACK;  
}  
  
// ---- статус ----  
rttydec_state_t rttydec_state(void)  { return state; }  
float rttydec_f_mark(void)   { return rrx.f_mark; }  
float rttydec_f_space(void)  { return rrx.f_space; }  
float rttydec_freqerr(void)  { return rrx.freqerr; }  
bool  rttydec_rev(void)      { return rtty_rev; }  
bool  rttydec_scan_en(void)  { return scan_en; }  
bool  rttydec_rsid_en(void)  { return rsid_en; }  
float rttydec_sql(void)      { return sq_level; }  
uint32_t rttydec_dropped(void) { return rtty_dropped_cnt; }  
