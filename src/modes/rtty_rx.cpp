// src/modes/rtty_rx.cpp — RTTY-декодер 45.45/170, ITA-2 + МТК-2.  
// Ядро — порт rx_decoders.cpp (RP2040_beacon) -> fldigi rtty.cxx:  
//   x -> mix(mark)  -> boxcar(Tбит) -> env_m  
//     -> mix(space) -> boxcar(Tбит) -> env_s -> clipped ATC v3 -> бит-автомат  
// AFC: dphi между boxcar-суммами на стробах битов, кламп +-22.7 Гц, k=0.25.  
// Захват: RSID (rsid_rx.cpp) или сканер Гёрцеля (пара пиков со сдвигом 170).  
// Обвязка rttydec_*: Core 1 push = децимация /4; дальше по состоянию:      
//   SCAN  — отсчёты в SPSC-очередь -> сканер Гёрцеля на Core 0 (poll);      
//   TRACK — миксеры/boxcar/ATC/бит-автомат на Core 1, символы в chq.      
// Core 0 poll: сканер + дренаж chq -> Serial/OLED + таймауты lost/false-lock.        
#include "rtty_rx.h"    
// #include "rsid_rx.h"     // RSID временно отключён — экономия ~50 КБ RAM   
#include "af_oled.h"        // af_oled_putc — эхо декодированного текста на экран  
#include "../af_config.h"    
#include <math.h>    
#include <string.h>    
    
#define RTTY_FS        ((float)AF_FS_OUT_HZ)   // 15464 — вход push    
#define RTTY_DECIM     4                       // децимация в push (Core 1)    
#define RTTY_DFS       (RTTY_FS / RTTY_DECIM)  // 3866 — Fs декодера/сканера    
#define RTTY_SYMLEN    ((int)(RTTY_DFS / RTTY_BAUD + 0.5f))  // ~85    
#define RTTY_MAX_SYMLEN 128    
  
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
static volatile bool   rtty_active = false;    
static volatile bool   rtty_rev    = false;    
static volatile float  sq_level    = 0.15f;  // сквелч по SNR огибающих, 0=off    
static volatile bool   sq_open     = true;    
  
static volatile rttydec_state_t state = RTTY_ST_SCAN;  
static bool   scan_en  = true;    
static bool   rsid_en  = true;    
static bool   dbg_en   = false;   // rtty dbg on|off — диагностика scan/trk    
static volatile uint32_t track_lost_ms = 0;  // таймаут потери сигнала в TRACK (пишет Core 1, читает Core 0)  
static float    ban_lo = 0, ban_hi = 0;  // забаненная пара (Гц), 0 = нет    
static uint32_t ban_ms = 0;              // когда забанена (TTL 30 с)    
  
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
  
// Очередь декодированных байт Core1->Core0 (UTF-8 кириллица идёт побайтно).      
#define RTTY_CHQ_LEN 256      
static volatile uint8_t  chq[RTTY_CHQ_LEN];      
static volatile uint16_t chq_head = 0, chq_tail = 0;      
  
// Очередь децимированных отсчётов Core1->Core0 для сканера (только в SCAN).      
#define RTTY_SQ_LEN 256        
static volatile int16_t  sq[RTTY_SQ_LEN];        
static volatile uint16_t sq_head = 0, sq_tail = 0;        
static volatile uint32_t rtty_dropped_cnt = 0;  // переполнение sq (Core 0 не успевает)  
        
static void sq_put(int16_t v) {             // Core 1           
    uint16_t n = (uint16_t)((sq_head + 1) & (RTTY_SQ_LEN - 1));      
    if (n == sq_tail) { rtty_dropped_cnt++; return; }  // Core 0 не успевает      
    sq[sq_head] = v;      
    __dmb();      
    sq_head = n;      
}       
    
static void chq_put(uint8_t b) {          // Core 1    
    uint16_t n = (uint16_t)((chq_head + 1) & (RTTY_CHQ_LEN - 1));    
    if (n == chq_tail) return;            // переполнение — байт теряем    
    chq[chq_head] = b;    
    __dmb();    
    chq_head = n;    
}    
    
static void rtty_emit(uint8_t c) {        // Core 1 — только в очередь    
    if (c == MTK2_FIG) { rrx.figs = 1; return; }    
    if (c == MTK2_LAT) { rrx.figs = 0; return; }    
    if (c == MTK2_RUS) { rrx.figs = 2; return; }   // blank -> RUS-регистр МТК-2    
    if (!sq_open) return;    
    if (rrx.figs == 2) {      
        const char *s = baudot_rus[c];      
        for (const char *p = s; *p; p++) chq_put((uint8_t)*p);  // UTF-8 байты    
    } else {      
        char ch = rrx.figs ? baudot_figs[c] : baudot_ltrs[c];      
        if (ch) chq_put((uint8_t)ch);      
    }    
}    
  
// ---------------------------------------------------------------------------    
// Ядро: миксер, boxcar, ATC v3, бит-автомат (порт rtty.cxx)    
// ---------------------------------------------------------------------------    
// LUT sin/cos: 512 записей на оборот (~0.7° шаг, ошибка <0.02%).    
// sinf/cosf на M0+ — ~300 тактов каждый, x2 на отсчёт @3866 = ~4.5M такт/с.    
// LUT: ~10 тактов — экономия ~97% на осцилляторах.    
#define NCO_LUT_BITS 9    
#define NCO_LUT_LEN  (1 << NCO_LUT_BITS)          // 512    
static float nco_sin[NCO_LUT_LEN];    
static bool  nco_lut_ready = false;    
    
static void nco_lut_init(void) {                  // Core 0, из rttydec_init    
    for (int i = 0; i < NCO_LUT_LEN; i++)    
        nco_sin[i] = sinf(2.0f * (float)M_PI * i / NCO_LUT_LEN);    
    nco_lut_ready = true;    
}    
    
static inline void nco_sincos(float phase, float &c, float &s) {    
    // phase в радианах, любой знак — приводим к индексу LUT    
    int idx = (int)(phase * (NCO_LUT_LEN / (2.0f * (float)M_PI)));    
    idx &= (NCO_LUT_LEN - 1);    
    s = nco_sin[idx];    
    c = nco_sin[(idx + NCO_LUT_LEN / 4) & (NCO_LUT_LEN - 1)];  // cos = sin+90°    
}    
    
static inline void rtty_mixer(float &phase, float f, float xr, float xq,    
                              float &or_, float &oi) {    
    float c, s;    
    nco_sincos(phase, c, s);    
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
    
    // sqrtf ~200 тактов x2 на отсчёт @3866 = ~1.5M такт/с.    
    // Огибающая меняется медленно (T/4..T*48) — считаем раз в symbollen,    
    // между обновлениями держим прошлое значение.    
    static int mag_cnt = 0;    
    if (++mag_cnt >= L) {    
        mag_cnt = 0;    
        rrx.mark_mag  = sqrtf(rrx.sum_mi*rrx.sum_mi + rrx.sum_mq*rrx.sum_mq) / L;    
        rrx.space_mag = sqrtf(rrx.sum_si*rrx.sum_si + rrx.sum_sq*rrx.sum_sq) / L;    
    }    
  
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
#define SCAN_FS       RTTY_DFS                   // 3866 — поток после децимации    
#define SCAN_NTONES   19                         // 300..1200 шаг 50 Гц (Найквист 1933)    
#define SCAN_F0       300.0f    
#define SCAN_STEP     50.0f    
#define SCAN_WIN      128                        // отсчётов @3866 = ~33 мс    
#define SCAN_NEED     3                          // окон подряд для захвата    
    
typedef struct { float cr, ci, ar, ai; } gtz_t;    
static gtz_t   gtz[SCAN_NTONES];    
static int     scan_cnt = 0;    
static int     scan_hit_tone = -1, scan_hit_n = 0;    
    
static void scan_init(void) {    
    for (int t = 0; t < SCAN_NTONES; t++) {    
        float w = 2.0f * (float)M_PI * (SCAN_F0 + t * SCAN_STEP) / SCAN_FS;    
        gtz[t].cr = cosf(w); gtz[t].ci = sinf(w);    
        gtz[t].ar = gtz[t].ai = 0.0f;    
    }    
    scan_cnt = 0;    
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
    
    // Топ-пик — исключаем его из среднего: мешающая несущая (2792 Гц)    
    // не должна завышать порог для остальных тонов.    
    int i_top = 0;    
    for (int t = 1; t < SCAN_NTONES; t++)    
        if (p[t] > p[i_top]) i_top = t;    
    float avg2 = (avg * SCAN_NTONES - p[i_top]) / (SCAN_NTONES - 1);    
    if (avg2 < 1.0f) avg2 = 1.0f;    
    
    // Пара mark/space: для каждого пика t ищем партнёра в окне 170±45 Гц.    
    // Порог 3x среднего — второй тон слабее и размазан между бинами.    
    int best = -1, best_u = -1; float best_p = 0.0f;      
    for (int t = 0; t < SCAN_NTONES; t++) {      
        if (p[t] < avg2 * 3.0f) continue;      
        float f_t = SCAN_F0 + t * SCAN_STEP;      
        for (int u = t + 1; u < SCAN_NTONES; u++) {      
            float f_u = SCAN_F0 + u * SCAN_STEP;      
            float d = f_u - f_t;      
            if (d < 125.0f) continue;      
            if (d > 215.0f) break;      
            if (p[u] < avg2 * 3.0f) continue;        
            // Пропуск забаненной пары (TTL 30 с — частота QSO могла смениться)    
            if (ban_lo > 0 && millis() - ban_ms < 30000 &&    
                fabsf(f_t - ban_lo) < 60.0f && fabsf(f_u - ban_hi) < 60.0f) continue;    
            float pp = fminf(p[t], p[u]);        
            if (pp > best_p) { best_p = pp; best = t; best_u = u; }      
        }      
    }      
  
    // --- ДИАГНОСТИКА: лучшая пара и топ-3 пика раз в секунду ---      
    if (dbg_en) {      
        static uint32_t dbg_ms = 0;      
        if (millis() - dbg_ms > 1000) {      
            dbg_ms = millis();    
            // топ-3 тона по мощности    
            int i1 = 0, i2 = 0, i3 = 0;    
            for (int t = 0; t < SCAN_NTONES; t++) {    
                if (p[t] > p[i1])      { i3 = i2; i2 = i1; i1 = t; }    
                else if (p[t] > p[i2]) { i3 = i2; i2 = t; }    
                else if (p[t] > p[i3]) { i3 = t; }    
            }    
            Serial.printf("# scan: avg=%d thr=%d | top: %dHz=%d %dHz=%d %dHz=%d | best=%s hit=%d/%d\n",  
                (int)avg2, (int)(avg2 * 3.0f),    
                (int)(SCAN_F0 + i1 * SCAN_STEP), (int)p[i1],  
                (int)(SCAN_F0 + i2 * SCAN_STEP), (int)p[i2],  
                (int)(SCAN_F0 + i3 * SCAN_STEP), (int)p[i3],  
                best >= 0 ? "pair" : "-", scan_hit_n, SCAN_NEED);  
        }    
    }    
  
    if (best >= 0 && best == scan_hit_tone) {    
        if (++scan_hit_n >= SCAN_NEED) {  
            // Захват: центр = середина пары  
            float f_lo = SCAN_F0 + best * SCAN_STEP;    
            float f_hi = SCAN_F0 + best_u * SCAN_STEP;   // реальный партнёр, не t+2    
            rtty_tune((f_lo + f_hi) * 0.5f);  
            state = RTTY_ST_TRACK;  
            Serial.printf("# rtty lock: mark=%d space=%d\n",    
                          (int)rrx.f_mark, (int)rrx.f_space);    
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
static int push_decim = 0;              // счётчик децимации /4 (Core 1)      
// rtty_dropped_cnt объявлен выше, у очереди sq — используется в sq_put  
  
void rttydec_init(void) {    
    chq_head = chq_tail = 0;      
    sq_head = sq_tail = 0;      
    push_decim = 0;      
    rtty_dropped_cnt = 0;    
    if (!nco_lut_ready) nco_lut_init();   // однократно — 512 sinf при первом rtty on    
    memset(&rrx, 0, sizeof(rrx));      
    rrx.fs = RTTY_DFS;                 // 3866 — Fs после децимации      
    rrx.symbollen = RTTY_SYMLEN;  
    rrx.nbits = 5;  
    sq_open = (sq_level == 0.0f);    
    ban_lo = ban_hi = 0;               // чистый старт — бан снят    
    scan_init();      
    // RSID отключён: старт сразу со сканера (или TRACK при ручном центре).    
    state = scan_en ? RTTY_ST_SCAN : RTTY_ST_TRACK;    
    if (state == RTTY_ST_TRACK) rtty_tune(RTTY_DEF_CENTER);    
    rtty_active = true;    
    Serial.printf("# rtty on: %s\n",    
                  state == RTTY_ST_SCAN ? "scan" : "track");    
}  
  
void rttydec_push(int16_t x) {          // Core 1, горячий путь      
    if (!rtty_active) return;      
    if (++push_decim < RTTY_DECIM) return;      
    push_decim = 0;      
    if (state == RTTY_ST_SCAN) {      
        sq_put(x);                      // сканер — на Core 0 (poll)      
    } else {      
        rtty_rx_sample((float)x, 0.0f); // декодер — на Core 1, моно: xq=0      
    }      
}      
  
void rttydec_poll(void) {               // Core 0, из loop()        
    if (!rtty_active) return;        
        
    // Сканер на Core 0: дренаж децимированных отсчётов в банк Гёрцеля.      
    // В TRACK очередь не наполняется (push идёт в декодер) — цикл пуст.      
    while (sq_tail != sq_head) {      
        int16_t v = sq[sq_tail];      
        sq_tail = (uint16_t)((sq_tail + 1) & (RTTY_SQ_LEN - 1));      
        if (state == RTTY_ST_SCAN) scan_sample((float)v);      
    }      
        
    // Потеря сигнала в TRACK: 5 с без валидных символов -> перезахват          
    if (state == RTTY_ST_TRACK && millis() - track_lost_ms > 5000) {        
        state = RTTY_ST_SCAN;        
        scan_init();        
        Serial.println("# rtty: lost -> scan");        
    }        
        
    // False lock: 3 с в TRACK, а в каналах mark/space энергии нет —      
    // сканер захватил помеху. Баним пару и назад в сканер.      
    if (state == RTTY_ST_TRACK && millis() - track_lost_ms > 3000) {      
        float env = fmaxf(rrx.mark_env, rrx.space_env);      
        if (env < 2.0f) {        
            ban_lo = rrx.f_space; ban_hi = rrx.f_mark;      
            ban_ms = millis();      
            state = RTTY_ST_SCAN;      
            scan_init();      
            Serial.printf("# rtty: false lock %d/%d -> scan\n",      
                          (int)ban_lo, (int)ban_hi);      
        }      
    }      
      
    // Дренаж очереди символов -> Serial + OLED (только Core 0 трогает их)    
    while (chq_tail != chq_head) {      
        uint8_t b = chq[chq_tail];      
        chq_tail = (uint16_t)((chq_tail + 1) & (RTTY_CHQ_LEN - 1));      
        Serial.write(b);      
        af_oled_putc((char)b);      
    }      
    
    // --- ДИАГНОСТИКА TRACK --- (без изменений)        
    if (dbg_en && state == RTTY_ST_TRACK) {        
        static uint32_t dbg_trk_ms = 0;         
        if (millis() - dbg_trk_ms > 2000) {      
            dbg_trk_ms = millis();      
            float env = fmaxf(rrx.mark_env, rrx.space_env);      
            float snr = (rrx.noise_floor > 1e-6f) ? env / rrx.noise_floor : 0.0f;      
            Serial.printf("# trk: m=%d s=%d nf=%d snr=%d.%d sq=%s v3=%d ferr=%d.%d\n",    
                (int)rrx.mark_env, (int)rrx.space_env,    
                (int)rrx.noise_floor,    
                (int)snr, abs((int)(snr * 10.0f)) % 10,    
                sq_open ? "open" : "shut",    
                (int)rrx.v3,    
                (int)rrx.freqerr, abs((int)(rrx.freqerr * 10.0f)) % 10);    
        }      
    }      
}      
  
void rttydec_stop(void) {      
    rtty_active = false;               // push сразу перестаёт работать      
    chq_head = chq_tail = 0;        
    sq_head = sq_tail = 0;        
}      
  
// ---- настройки ----  
void rttydec_set_scan(bool on)  { scan_en = on; }    
void rttydec_set_rsid(bool on)  { rsid_en = on; }    
void rttydec_set_rev(bool on)   { rtty_rev = on; }    
void rttydec_set_dbg(bool on)   { dbg_en = on; }    
void rttydec_set_sql(float lvl) {  
    if (lvl < 0.0f || lvl > 0.9f) return;  
    sq_level = lvl;  
    if (lvl == 0.0f) sq_open = true;  
} 

void rttydec_set_center(float hz) {    
    if (hz <= 0.0f) {                    // 0 = авто-режим: назад в сканер    
        scan_init();    
        state = RTTY_ST_SCAN;    
        return;    
    }    
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
bool  rttydec_dbg_en(void)   { return dbg_en; }    
float rttydec_sql(void)      { return sq_level; }  
uint32_t rttydec_dropped(void) { return rtty_dropped_cnt; }  
