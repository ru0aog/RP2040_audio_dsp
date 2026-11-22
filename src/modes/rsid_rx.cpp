// src/modes/rsid_rx.cpp — RSID-детектор, порт rx_rsid.cpp (RP2040_receiver)  
// под Core 0 и вход 15464 Гц. Детектируем только код 39 (RTTY-45) —  
// таблица из одного кода, поиск быстрый. FFT-2048 Q15, окно 512 @11025.  
#include "rsid_rx.h"  
#include <math.h>  
#include <string.h>  
  
#define RSID_FS_IN     15464.0f  
#define RSID_FS        11025.0f  
#define RSID_STEP      (RSID_FS / RSID_FS_IN)   // 0.7128  
#define RSID_FFT_N     2048  
#define RSID_FFT_HALF  1024  
#define RSID_FFT_SAMPLES 512  
#define RSID_NSYMBOLS  15  
#define RSID_NTIMES    30  
#define RSID_BIN_LOW   ((int)(300.0  * 2048.0 / RSID_FS))            // 55  
#define RSID_BIN_HIGH  ((int)(3000.0 * 2048.0 / RSID_FS) + RSID_NTIMES)  
#define RSID_CHUNK     256  
#define RSID_HAMMING_RES 2  
  
// --- Reed-Solomon кодер (verbatim из rx_rsid.cpp) ---  
static const int Squares[256] = {  
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  
    0,2,4,6,8,10,12,14,9,11,13,15,1,3,5,7,  
    0,3,6,5,12,15,10,9,1,2,7,4,13,14,11,8,  
    0,4,8,12,9,13,1,5,11,15,3,7,2,6,10,14,  
    0,5,10,15,13,8,7,2,3,6,9,12,14,11,4,1,  
    0,6,12,10,1,7,13,11,2,4,14,8,3,5,15,9,  
    0,7,14,9,5,2,11,12,10,13,4,3,15,8,1,6,  
    0,8,9,1,11,3,2,10,15,7,6,14,4,12,13,5,  
    0,9,11,2,15,6,4,13,7,14,12,5,8,1,3,10,  
    0,10,13,7,3,9,14,4,6,12,11,1,5,15,8,2,  
    0,11,15,4,7,12,8,3,14,5,1,10,9,2,6,13,  
    0,12,1,13,2,14,3,15,4,8,5,9,6,10,7,11,  
    0,13,3,14,6,11,5,8,12,1,15,2,10,7,9,4,  
    0,14,5,11,10,4,15,1,13,3,8,6,7,9,2,12,  
    0,15,7,8,14,1,9,6,5,10,2,13,11,4,12,3  
};  
static const int indices[12] = { 2,4,8,9,11,15,7,14,5,10,13,3 };  
  
static void rsid_encode(int code, unsigned char* rsid) {  
    rsid[0] = code >> 8;  
    rsid[1] = (code >> 4) & 0x0f;  
    rsid[2] = code & 0x0f;  
    for (int i = 3; i < RSID_NSYMBOLS; i++) rsid[i] = 0;  
    for (int i = 0; i < 12; i++) {  
        for (int j = RSID_NSYMBOLS - 1; j > 0; j--)  
            rsid[j] = rsid[j-1] ^ Squares[(rsid[j] << 4) + indices[i]];  
        rsid[0] = Squares[(rsid[0] << 4) + indices[i]];  
    }  
}  
  
// --- буферы ---  
static int16_t  aInput[RSID_FFT_N + RSID_FFT_SAMPLES];  // скользящее окно  
static int      inptr;  
static int16_t  window_q15[RSID_FFT_N];  
static int16_t  tw_re[RSID_FFT_HALF], tw_im[RSID_FFT_HALF];  
static int16_t  fft_i[RSID_FFT_N], fft_q[RSID_FFT_N];  
static uint64_t aFFTAmpl[RSID_BIN_HIGH];  
static int16_t  fft_buckets[RSID_NTIMES][RSID_BIN_HIGH];  
static unsigned char pCode_rtty[RSID_NSYMBOLS];   // код 39 = RTTY-45  
  
// ресемплер 15464 -> 11025 (линейный)  
static float rs_pos = 0.0f;  
static float rs_prev = 0.0f;  
static bool  rs_have_prev = false;  
  
// результат  
static float         rtty_center_hz = 0.0f;  
static volatile bool rtty_center_new = false;  
static bool          rsid_active = false;  
  
// стейт-машина (квантованная, как в rx_rsid.cpp)  
enum { ST_IDLE, ST_WIN, ST_BR, ST_FFT, ST_AMPL, ST_BUCKETS, ST_SEARCH };  
static int st = ST_IDLE;  
static uint16_t w_i, br_i, br_j, fft_len, fft_base, fft_j2;  
static int ampl_i, bkt_phase, bkt_row, bkt_i, bkt_imax;  
static uint64_t bkt_ampmax;  
static bool win_ready = false;  
  
static inline int16_t sat16(int32_t x) {  
    return x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;  
}  
  
void rsid_rx_init(void) {  
    for (int i = 0; i < RSID_FFT_N; i++)  
        window_q15[i] = sat16((int32_t)(  
            (0.54 - 0.46 * cos(2.0 * M_PI * i / RSID_FFT_N)) * 32767.0));  
    for (int k = 0; k < RSID_FFT_HALF; k++) {  
        double a = -6.283185307179586 * k / RSID_FFT_N;  
        tw_re[k] = sat16((int32_t)(cos(a) * 32767.0));  
        tw_im[k] = sat16((int32_t)(sin(a) * 32767.0));  
    }  
    rsid_encode(39, pCode_rtty);              // RTTY-45  
    memset(aInput, 0, sizeof(aInput));  
    memset(fft_buckets, 0, sizeof(fft_buckets));  
    inptr = RSID_FFT_N;                        // первое окно — после 512 отсчётов  
    rs_pos = 0.0f; rs_have_prev = false;  
    rtty_center_new = false;  
    st = ST_IDLE; win_ready = false;  
    rsid_active = true;  
}  
  
void rsid_rx_stop(void) { rsid_active = false; }  
  
// Вход 15464 Гц -> линейный ресемплинг -> накопитель окон 11025 Гц.  
void rsid_rx_push15464(int16_t s) {  
    if (!rsid_active) return;  
    float cur = (float)s;  
    if (!rs_have_prev) { rs_prev = cur; rs_have_prev = true; return; }  
    // выдаём отсчёты 11025, пока фаза внутри [prev, cur)  
    while (rs_pos < 1.0f) {  
        float y = rs_prev + (cur - rs_prev) * rs_pos;  
        aInput[inptr++] = (int16_t)y;  
        if (inptr >= RSID_FFT_N + RSID_FFT_SAMPLES) {  
            win_ready = true;                    // полное окно для FFT  
            memmove(&aInput[0], &aInput[RSID_FFT_SAMPLES],  
                    RSID_FFT_N * sizeof(int16_t));  
            inptr = RSID_FFT_N;  
        }  
        rs_pos += RSID_STEP;  
    }  
    rs_pos -= 1.0f;  
    rs_prev = cur;  
}  
  
bool rsid_get_rtty_center(float* f_hz) {  
    if (!rtty_center_new) return false;  
    rtty_center_new = false;  
    if (f_hz) *f_hz = rtty_center_hz;  
    return true;  
}  
  
// HammingDistance по бакетам (как rx_rsid.cpp)  
static int hamming_dist(int iBucket) {  
    int dist = 0;  
    for (int i = 0, j = 1; i < RSID_NSYMBOLS; i++, j += 2) {  
        if (fft_buckets[j][iBucket] != pCode_rtty[i])  
            if (++dist > RSID_HAMMING_RES) break;  
    }  
    return dist;  
}  
  
// Один квант обработки; вызывать из rttydec_poll (Core 0).  
void rsid_rx_poll(void) {  
    if (!rsid_active) return;  
    const uint32_t t0 = micros();  
  
    do {  
        switch (st) {  
        case ST_IDLE:  
            if (win_ready) {  
                win_ready = false;  
                w_i = 0; br_i = 1; br_j = 0; fft_j2 = 0;  
                ampl_i = RSID_BIN_LOW - 2;  
                bkt_phase = 0; bkt_row = 0;  
                st = ST_WIN;  
            }  
            break;  
  
        case ST_WIN:  
            for (int n = 0; n < RSID_CHUNK && w_i < RSID_FFT_N; n++, w_i++)  
                fft_i[w_i] = (int16_t)(((int32_t)aInput[w_i] * window_q15[w_i]) >> 15),  
                fft_q[w_i] = 0;  
            if (w_i >= RSID_FFT_N) st = ST_BR;  
            break;  
  
        case ST_BR: {  
            int n = 0;  
            for (; br_i < RSID_FFT_N && n < RSID_CHUNK; ++br_i, ++n) {  
                uint16_t bit = RSID_FFT_HALF;  
                while (br_j & bit) { br_j ^= bit; bit >>= 1; }  
                br_j ^= bit;  
                if (br_i < br_j) {  
                    int16_t t;  
                    t = fft_i[br_i]; fft_i[br_i] = fft_i[br_j]; fft_i[br_j] = t;  
                    t = fft_q[br_i]; fft_q[br_i] = fft_q[br_j]; fft_q[br_j] = t;  
                }  
            }  
            if (br_i >= RSID_FFT_N) { fft_len = 2; fft_base = 0; st = ST_FFT; }  
            break;  
        }  
  
        case ST_FFT: {  
            const uint16_t half = fft_len >> 1;  
            const uint16_t tw_step = RSID_FFT_N / fft_len;  
            for (uint16_t n = 0; n < RSID_CHUNK && fft_j2 < half; ++fft_j2, ++n) {  
                const uint16_t tw = fft_j2 * tw_step;  
                const int32_t wr = tw_re[tw], wi = tw_im[tw];  
                const uint16_t a = fft_base + fft_j2, b = a + half;  
                const int32_t br = fft_i[b], bi = fft_q[b];  
                const int32_t tr = (br * wr - bi * wi) >> 15;  
                const int32_t ti = (br * wi + bi * wr) >> 15;  
                const int32_t ar = fft_i[a], ai = fft_q[a];  
                fft_i[a] = sat16((ar + tr) >> 1);  
                fft_q[a] = sat16((ai + ti) >> 1);  
                fft_i[b] = sat16((ar - tr) >> 1);  
                fft_q[b] = sat16((ai - ti) >> 1);  
            }  
            if (fft_j2 >= half) {  
                fft_j2 = 0; fft_base += fft_len;  
                if (fft_base >= RSID_FFT_N) {  
                    fft_base = 0; fft_len <<= 1;  
                    if (fft_len > RSID_FFT_N) st = ST_AMPL;  
                }  
            }  
            break;  
        }  
  
        case ST_AMPL:  
            for (int n = 0; n < RSID_CHUNK && ampl_i < RSID_BIN_HIGH; n++, ampl_i++) {  
                int32_t re = fft_i[ampl_i], im = fft_q[ampl_i];  
                aFFTAmpl[ampl_i] = (uint64_t)((int64_t)re*re + (int64_t)im*im);  
            }  
            if (ampl_i >= RSID_BIN_HIGH) st = ST_BUCKETS;  
            break;  
  
        case ST_BUCKETS: {  
            const int bw   = RSID_BIN_HIGH - RSID_BIN_LOW;  
            const int iEnd = RSID_BIN_HIGH - RSID_NTIMES;  
            switch (bkt_phase) {  
            case 0: {  
                int n = 0;  
                for (; bkt_row < RSID_NTIMES - 1 && n < 8; bkt_row++, n++)  
                    memmove(&fft_buckets[bkt_row][RSID_BIN_LOW],  
                            &fft_buckets[bkt_row + 1][RSID_BIN_LOW],  
                            bw * sizeof(int16_t));  
                if (bkt_row >= RSID_NTIMES - 1) bkt_phase = 1;  
                break;  
            }  
            case 1:  
                memset(&fft_buckets[RSID_NTIMES - 1][RSID_BIN_LOW], 0,  
                       bw * sizeof(int16_t));  
                bkt_i = RSID_BIN_LOW; bkt_imax = RSID_BIN_LOW - 2; bkt_ampmax = 0;  
                bkt_phase = 2;  
                break;  
            case 2: case 3: {   // чётные / нечётные бины  
                int n = 0;  
                for (; bkt_i < iEnd && n < RSID_CHUNK; bkt_i += 2, n++) {  
                    const int i = bkt_i;  
                    if (bkt_imax == i - 2) {  
                        bkt_ampmax = aFFTAmpl[i]; bkt_imax = i;  
                        for (int j = i + 2; j < i + RSID_NTIMES + 2; j += 2)  
                            if (aFFTAmpl[j] > bkt_ampmax) {  
                                bkt_ampmax = aFFTAmpl[j]; bkt_imax = j;  
                            }  
                    } else {  
                        const int j = i + RSID_NTIMES;  
                        if (aFFTAmpl[j] > bkt_ampmax) {  
                            bkt_ampmax = aFFTAmpl[j]; bkt_imax = j;  
                        }  
                    }  
                    fft_buckets[RSID_NTIMES - 1][i] = (int16_t)((bkt_imax - i) >> 1);  
                }  
                if (bkt_i >= iEnd) {  
                    if (bkt_phase == 2) {  
                        bkt_i = RSID_BIN_LOW + 1; bkt_imax = RSID_BIN_LOW - 1;  
                        bkt_ampmax = 0; bkt_phase = 3;  
                    } else st = ST_SEARCH;  
                }  
                break;  
            }  
            }  
            break;  
        }  
  
        case ST_SEARCH: {  
            int best_dist = 1000, best_bin = -1;  
            for (int j = RSID_BIN_LOW; j < RSID_BIN_HIGH - RSID_NTIMES; j++) {  
                int d = hamming_dist(j);  
                if (d < best_dist) { best_dist = d; best_bin = j; }  
                if (d == 0) break;  
            }  
            if (best_dist <= RSID_HAMMING_RES) {  
                rtty_center_hz = (float)((best_bin + RSID_NSYMBOLS - 0.5)  
                                 * RSID_FS / 2048.0);  
                __dmb();  
                rtty_center_new = true;  
                memset(fft_buckets, 0, sizeof(fft_buckets));  
            }  
            st = ST_IDLE;  
            break;  
        }  
        }  
    } while ((uint32_t)(micros() - t0) < 400);   // бюджет 400 мкс на Core 0  
}  
