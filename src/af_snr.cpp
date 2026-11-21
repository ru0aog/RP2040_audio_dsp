// src/af_snr.cpp  
// ============================================================================  
// ---- FFT radix-2 in-place, re/im int16, scaling >>1 на стадию ----    
// >>1 scaling per stage — схема из rsid.cpp RP2040_radiomodem) ->  
// per-bin minima-tracking шума -> Wiener-like gain с over-subtraction ->  
// IFFT (через conjugate) -> overlap-add 50%.  
// Вся runtime-математика целочисленная; float только в af_snr_init()  
// для таблиц окна и twiddles.  
// ============================================================================  
#include <Arduino.h>    
#include <math.h>    
#include <string.h>    
#include "af_snr.h"    
  
#define SNR_N    128  
#define SNR_HOP  64  
#define SNR_BINS (SNR_N/2 + 1)   // 65 
#define SNR_Q    15                     // twiddles/окно/gain — Q15    
  
// ---- Таблицы (init) ----  
static int16_t snr_win[SNR_N];          // Hann Q15  
static int16_t snr_tw_re[SNR_N / 2];    // cos Q15  
static int16_t snr_tw_im[SNR_N / 2];    // -sin Q15  
static uint16_t snr_rev[SNR_N];         // bit-reverse  
  
// ---- Состояние кадра ----    
// int32: бины после FFT/256 с <<8-предусилением достигают ~2^23,    
// int16 переполнялся бы на каждой стадии    
static int16_t snr_re[SNR_N];   // int16 + >>1 на стадию, как fft_i/fft_q в rsid  
static int16_t snr_im[SNR_N];  
static int16_t snr_ola[SNR_HOP];        // выходной буфер кадра (128 отсчётов)  
static int16_t snr_tail[SNR_HOP];       // хвост предыдущего кадра для overlap  
static int16_t snr_in[SNR_N];           // входное кольцо кадра  
static uint16_t snr_in_pos = 0;         // позиция записи во входной кадр  
static uint16_t snr_out_pos = 0;        // позиция чтения из OLA  
static uint16_t snr_out_avail = 0;      // сколько готовых отсчётов в OLA-хвосте  
  
// ---- Per-bin состояние ----  
static uint16_t snr_noise[SNR_BINS];    // оценка шума (minima tracker), mag <= 65535  
static int32_t  snr_gain[SNR_BINS];     // сглаженный gain Q15 (int32: значение 32768 не влезает в int16)    
  
// ---- Параметры ----  
static int32_t snr_depth  = 24576;      // beta Q15 (из nrdepth)  
static int32_t snr_smsh   = 2;          // сдвиг сглаживания gain  
static uint8_t snr_warmup = 0;          // кадры прогрева: gain=1, только сбор шума  
static int32_t snr_makeup = 32768;      // Q15 makeup (из nrmakeup), ×1 по умолчанию  
  
#define SNR_GMIN  3277                  // gain floor 0.1 Q15 — против musical noise    
  
// ---- Слайсер: кадр считается квантами по тайм-бюджету (схема rsid.cpp) ----    
#define SNR_SLICE_US  60                // тайм-квант на один вызов af_snr_push, мкс    
#define SNR_CHUNK     32                // бабочек/бинов за итерацию внутри кванта    
  
typedef enum {    
    SNR_SM_IDLE = 0,    
    SNR_SM_LOAD,                        // окно + <<8 -> re/im    
    SNR_SM_FFT_FWD,                     // прямой FFT по стадиям    
    SNR_SM_BINS,                        // mag -> noise -> gain -> apply    
    SNR_SM_FFT_INV,                     // IFFT по стадиям    
    SNR_SM_OLA                          // окно синтеза + overlap-add    
} snr_sm_t;    
  
static snr_sm_t  snr_sm       = SNR_SM_IDLE;    
static uint16_t  snr_sm_i     = 0;      // универсальный индекс внутри стадии    
static uint16_t  snr_sm_len   = 0;      // текущий len FFT-стадии    
static uint16_t  snr_sm_base  = 0;      // текущий i (база блока) внутри стадии    
static uint16_t  snr_sm_j     = 0;      // текущий j внутри блока    
static bool      snr_sm_inv   = false;  // направление FFT      
static bool      snr_sm_br    = false;  // фаза bit-reverse (иначе j=0 неотличим от маркера)  
static uint32_t  snr_overrun  = 0;      // диагностика: кадр не успел за hop    

// ---- Отладка: бин SNR_DBG_BIN (~1.2 кГц, внутри полосы) ----    
#define SNR_DBG_BIN 10                  // ~1.2 кГц при N=128    
static uint16_t snr_dbg_mag = 0; 

// ---- isqrt uint32 ----  
// ---- isqrt uint32: классический битовый, b — степень 4 ----    
static uint16_t snr_isqrt(uint32_t v) {    
    uint32_t r = 0, b = 1u << 30;         // наибольшая степень 4 <= uint32    
    while (b > v) b >>= 2;    
    while (b) {    
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }    
        else r >>= 1;    
        b >>= 2;    
    }    
    return (uint16_t)r;    
}    
  
// ---- FFT radix-2 in-place, re/im int32, scaling >>1 на стадию ----    
// Пошаговый движок: snr_fft_begin() + snr_fft_step() — одна бабочка за вызов.    
// Состояние (len/base/j) — в snr_sm_*, чтобы стадия прерывалась по кванту.    
  
static void snr_fft_begin(bool inverse) {      
    snr_sm_inv = inverse;      
    snr_sm_br  = true;                  // фаза 0: bit-reverse      
    snr_sm_i   = 0;      
    snr_sm_len = 2;      
    snr_sm_base = 0;      
    snr_sm_j    = 0;      
}      
  
// Возвращает true, когда FFT закончен.    
static bool snr_fft_step(void) {    
    if (snr_sm_len == 0) return true;   // защита    
  
    if (snr_sm_br) {                    // фаза bit-reverse      
        uint16_t i = snr_sm_i;      
        uint16_t j = snr_rev[i];      
        if (j > i) {      
            int32_t t;      
            t = snr_re[i]; snr_re[i] = snr_re[j]; snr_re[j] = t;      
            t = snr_im[i]; snr_im[i] = snr_im[j]; snr_im[j] = t;      
        }      
        if (++snr_sm_i >= SNR_N) {      
            snr_sm_br  = false;         // переход к стадиям бабочек      
            snr_sm_len = 2; snr_sm_base = 0; snr_sm_j = 0;      
        }      
        return false;      
    }      
  
    // Стадии бабочек    
    for (;;) {    
        uint16_t half = snr_sm_len >> 1;    
        uint16_t step = SNR_N / snr_sm_len;    
        uint16_t tw = snr_sm_j * step;    
        int32_t wr = snr_tw_re[tw];    
        int32_t wi = snr_sm_inv ? -snr_tw_im[tw] : snr_tw_im[tw];    
        uint16_t a = snr_sm_base + snr_sm_j, b = a + half;    
        int32_t br = snr_re[b], bi = snr_im[b];    
        int32_t tr = (int32_t)(((int64_t)br * wr - (int64_t)bi * wi) >> SNR_Q);    
        int32_t ti = (int32_t)(((int64_t)br * wi + (int64_t)bi * wr) >> SNR_Q);    
        snr_re[b] = (snr_re[a] - tr) >> 1;    
        snr_im[b] = (snr_im[a] - ti) >> 1;    
        snr_re[a] = (snr_re[a] + tr) >> 1;    
        snr_im[a] = (snr_im[a] + ti) >> 1;    
  
        if (++snr_sm_j >= half) {    
            snr_sm_j = 0;    
            snr_sm_base += snr_sm_len;    
            if (snr_sm_base >= SNR_N) {    
                snr_sm_base = 0;    
                snr_sm_len <<= 1;    
                if (snr_sm_len > SNR_N) return true;   // FFT закончен    
            }    
        }    
        return false;                   // одна бабочка за вызов    
    }    
}    
  
// ---- Стейт-машина кадра: вызывается квантами из af_snr_push ----    
// Каждая стадия делает не более SNR_CHUNK единиц работы за итерацию,    
// внешний цикл ограничен тайм-бюджетом SNR_SLICE_US.    
  
static void snr_sm_service(void) {    
    const uint32_t t0 = micros();    
    do {    
        switch (snr_sm) {    
  
        case SNR_SM_IDLE:    
            return;    
  
        case SNR_SM_LOAD: {             // окно + <<8 -> re/im    
            uint16_t n = 0;    
            for (; snr_sm_i < SNR_N && n < SNR_CHUNK; snr_sm_i++, n++) {    
                int32_t x = snr_in[snr_sm_i];         // int16-тракт: без предусиления  
                snr_re[snr_sm_i] = (int16_t)((x * snr_win[snr_sm_i]) >> SNR_Q);      
                snr_im[snr_sm_i] = 0;    
            }    
            if (snr_sm_i >= SNR_N) { snr_fft_begin(false); snr_sm = SNR_SM_FFT_FWD; }    
            break;    
        }    
  
        case SNR_SM_FFT_FWD: {    
            uint16_t n = 0;    
            while (n < SNR_CHUNK) {    
                if (snr_fft_step()) { snr_sm = SNR_SM_BINS; snr_sm_i = 0; break; }    
                n++;    
            }    
            break;    
        }    
  
        case SNR_SM_BINS: {             // mag -> noise -> gain -> apply    
            uint16_t n = 0;    
            for (; snr_sm_i < SNR_BINS && n < SNR_CHUNK; snr_sm_i++, n++) {    
                uint16_t k = snr_sm_i;    
                int32_t re = snr_re[k], im = snr_im[k];    
                // re,im <= 32767: сумма квадратов <= 2.1e9 — влезает в uint32 без сдвига  
                uint16_t mag = snr_isqrt((uint32_t)(re * re + im * im));      
                if (k == SNR_DBG_BIN) snr_dbg_mag = mag;    
  
                uint32_t nn = snr_noise[k];    
                if (mag < nn) nn = mag;    
                else          nn += (mag - nn) >> 6;    
                snr_noise[k] = nn;    
  
                int32_t g = 32768;    
                if (mag > 0) {    
                    int32_t sub = (int32_t)(((int64_t)snr_depth * nn) / mag);    
                    g = 32768 - sub;    
                }    
                if (g < SNR_GMIN) g = SNR_GMIN;    
                if (g > 32768)    g = 32768;    
  
                int32_t gs;    
                if (snr_warmup > 0) gs = 32768;    
                else gs = snr_gain[k] + ((g - snr_gain[k]) >> snr_smsh);    
                snr_gain[k] = gs;    
  
                snr_re[k] = (int32_t)(((int64_t)re * gs) >> SNR_Q);    
                snr_im[k] = (int32_t)(((int64_t)im * gs) >> SNR_Q);    
                if (k > 0 && k < SNR_N / 2) {    
                    snr_re[SNR_N - k] = snr_re[k];    
                    snr_im[SNR_N - k] = -snr_im[k];    
                }    
            }    
            if (snr_sm_i >= SNR_BINS) { snr_fft_begin(true); snr_sm = SNR_SM_FFT_INV; }    
            break;    
        }    
  
        case SNR_SM_FFT_INV: {    
            uint16_t n = 0;    
            while (n < SNR_CHUNK) {    
                if (snr_fft_step()) { snr_sm = SNR_SM_OLA; snr_sm_i = 0; break; }    
                n++;    
            }    
            break;    
        }    
  
        case SNR_SM_OLA: {              // окно синтеза + overlap-add    
            // Итог тракта: x -> *win -> FFT/128 -> gain -> IFFT/128  
            //   -> *win*(4/3) -> OLA ~= x  
            uint16_t n = 0;    
            for (; snr_sm_i < SNR_N && n < SNR_CHUNK; snr_sm_i++, n++) {    
                uint16_t i = snr_sm_i;    
                // re = x*win/2 после IFFT; re*win*21846>>15 = x*win^2/3; <<1 -> *2/3  
                // сумма win^2 по 50% overlap = 1.5 -> итог ~x (единичное усиление)  
                int32_t y = (int32_t)(((int64_t)snr_re[i] * snr_win[i] * 21846) >> SNR_Q) << 1;           
                if (i < SNR_HOP) {    
                    y = snr_tail[i] + y;    
                    if (y > 32767) y = 32767; else if (y < -32768) y = -32768;    
                    snr_ola[i] = (int16_t)y;    
                } else {    
                    if (y > 32767) y = 32767; else if (y < -32768) y = -32768;    
                    snr_tail[i - SNR_HOP] = (int16_t)y;    
                }    
            }    
            if (snr_sm_i >= SNR_N) {    
                if (snr_warmup > 0) snr_warmup--;    
                snr_out_avail = SNR_HOP;    
                snr_out_pos   = 0;    
                snr_sm = SNR_SM_IDLE;    
            }    
            break;    
        }    
        }    
    } while ((int32_t)(micros() - t0) < SNR_SLICE_US);    
}    
  
void af_snr_init(void) {  
    for (uint16_t i = 0; i < SNR_N; i++) {  
        snr_win[i] = (int16_t)(32767.0f * 0.5f *  
                     (1.0f - cosf(2.0f * (float)M_PI * i / SNR_N)));  
        uint16_t r = 0;    
        for (uint16_t b = 0; b < 7; b++)            // log2(128)=7    
            r |= ((i >> b) & 1) << (6 - b);    
        snr_rev[i] = r;    
    }  
    for (uint16_t i = 0; i < SNR_N / 2; i++) {  
        float a = 2.0f * (float)M_PI * i / SNR_N;  
        snr_tw_re[i] = (int16_t)(32767.0f * cosf(a));  
        snr_tw_im[i] = (int16_t)(-32767.0f * sinf(a));  
    }  
    af_snr_reset();  
}  
  
void af_snr_reset(void) {  
    memset(snr_ola,  0, sizeof(snr_ola));    
    memset(snr_tail, 0, sizeof(snr_tail));    
    memset(snr_in,   0, sizeof(snr_in));    
    memset(snr_re,   0, sizeof(snr_re));    
    memset(snr_im,   0, sizeof(snr_im));    
    for (uint16_t k = 0; k < SNR_BINS; k++) {  
        snr_noise[k] = 0;  
        snr_gain[k]  = 32768;  
    }  
    snr_in_pos = snr_out_pos = snr_out_avail = 0;    
    snr_sm = SNR_SM_IDLE;      
    snr_sm_i = snr_sm_len = snr_sm_base = snr_sm_j = 0;      
    snr_sm_br = false;      
    snr_overrun = 0;    
    snr_warmup = 8;                     // ~66 мс прозрачного вывода на сбор шума    
}  
  
int32_t af_snr_push(int32_t x) {    
    // Выход: читаем из OLA-буфера; пока не накоплен — нули (latency fill)    
    int32_t y = 0;    
    if (snr_out_avail > 0) {    
        y = snr_ola[snr_out_pos++];    
        snr_out_avail--;    
    }    
    // Вход: пишем в кадр; по заполнении HOP — обработка    
    snr_in[snr_in_pos++] = (int16_t)x;      
    if (snr_in_pos >= SNR_N) snr_in_pos = 0;    
    if (snr_in_pos == 0) {      
        // кадр готов: in[] содержит последние 256 отсчётов.    
        // Если SM ещё занята предыдущим кадром — overrun: кадр пропускаем,    
        // но входное кольцо всё равно сдвигаем (дырка в выходе, не рассинхрон).    
        if (snr_sm == SNR_SM_IDLE) {    
            snr_sm   = SNR_SM_LOAD;    
            snr_sm_i = 0;    
        } else {    
            snr_overrun++;    
        }    
        memmove(&snr_in[0], &snr_in[SNR_HOP], SNR_HOP * sizeof(int16_t));    
        snr_in_pos = SNR_HOP;    
    }    
    static uint8_t svc_div = 0;  
    if (++svc_div >= 4) { svc_div = 0; snr_sm_service(); }  
    // makeup    
    y = (int32_t)(((int64_t)y * snr_makeup) >> SNR_Q);  
    if (y > 32767) y = 32767; else if (y < -32768) y = -32768;  
    return y;  
}  
  
void af_snr_set_depth(int32_t q15)  { snr_depth  = q15; }  
void af_snr_set_smooth(int32_t v)   { snr_smsh   = (v < 1) ? 1 : (v > 6 ? 6 : v); }  
void af_snr_set_makeup(int32_t q15) { snr_makeup = q15; }  

uint32_t af_snr_dbg_mag(void)   { return snr_dbg_mag; }  
uint32_t af_snr_dbg_noise(void) { return snr_noise[SNR_DBG_BIN]; }  
int32_t  af_snr_dbg_gain(void)  { return snr_gain[SNR_DBG_BIN]; }  

uint32_t af_snr_dbg_overrun(void) { return snr_overrun; }  
