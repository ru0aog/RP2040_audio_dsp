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
  
#define SNR_N    256                    // FFT size  
#define SNR_HOP  128                    // 50% overlap  
#define SNR_BINS (SNR_N / 2 + 1)        // 129 бинов, шаг ~60.4 Гц @15464  
#define SNR_Q    15  
  
// ---- Таблицы (init) ----  
static int16_t snr_win[SNR_N];          // Hann Q15  
static int16_t snr_tw_re[SNR_N / 2];    // cos Q15  
static int16_t snr_tw_im[SNR_N / 2];    // -sin Q15  
static uint16_t snr_rev[SNR_N];         // bit-reverse  
  
// ---- Состояние кадра ----    
// int32: бины после FFT/256 с <<8-предусилением достигают ~2^23,    
// int16 переполнялся бы на каждой стадии    
static int32_t snr_re[SNR_N];    
static int32_t snr_im[SNR_N];    
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

// ---- Отладка: бин SNR_DBG_BIN (~1.2 кГц, внутри полосы) ----    
#define SNR_DBG_BIN 20    
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
  
// ---- FFT radix-2 in-place, re/im int16, scaling >>1 на стадию ----  
static void snr_fft(int32_t *re, int32_t *im, bool inverse) {    
    for (uint16_t i = 0; i < SNR_N; i++) {          // bit-reverse    
        uint16_t j = snr_rev[i];    
        if (j > i) {    
            int32_t t;    
            t = re[i]; re[i] = re[j]; re[j] = t;    
            t = im[i]; im[i] = im[j]; im[j] = t;    
        }    
    }    
    for (uint16_t len = 2; len <= SNR_N; len <<= 1) {    
        uint16_t half = len >> 1;    
        uint16_t step = SNR_N / len;    
        for (uint16_t i = 0; i < SNR_N; i += len) {    
            for (uint16_t j = 0; j < half; j++) {    
                uint16_t tw = j * step;    
                int32_t wr = snr_tw_re[tw];    
                int32_t wi = inverse ? -snr_tw_im[tw] : snr_tw_im[tw];    
                uint16_t a = i + j, b = a + half;    
                // Бины ~2^23: >>8 -> ~2^15, тогда *Q15 = ~2^30 влезает в int32.  
                // (br>>8)*wr >> 7 == br*wr >> 15 — масштаб сохранён.  
                int32_t br = re[b] >> 8, bi = im[b] >> 8;  
                int32_t tr = (br * wr - bi * wi) >> 7;  
                int32_t ti = (br * wi + bi * wr) >> 7;      
                re[b] = (re[a] - tr) >> 1;    
                im[b] = (im[a] - ti) >> 1;    
                re[a] = (re[a] + tr) >> 1;    
                im[a] = (im[a] + ti) >> 1;    
            }    
        }    
    }    
}    
  
// ---- Обработка одного кадра ----    
static void snr_frame(void) {    
    // Окно + предусиление <<8: компенсация /256 прямого FFT.      
    // Бины получаются ~x*256*win — в int32 помещаются с запасом.    
    for (uint16_t i = 0; i < SNR_N; i++) {    
        int32_t x = snr_in[i] << 8;      
        snr_re[i] = (int32_t)(((int64_t)x * snr_win[i]) >> SNR_Q);      
        snr_im[i] = 0;    
    }    
    snr_fft(snr_re, snr_im, false);  
  
    // Per-bin: magnitude -> minima tracker -> gain -> apply  
    for (uint16_t k = 0; k < SNR_BINS; k++) {  
        int32_t re = snr_re[k], im = snr_im[k];      
        // re,im ~2^23 -> квадраты ~2^46: int64 + >>16 -> ~2^30 в uint32,      
        // isqrt -> mag ~2^15 (uint16)      
        uint16_t mag = snr_isqrt((uint32_t)(        
            ((int64_t)re * re + (int64_t)im * im) >> 16));        
        if (k == SNR_DBG_BIN) snr_dbg_mag = mag;   // отладка: mag бина ~1.2 кГц  
      
        uint32_t n = snr_noise[k];      
        if (mag < n) n = mag;                       // быстро вниз    
        else         n += (mag - n) >> 6;           // медленно вверх ~1с    
        snr_noise[k] = n;    
  
        // gain = 1 - depth * n/mag, floor GMIN  
        int32_t g = 32768;  
        if (mag > 0) {  
            int32_t sub = (int32_t)(((int64_t)snr_depth * n) / mag);  
            g = 32768 - sub;  
        }  
        if (g < SNR_GMIN) g = SNR_GMIN;  
        if (g > 32768)    g = 32768;  
  
        int32_t gs;  
        if (snr_warmup > 0) {  
            gs = 32768;                 // прогрев: прозрачно, только сбор шумовой оценки  
        } else {  
            gs = snr_gain[k] + ((g - snr_gain[k]) >> snr_smsh);  
        }  
        snr_gain[k] = gs;      
  
        snr_re[k] = (int32_t)(((int64_t)re * gs) >> SNR_Q);      
        snr_im[k] = (int32_t)(((int64_t)im * gs) >> SNR_Q);      
        if (k > 0 && k < SNR_N / 2) {               // conjugate symmetric  
            snr_re[SNR_N - k] = snr_re[k];  
            snr_im[SNR_N - k] = -snr_im[k];  
        }  
    }  
  
    snr_fft(snr_re, snr_im, true);                  // IFFT (scaling тот же)  
  
    // OLA: окно синтеза + компенсация hann^2 (~1.5) -> *2/3.  
    // Первая половина кадра складывается с хвостом предыдущего -> на выход;  
    // вторая половина уходит в snr_tail для следующего кадра.  
    // IFFT/256 даёт 256*g*x*win (вход был <<8): нужен >>8 на выходе.  
    // Итог тракта:  
    // x -> <<8 -> FFT/256 (=DFT(x)) -> gain -> IFFT/256 (=256*x*win)  
    //   -> *win -> >>8 -> OLA*(2/3) ~= x  
    for (uint16_t i = 0; i < SNR_HOP; i++) {        
        int32_t y = (int32_t)(((int64_t)snr_re[i] * snr_win[i]) >> (SNR_Q + 8));          
        y = snr_tail[i] + (int32_t)(((int64_t)y * 21846) >> SNR_Q);          
        if (y > 32767) y = 32767; else if (y < -32768) y = -32768;        
        snr_ola[i] = (int16_t)y;        
    }        
    for (uint16_t i = 0; i < SNR_HOP; i++) {        
        int32_t y = (int32_t)(((int64_t)snr_re[SNR_HOP + i] * snr_win[SNR_HOP + i]) >> (SNR_Q + 8));          
        y = (int32_t)(((int64_t)y * 21846) >> SNR_Q);          
        if (y > 32767) y = 32767; else if (y < -32768) y = -32768;        
        snr_tail[i] = (int16_t)y;        
    }        
    if (snr_warmup > 0) snr_warmup--;  
    snr_out_avail = SNR_HOP;    
    snr_out_pos   = 0;    
}  
  
void af_snr_init(void) {  
    for (uint16_t i = 0; i < SNR_N; i++) {  
        snr_win[i] = (int16_t)(32767.0f * 0.5f *  
                     (1.0f - cosf(2.0f * (float)M_PI * i / SNR_N)));  
        uint16_t r = 0;  
        for (uint16_t b = 0; b < 8; b++)            // log2(256)=8  
            r |= ((i >> b) & 1) << (7 - b);  
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
        // Первый кадр — после полного заполнения (256 отсчётов),    
        // далее каждые 128: после memmove pos=128 -> wrap через HOP.    
        snr_frame();    
        memmove(&snr_in[0], &snr_in[SNR_HOP], SNR_HOP * sizeof(int16_t));    
        snr_in_pos = SNR_HOP;    
    }    
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

