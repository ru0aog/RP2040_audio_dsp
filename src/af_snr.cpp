// src/af_snr.cpp  
// ============================================================================  
// Спектральный NR: Hann-окно -> FFT-256 (int16, radix-2, Q15 twiddles,  
// >>1 scaling per stage — схема из rsid.cpp RP2040_radiomodem) ->  
// per-bin minima-tracking шума -> Wiener-like gain с over-subtraction ->  
// IFFT (через conjugate) -> overlap-add 50%.  
// Вся runtime-математика целочисленная; float только в af_snr_init()  
// для таблиц окна и twiddles.  
// ============================================================================  
#include <Arduino.h>  
#include <math.h>  
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
static int16_t snr_re[SNR_N];  
static int16_t snr_im[SNR_N];  
static int16_t snr_ola[SNR_N];          // аккумулятор overlap-add  
static int16_t snr_in[SNR_N];           // входное кольцо кадра  
static uint16_t snr_in_pos = 0;         // позиция записи во входной кадр  
static uint16_t snr_out_pos = 0;        // позиция чтения из OLA  
static uint16_t snr_out_avail = 0;      // сколько готовых отсчётов в OLA-хвосте  
  
// ---- Per-bin состояние ----  
static uint16_t snr_noise[SNR_BINS];    // оценка шума (minima tracker)  
static int16_t  snr_gain[SNR_BINS];     // сглаженный gain Q15  
  
// ---- Параметры ----  
static int32_t snr_depth  = 24576;      // beta Q15 (из nrdepth)  
static int32_t snr_smsh   = 2;          // сдвиг сглаживания gain  
static int32_t snr_makeup = 65536;      // Q15 makeup (из nrmakeup)  
  
#define SNR_GMIN  3277                  // gain floor 0.1 Q15 — против musical noise  
  
// ---- isqrt uint32 ----  
static uint16_t snr_isqrt(uint32_t v) {  
    uint32_t r = 0, b = 0x8000;  
    while (b > v) b >>= 2;  
    while (b) {  
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }  
        else r >>= 1;  
        b >>= 2;  
    }  
    return (uint16_t)r;  
}  
  
// ---- FFT radix-2 in-place, re/im int16, scaling >>1 на стадию ----  
static void snr_fft(int16_t *re, int16_t *im, bool inverse) {  
    for (uint16_t i = 0; i < SNR_N; i++) {          // bit-reverse  
        uint16_t j = snr_rev[i];  
        if (j > i) {  
            int16_t t;  
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
                int32_t br = re[b], bi = im[b];  
                int32_t tr = (br * wr - bi * wi) >> SNR_Q;  
                int32_t ti = (br * wi + bi * wr) >> SNR_Q;  
                re[b] = (int16_t)((re[a] - tr) >> 1);  
                im[b] = (int16_t)((im[a] - ti) >> 1);  
                re[a] = (int16_t)((re[a] + tr) >> 1);  
                im[a] = (int16_t)((im[a] + ti) >> 1);  
            }  
        }  
    }  
}  
  
// ---- Обработка одного кадра ----  
static void snr_frame(void) {  
    // Окно + предусиление <<4 (компенсация >>1 x 8 стадий = /256)  
    for (uint16_t i = 0; i < SNR_N; i++) {  
        int32_t x = snr_in[i] << 4;  
        snr_re[i] = (int16_t)((x * snr_win[i]) >> SNR_Q);  
        snr_im[i] = 0;  
    }  
    snr_fft(snr_re, snr_im, false);  
  
    // Per-bin: magnitude -> minima tracker -> gain -> apply  
    for (uint16_t k = 0; k < SNR_BINS; k++) {  
        int32_t re = snr_re[k], im = snr_im[k];  
        uint16_t mag = snr_isqrt((uint32_t)(re * re + im * im));  
  
        uint16_t n = snr_noise[k];  
        if (mag < n) n = mag;                       // быстро вниз  
        else         n += (uint16_t)((mag - n) >> 6); // медленно вверх ~1с  
        snr_noise[k] = n;  
  
        // gain = 1 - depth * n/mag, floor GMIN  
        int32_t g = 32768;  
        if (mag > 0) {  
            int32_t sub = (int32_t)(((int64_t)snr_depth * n) / mag);  
            g = 32768 - sub;  
        }  
        if (g < SNR_GMIN) g = SNR_GMIN;  
        if (g > 32768)    g = 32768;  
  
        int32_t gs = snr_gain[k] + ((g - snr_gain[k]) >> snr_smsh);  
        snr_gain[k] = (int16_t)gs;  
  
        snr_re[k] = (int16_t)((re * gs) >> SNR_Q);  
        snr_im[k] = (int16_t)((im * gs) >> SNR_Q);  
        if (k > 0 && k < SNR_N / 2) {               // conjugate symmetric  
            snr_re[SNR_N - k] = snr_re[k];  
            snr_im[SNR_N - k] = -snr_im[k];  
        }  
    }  
  
    snr_fft(snr_re, snr_im, true);                  // IFFT (scaling тот же)  
  
    // OLA: окно синтеза + компенсация hann^2 суммы (~1.5) -> *2/3  
    for (uint16_t i = 0; i < SNR_N; i++) {  
        int32_t y = (snr_re[i] * snr_win[i]) >> SNR_Q;  
        y = (snr_ola[i] + ((y * 21846) >> SNR_Q));  // 21846 = 2/3 Q15  
        snr_ola[i] = (int16_t)y;  
    }  
    snr_out_avail = SNR_HOP;  
    snr_out_pos   = 0;  
}  
  
void af_snr_init(void) {  
    for (uint16_t i = 0; i < SNR_N; i++) {  
        snr_win[i] = (int16_t)(32767.0f * 0.5f *  
                     (1.0f - cosf(2.0f * (float)M_PI * i / (SNR_N - 1))));  
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
    memset(snr_ola, 0, sizeof(snr_ola));  
    memset(snr_in,  0, sizeof(snr_in));  
    memset(snr_re,  0, sizeof(snr_re));  
    memset(snr_im,  0, sizeof(snr_im));  
    for (uint16_t k = 0; k < SNR_BINS; k++) {  
        snr_noise[k] = 0;  
        snr_gain[k]  = 32768;  
    }  
    snr_in_pos = snr_out_pos = snr_out_avail = 0;  
}  
  
int32_t af_snr_push(int32_t x) {  
    // Выход: читаем из OLA-хвоста; пока не накоплен — нули (latency fill)  
    int32_t y = 0;  
    if (snr_out_avail > 0) {  
        y = snr_ola[snr_out_pos];  
        snr_ola[snr_out_pos] = 0;                   // освобождаем для след. кадра  
        snr_out_pos++;  
        snr_out_avail--;  
    }  
    // Вход: пишем в кадр; по заполнении HOP — обработка  
    snr_in[snr_in_pos++] = (int16_t)x;  
    if (snr_in_pos >= SNR_N) snr_in_pos = 0;  
    if (snr_in_pos == SNR_HOP || snr_in_pos == 0) {  
        // кадр готов: in[] содержит последние 256 отсчётов  
        // (кольцо уже в правильном порядке: старшие 128 + новые 128)  
        snr_frame();  
        // сдвиг окна: переносим вторую половину в начало  
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
