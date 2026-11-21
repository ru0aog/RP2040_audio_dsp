// src/af_snr.h  
// Спектральный шумодав (Core 1): FFT-256 int16, overlap-add 50%,  
// оценка шума по минимумам, over-subtraction, сглаживание gain.  
#pragma once  
#include <Arduino.h>  
  
void    af_snr_init(void);  
int32_t af_snr_push(int32_t x);        // int32 in -> int32 out, задержка ~16 мс  
void    af_snr_reset(void);  
void    af_snr_set_depth(int32_t q15); // over-subtraction beta, 0..32768  
void    af_snr_set_smooth(int32_t v);  // скорость сглаживания gain (сдвиг 1..6)  
void    af_snr_set_makeup(int32_t q15);// выходной makeup, 0..131072  
