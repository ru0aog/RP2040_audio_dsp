// src/modes/tt_gen.cpp — двухтональный генератор для IMD-теста TX-тракта.  
// Два NCO (как cwtx), сумма тонов ~0.4 FS. Частоты должны лежать внутри  
// полосы TT-FIR 400..3200 Гц — иначе фильтр их срезает.  
#include <Arduino.h>  
#include "tt_gen.h"  
#include "../af_config.h"  
  
#define TT_LUT_LEN   256  
#define TT_AMP       13000    // сумма двух тонов ~0.4 FS — запас под gain каскада  
  
static int16_t  tt_lut[TT_LUT_LEN];  
static volatile bool     tt_on = false;  
static volatile uint32_t tt_step1, tt_step2;  
static uint32_t tt_ph1, tt_ph2;  
  
void tt_gen_init(void) {  
    for (int i = 0; i < TT_LUT_LEN; i++)  
        tt_lut[i] = (int16_t)lroundf(sinf(2.0f * PI * i / TT_LUT_LEN) * 32767.0f);  
    tt_gen_set(700, 1900);  
}  
  
void tt_gen_set(uint32_t f1, uint32_t f2) {  
    // кламп в полосу FIR — вне 400..3200 тест бессмыслен  
    if (f1 < TX_TT_LO_HZ) f1 = TX_TT_LO_HZ;  
    if (f2 < TX_TT_LO_HZ) f2 = TX_TT_LO_HZ;  
    if (f1 > TX_TT_HI_HZ) f1 = TX_TT_HI_HZ;  
    if (f2 > TX_TT_HI_HZ) f2 = TX_TT_HI_HZ;  
    tt_step1 = (uint32_t)(((uint64_t)f1 << 32) / AF_FS_OUT_HZ);  
    tt_step2 = (uint32_t)(((uint64_t)f2 << 32) / AF_FS_OUT_HZ);  
}  
  
void tt_gen_on(bool on)  { tt_on = on; if (!on) { tt_ph1 = tt_ph2 = 0; } }  
bool tt_gen_busy(void)   { return tt_on; }  
  
int16_t __not_in_flash_func(tt_gen_sample)(void) {  
    if (!tt_on) return 0;  
    tt_ph1 += tt_step1;  
    tt_ph2 += tt_step2;  
    int32_t t = tt_lut[(tt_ph1 >> 24) & (TT_LUT_LEN - 1)]  
              + tt_lut[(tt_ph2 >> 24) & (TT_LUT_LEN - 1)];  
    return (int16_t)((t * TT_AMP) >> 16);   // каждый тон ~0.2 FS  
}  
