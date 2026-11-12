// src/af_cal.cpp  
// ============================================================================  
// Универсальная коррекция ADC RP2040 (одинаковая для всех чипов).  
//  
// Таблица rp2040adc_err[] — измеренная INL/DNL-поправка в единицах LSB  
// (источник: github.com/kitanokitsune/rp2040adc_correction). Покрывает  
// систематические дефекты RP2040 ADC: DNL-пики на 512/1536/2560/3584  
// и наклон/волну передаточной кривой (-15..+14 LSB по шкале).  
//  
// Применение: corrected = code + err[code], кламп 0..4095.  
// Таблица int8_t[4096] = 4 КБ во flash (.rodata), RAM не тратится.  
// ============================================================================  
#include "af_cal.h"  
#include "af_config.h"  
#include "pico/platform.h"   // __not_in_flash_func  
#include "af_cal_lut.h"      // rp2040adc_err[4096]  
  
static volatile bool cal_on = true;  
  
void af_cal_set_enabled(bool on) { cal_on = on; }  
bool af_cal_is_enabled(void)     { return cal_on; }  
  
int32_t __not_in_flash_func(af_cal_apply)(int32_t code) {  
    if (!cal_on) return code;  
    int32_t c = code + (int32_t)rp2040adc_err[code & 4095];  
    if (c < 0)    c = 0;  
    if (c > 4095) c = 4095;  
    return c;  
}  