// src/af_pwm.h    
//    
// Основной выход аудио: PWM на GPIO + RC-фильтр.    
// PWM-слайс свободно бежит на несущей ~125 кГц; DMA обновляет duty    
// с темпом AF_PWM_FS_HZ (~61.9 ksps = AF_FS_OUT_HZ × AF_PWM_INTERP),    
// paced аппаратным таймером. Входные блоки 15.464 ksps интерполируются    
// ×4 внутри af_pwm_poll — шум шейпинга выталкивается к ~31 кГц,    
// где его давит RC-фильтр.    
// Пинг-понг буферы, DMA_IRQ_1 на Core 0.    
  
#pragma once  
  
#include <Arduino.h>  
  
// Запуск PWM-выхода на указанном GPIO. Вызывать из setup() (Core 0).  
void af_pwm_init(void);
  
// Поллинг: забирать int16-блоки (64 отсчёта @15.464 ksps) из выходной    
// очереди af_dsp, интерполировать ×4 -> 256 duty @61.856 ksps    
// (линейная интерполяция + шейпинг 2-го порядка + TPDF-дизер)    
// и отдавать DMA. Вызывать из loop() (Core 0).    
void af_pwm_poll(void);  

// Включение/отключение PWM-выхода. При off несущая глушится:  
// DMA остановлен, слайс выключен, пин переведён в SIO low.  
void af_pwm_set_enabled(bool on);  
bool af_pwm_is_enabled(void);

// Диагностика.  
uint32_t af_pwm_underruns(void);   // DMA дошёл до конца, а новый блок не готов  
bool     af_pwm_is_running(void);

uint32_t af_pwm_blocks_filled(void);    
uint32_t af_pwm_blocks_played(void);    
uint32_t af_pwm_restarts(void);    
uint8_t  af_pwm_duty_fill_max(void);    
void     af_pwm_ur_log(uint32_t* t);   // t — массив из 8 элементов    