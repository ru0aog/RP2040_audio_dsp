// src/af_pwm.h  
//  
// Основной выход аудио: PWM на GPIO + RC-фильтр.  
// PWM-слайс свободно бежит на несущей ~125 кГц; DMA обновляет duty  
// с темпом AF_FS_OUT_HZ (31.25 ksps), paced аппаратным таймером.  
// Пинг-понг буферы, DMA_IRQ_1 на Core 0.  
  
#pragma once  
  
#include <Arduino.h>  
  
// Запуск PWM-выхода на указанном GPIO. Вызывать из setup() (Core 0).  
void af_pwm_init(void);
  
// Поллинг: забирать int16-блоки из выходной очереди af_dsp,  
// конвертировать в duty и отдавать DMA. Вызывать из loop() (Core 0).  
void af_pwm_poll(void);  
  
// Диагностика.  
uint32_t af_pwm_underruns(void);   // DMA дошёл до конца, а новый блок не готов  
bool     af_pwm_is_running(void);
