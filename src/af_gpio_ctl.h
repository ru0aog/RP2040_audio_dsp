// src/af_gpio_ctl.h  
//  
// Управление функциями DSP по состоянию GPIO (Core 0).  
// Пин замкнут на GND -> функция включена; разомкнут -> выключена.  
// Пины и пресеты полосы — в af_config.h (секция AF_GPIO_*).  
  
#pragma once  
  
#include <Arduino.h>  
  
// Настройка пинов (INPUT_PULLUP) и применение начального состояния.  
// Вызывать из setup() (Core 0) ПОСЛЕ af_dsp_init().  
void af_gpio_ctl_init(void);  
  
// Неблокирующий опрос пинов с дебаунсом. Вызывать из loop() (Core 0).  
void af_gpio_ctl_poll(void);