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
  
// Текущий код банка с GP2..GP4 (0 = ручной режим).    
uint8_t af_gpio_bank_code(void);  

// Управление PTT/TX для CLI (play voice_14 в TX+PTT, аналог GP10).    
bool af_gpio_ptt_tx_on(void);    
void af_gpio_ptt_tx_off(void);    

uint8_t af_gpio_bank_code(void);              // текущий код банка (0 = manual)  
void    af_gpio_set_bank_code(uint8_t code);  // установить из CLI 'load N'  

uint8_t af_gpio_bank_code(void);      // текущий код 0..7 (0 = manual)    
uint8_t af_gpio_last_bank(void);      // последний загруженный банк 1..7    
void    af_gpio_set_bank_code(uint8_t code);    
