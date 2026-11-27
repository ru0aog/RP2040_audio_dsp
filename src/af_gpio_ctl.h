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
  
// Управление PTT/TX для CLI (play voice_14 в TX+PTT, аналог GP10).    
bool af_gpio_ptt_tx_on(void);    
void af_gpio_ptt_tx_off(void);    

void    af_gpio_set_bank_code(uint8_t code);  // установить из CLI 'load N'  
uint8_t af_gpio_bank_code(void);      // текущий код 0..7 (0 = manual)    
uint8_t af_gpio_last_bank(void);      // последний загруженный банк 1..7    
uint8_t af_gpio_msg_sel(void);        // код селектора сообщений 0..3 (GP13/GP28)  

void    af_gpio_key_mode(uint8_t m);     // 0=off 1=iambicA 2=iambicB 3=straight    
uint8_t af_gpio_key_mode_get(void);    
void    af_gpio_beacon_set(const char* text, uint32_t period_s);  // 0/пусто = выкл    
bool    af_gpio_beacon_on(void);    
void    af_gpio_vbcn_set(uint32_t period_s);   // голосовой маяк, 0 = выкл    
bool    af_gpio_vbcn_on(void);    

// Мультимодовый маяк GP7: CW -> JTTY -> RTTY, стоп по концу цикла.  
bool af_gpio_mbcn_set(bool on);    // on=false — запрос стопа; false = отказ (пустой слот)  
bool af_gpio_mbcn_on(void);  
void af_gpio_mbcn_abort(void);     // немедленный обрыв посылки и маяка  
bool af_gpio_mbcn_set_sel(uint8_t slot);   // mbcn on <0..3> — слот из CLI, не с пинов  

bool    af_gpio_ptt_active(void);   // sw_ptt_active — PTT поднят софтом  
bool    af_gpio_key_tx(void);       // key_tx — ключ держит TX  
uint8_t af_gpio_key_mode(void);     // 0=off 1=iambicA 2=iambicB 3=straight  
