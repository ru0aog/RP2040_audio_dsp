// src/af_usb.h  
//  
// Выход: USB Audio Class, микрофон 44100 Гц / 16 бит / моно (всегда включён).  
// Порт src/rx/rx_usb_audio.cpp из RP2040_radiomodem (моно-версия).  
  
#pragma once  
  
#include <Arduino.h>  
  
// Инициализация TinyUSB + UAC-дескрипторов. Вызывать из setup() (Core 0)  
// ПЕРВЫМ — до Serial.begin(): дескрипторы не должны меняться в окне  
// энумерации (правило из радиомодема).  
void af_usb_init(void);  
  
// Поллинг: забирать int16-блоки из af_dsp_out_get(), ресемплировать  
// 31250 -> 44100 и отдавать в UAC. Вызывать из loop() (Core 0).  
void af_usb_poll(void);  
  
// Диагностика.  
uint32_t af_usb_dropped(void);      // отсчёты, не влезшие в USB-FIFO  
bool     af_usb_mounted(void);      // хост открыл аудио-интерфейс

uint32_t af_usb_underruns(void);    
uint32_t af_usb_frames_short(void);    
uint32_t af_usb_blocks_rx(void);    
int32_t  af_usb_rs_adj_mhz(void);    
void     af_usb_ur_log(uint32_t* t, uint16_t* n);   // t/n — массивы по 8 элементов 
uint32_t af_usb_reprimes(void);   // разы ушли на перезаправку (дно очереди)    