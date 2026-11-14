// src/af_tx.h    
//    
// Передающий тракт (SSB): полосовой FIR -> компрессор -> клиппер ->    
// постфильтр. Работает на AF_FS_OUT_HZ, вызывается из af_dsp_poll    
// (Core 1) при tx_on. Управление — с Core 0 (CLI).    
    
#pragma once    
    
#include <Arduino.h>    
    
void    af_tx_init(void);    
int16_t af_tx_process(int32_t y);          // один отсчёт @AF_FS_OUT_HZ    
    
void     af_tx_set_band(uint32_t lo_hz, uint32_t hi_hz);    
uint32_t af_tx_lo_hz(void);    
uint32_t af_tx_hi_hz(void);    
    
void     af_tx_set_ratio(uint32_t r_x10);  // компрессия x10: 20=2:1, 40=4:1    
uint32_t af_tx_ratio(void);    
    
void     af_tx_set_clip(uint32_t thr);     // порог клиппера 0..32767    
uint32_t af_tx_clip(void);

void     af_tx_set_agc(uint32_t target);   // целевой уровень АРУ 0..32767  
uint32_t af_tx_agc(void);  
  
void     af_tx_set_gate(uint32_t thr);     // порог гейта 0..32767 (0 = off)  
uint32_t af_tx_gate(void);  

void     af_tx_set_ap(bool on);        // all-pass симметризатор on/off  
bool     af_tx_ap(void);  
  
void     af_tx_set_softclip(bool on);  // кубический клиппер (off = hard)  
bool     af_tx_softclip(void);  

void     af_tx_set_2band(bool on);   // 2-полосная компрессия (split ~900 Гц)  
bool     af_tx_2band(void);  

void     af_tx_set_gfloor(uint32_t q16);  // нижняя ступень гейта Q16 (0..65536)    
uint32_t af_tx_gfloor(void);    