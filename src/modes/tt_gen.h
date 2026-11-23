#pragma once  
#include <stdint.h>  
void    tt_gen_init(void);  
void    tt_gen_set(uint32_t f1, uint32_t f2);  
void    tt_gen_on(bool on);  
bool    tt_gen_busy(void);  
int16_t tt_gen_sample(void);  
uint32_t tt_gen_f1_hz(void);  
uint32_t tt_gen_f2_hz(void);  
