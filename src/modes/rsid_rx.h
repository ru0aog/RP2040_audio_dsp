// src/modes/rsid_rx.h  
#pragma once  
#include <Arduino.h>  
void rsid_rx_init(void);  
void rsid_rx_push15464(int16_t s);   // отсчёт 15464 Гц (ресемплинг внутри)  
void rsid_rx_poll(void);             // квант стейт-машины, из rttydec_poll  
void rsid_rx_stop(void);  
bool rsid_get_rtty_center(float* f_hz);  // true = новый центр RTTY-45  
