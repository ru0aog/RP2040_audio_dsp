// ============================================================================  
// CW-передатчик: текст -> Морзе -> тон с колоколообразной огибающей.  
// Выход: int16 @ AF_FS_OUT_HZ, подмешивается в TX-тракт (GP1).  
// Генерация целочисленная — cwtx_sample() безопасен на Core 1.  
// Инициализация/очередь — Core 0 (cwtx_init/cwtx_send/cwtx_poll).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define CWTX_RISE_MS     5u     // фронт/спад посылки, мс (raised cosine)  
#define CWTX_LUT_LEN     256u   // таблица синуса, отсчётов на период  
  
// ---- Core 0 ----    
void     cwtx_init(uint32_t tone_hz);        // построить LUT + NCO-шаг    
bool     cwtx_send(const char* text);        // поставить текст в очередь (false = занято)    
void     cwtx_poll(void);                    // подача элементов/пауз в cwtx_el_rem (из loop())    
void     cwtx_set_wpm(uint8_t wpm);          // скорость 5..60    
bool     cwtx_busy(void);                    // идёт передача (очередь/элементы)    
void     cwtx_abort(void);                   // оборвать передачу    
  
// ---- Core 1 (TX-ветка af_dsp) ----  
int16_t  cwtx_sample(void);                  // следующий отсчёт тона (0 в паузах)  
void     cwtx_set_tone(uint32_t tone_hz);    // перестройка NCO без пересоздания LUT  
uint32_t cwtx_tone_hz(void);                 // текущий тон, Гц (для статуса OLED)  
