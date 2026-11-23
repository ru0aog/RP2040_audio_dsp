// ============================================================================  
// CW-передатчик: текст -> Морзе -> тон с колоколообразной огибающей.  
// Выход: int16 @ AF_FS_OUT_HZ, подмешивается в TX-тракт (GP1).  
// Генерация целочисленная — cwtx_sample() безопасен на Core 1.  
// Инициализация/очередь — Core 0 (cwtx_init/cwtx_send/cwtx_poll).  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define CWTX_RISE_MS     4u     // фронт/спад посылки, мс (raised cosine)  
#define CWTX_LUT_LEN     256u   // таблица синуса, отсчётов на период  

// ---- Ручной режим (кейер): Core 0 подаёт элементы напрямую, без очереди ----    
void     cwtx_manual_begin(void);        // вход в ручной режим (сброс состояния)    
bool     cwtx_manual_feed(bool dash);    // подать элемент (false = фаза ещё играет)    
void     cwtx_manual_key_down(void);     // прямой ключ: открыть (элемент "бесконечный")    
void     cwtx_manual_key_up(void);       // прямой ключ: закрыть (спад rise)    
void     cwtx_manual_end(void);          // выход: хвост-тишина -> active=false    
bool     cwtx_el_rem_free(void);       // true = элемент и пауза доиграны, можно feed    

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
