// ============================================================================  
// RTTY-передатчик: текст -> Baudot ITA-2/МТК-2 -> FSK-тон.  
// 45.45 бод, shift 170 Гц, центр 1000 Гц (mark=1085, space=915).  
// Стоп-бит 1.5, в паузах — mark (idle). Фаза NCO непрерывна (CPFSK).  
// Выход: int16 @ AF_FS_OUT_HZ, подмешивается в TX-тракт (GP1).  
// Генерация целочисленная — rttytx_sample() безопасен на Core 1.  
// Инициализация/очередь/бит-автомат — Core 0.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define RTTYTX_BAUD      45.45f  
#define RTTYTX_MARK_HZ   1085u    // центр 1000 + 85  
#define RTTYTX_SPACE_HZ   915u    // центр 1000 - 85  
#define RTTYTX_LUT_LEN    256u    // таблица синуса (как у CWTX)  
  
// ---- Core 0 ----  
void     rttytx_init(void);              // LUT + NCO-шаги mark/space  
bool     rttytx_send(const char* text);  // текст -> передача (false = занято)  
void     rttytx_poll(void);              // бит-автомат, из loop()  
bool     rttytx_busy(void);              // идёт передача  
void     rttytx_abort(void);             // оборвать  
  
// ---- Core 1 (TX-ветка af_dsp) ----  
int16_t  rttytx_sample(void);            // следующий отсчёт FSK-тона  
uint32_t rttytx_center_hz(void);   // центр mark/space для статусной строки  