// ============================================================================  
// JTTY-передатчик: текст -> атомы -> 34-бит payload -> CRC12 + TBCC -> 4-FSK.  
// WSJT-X 3.3 совместимый: 31.25 Бод, кадр 59 символов по 32 мс (1.888 с),  
// 13 sync-тонов + 46 дибитов, шаг тонов 31.25 Гц, полоса 93.75 Гц.  
// Центр тоновой сетки 1000 Гц (f0 = 953.125 Гц) — внутри FIR 700..1400.  
// Фаза NCO непрерывна (CPFSK): на границе символа меняется только шаг.  
// Выход: int16 @ AF_FS_OUT_HZ, подмешивается в TX-тракт через  
// af_tx_rtty_fir_push() (как rttytx_sample).  
// Генерация целочисленная — jttytx_sample() безопасен на Core 1.  
// Инициализация/упаковка/символьный автомат — Core 0.  
// ============================================================================  
#pragma once  
#include <Arduino.h>  
  
#define JTTYTX_CENTER_HZ  1000u   // центр 4-тонной решётки  
#define JTTYTX_LUT_LEN     256u   // таблица синуса (как у RTTYTX)  
  
// ---- Core 0 ----  
void     jttytx_init(void);              // LUT + NCO-шаги 4 тонов  
bool     jttytx_send(const char* text);  // текст -> передача (false = занято/ошибка упаковки)  
void     jttytx_poll(void);              // символьный автомат, из loop()  
bool     jttytx_busy(void);              // идёт передача  
void     jttytx_abort(void);             // оборвать  
void     jttytx_selftest(void);          // эталонные вектора WSJT-X -> Serial  
  
// ---- Core 1 (TX-ветка af_dsp) ----  
int16_t  jttytx_sample(void);            // следующий отсчёт 4-FSK тона  
uint32_t jttytx_center_hz(void);         // центр решётки для статусной строки  
