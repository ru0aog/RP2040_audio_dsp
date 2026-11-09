// src/af_store.h  
//  
// Хранилище банков настроек во flash (EEPROM-эмуляция, Core 0).  
// 7 ячеек; в каждой: полоса ПФ, CW on/off, f0, bw, noise.  
// NT/NR в банке не хранятся — управляются пинами GP5/GP6 и консолью.  
  
#pragma once  
  
#include <Arduino.h>

// Число ячеек банка (коды 001..111 на GP2..GP4; 000 = ручной режим).  
#define AF_STORE_BANKS  7
  
struct af_bank_t {  
    uint16_t lo_hz;      // нижний срез ПФ, Гц  
    uint16_t hi_hz;      // верхний срез ПФ, Гц  
    uint8_t  cw_on;      // 0/1  
    uint16_t cw_f0;      // центральная частота CW, Гц  
    uint16_t cw_bw;      // полоса CW, Гц  
    uint16_t cw_noise;   // уровень добавки шума 0..2000 (не проценты!)
    uint8_t  agc_on;      // 0/1 — АРУ; при 0 действует gain_pct  
    uint16_t gain_pct;    // ручное усиление 0..400%  
};  
  
// Инициализация EEPROM. Вызывать из setup() до af_gpio_ctl_init().  
void af_store_init(void);  
  
// Записать текущие настройки DSP в ячейку idx (1..AF_BANK_COUNT).  
// Возвращает false при неверном idx.  
bool af_store_save(uint8_t idx);  
  
// Прочитать ячейку idx в out. Возвращает false, если ячейка пустая/битая.  
bool af_store_read(uint8_t idx, af_bank_t* out);  
  
// Применить ячейку idx к DSP. Возвращает false, если ячейка пустая.  
bool af_store_load(uint8_t idx);

// Вызывать из af_dsp_poll (Core 1) каждую итерацию — обработка запроса  
// на парковку перед записью flash.  
void af_store_core1_park_poll(void);

// Записать массив из AF_STORE_BANKS записей разом (команда 'bank clr').  
// После записи — watchdog_reboot, функция не возвращается.  
void af_store_write_all(const af_bank_t banks[AF_STORE_BANKS]);