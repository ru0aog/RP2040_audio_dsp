// src/af_store.h  
//  
// Хранилище банков настроек во flash (последний сектор, Core 0).      
// 7 ячеек; каждая — полный снимок настраиваемых параметров RX и TX.    
// NT/NR/rxnotch on/off в банке не хранятся — включаются по обстановке  
// (пины GP5/GP6 и консоль); их параметры настройки (mu, depth) — хранятся.  
// rx_pres_on хранится — это параметр звучания, а не оперативный режим.  
  
#pragma once    
  
#include <Arduino.h>  
  
// Число ячеек банка (коды 001..111 на GP2..GP4; 000 = ручной режим).    
#define AF_STORE_BANKS  7  
    
struct af_bank_t {    
    // --- RX ---    
    uint16_t lo_hz;         // нижний срез ПФ, Гц    
    uint16_t hi_hz;         // верхний срез ПФ, Гц    
    uint8_t  cw_on;         // 0/1    
    uint16_t cw_f0;         // центральная частота CW, Гц    
    uint16_t cw_bw;         // полоса CW, Гц    
    uint16_t cw_noise;      // уровень добавки шума 0..2000 (не проценты!)  
    uint8_t  agc_on;        // 0/1 — АРУ; при 0 действует gain_pct    
    uint16_t gain_pct;      // ручное усиление, Q8 (256 = ×1.0)  
    uint16_t voice_vol;     // громкость голосовых сообщений, % (0..400)  
    uint16_t agc_ref;       // целевой уровень АРУ (SSB)    
    uint16_t agc_max_x100;  // потолок АРУ x100 (300 = ×3.00)    
    uint16_t agc_min_x100;  // нижний предел АРУ x100    
    uint16_t lim_ceil;      // потолок лимитера    
    uint16_t nr_depth;      // глубина NR, Q15    
    uint32_t nr_makeup;     // makeup NR, Q15 (до 131072 — не влезает в u16)    
    uint16_t nt_mu;         // шаг NLMS нотча  
    uint16_t nr_mu;         // шаг LMS NR  
    uint8_t  rx_pres_on;    // dynamic presence EQ RX (0/1)  
    uint16_t rx_presmax;    // потолок подъёма, дБ x10 (15 = +1.5 дБ)  
    uint16_t rx_pres_freq;  // центр пика, Гц (1000..4000)  
    uint16_t rx_pres_q;     // добротность x100 (100 = 1.00)  
    // --- TX ---  
    uint16_t tx_lo_hz;      // речевая полоса TX    
    uint16_t tx_hi_hz;    
    uint16_t tx_ratio_x10;  // компрессия x10 (40 = 4:1)    
    uint16_t tx_clip;       // порог клиппера    
    uint16_t tx_agc_target; // целевой уровень TX-АРУ (0 = off)    
    uint16_t tx_agc_max_x10;// потолок TX-АРУ x10    
    uint16_t tx_gate_thr;   // порог гейта (0 = off)    
    uint16_t tx_gate_hold_ms;    
    uint8_t  tx_gate_hyst_x10;    
    uint16_t tx_gfloor;     // нижняя ступень гейта, Q16    
    uint8_t  tx_ap_on;      // all-pass    
    uint8_t  tx_softclip;   // 1 = soft, 0 = hard    
    uint8_t  tx_2band_on;   // 2-полосная компрессия    
    uint16_t tx_split_hz;   // частота кроссовера    
    uint8_t  tx_lr2_on;     // кроссовер LR2    
    uint16_t tx_comp_thr;   // пороги компрессора    
    uint16_t tx_comp_thr_lo;    
    uint16_t tx_comp_thr_hi;    
    uint8_t  tx_duck_on;    // HF ducking    
    uint16_t tx_duckmax;    // потолок ослабления НЧ, дБ x10    
    uint8_t  tx_intel_on;   // автобаланс НЧ/ВЧ    
    uint16_t tx_intel_tgt;  // Q15    
    uint16_t tx_intel_cut;  // Q15    
    uint16_t tx_intel_boost;// Q15    
    uint8_t  tx_pres_on;    // dynamic presence EQ    
    uint16_t tx_presmax;    // потолок подъёма, дБ x10    
    uint16_t tx_pres_freq;  // центр пика, Гц    
    uint16_t tx_pres_q;     // добротность x100    
    uint16_t tx_shelf_db;   // подъём ВЧ, дБ x10 (0 = off)    
    uint16_t tx_shelf_freq; // частота полки, Гц    
    uint16_t tx_shelf_q;    // добротность x100    
    uint8_t  tx_exc_on;     // ВЧ-эксайтер    
    uint16_t tx_excmix;     // подмес эксайтера, Q15    
    uint8_t  tx_notch_on;   // TX-нотчи      
    uint16_t tx_notch1_hz;      
    uint16_t tx_notch2_hz;      
    uint8_t  nr_mode;       // движок NR: 0 = LMS-ALE, 1 = спектральный FFT-256  
    uint8_t  _pad;          // выравнивание/резерв  
};      
  
// Инициализация EEPROM. Вызывать из setup() до af_gpio_ctl_init().  
void af_store_init(void);  
  
// Записать текущие настройки DSP в ячейку idx (1..AF_BANK_COUNT).  
// Возвращает false при неверном idx.  
bool af_store_save(uint8_t idx);  
  
// Прочитать ячейку idx в out. Возвращает false, если ячейка пустая/битая.    
bool af_store_read(uint8_t idx, af_bank_t* out);    
    
// Снимок текущих настроек DSP в out (столбец "cur" таблицы banks).    
void af_store_snapshot(af_bank_t* out);    
  
// Применить ячейку idx к DSP. Возвращает false, если ячейка пустая.    
bool af_store_load(uint8_t idx);  
  
// Вызывать из loop() (Core 0): при пустом/чужом секторе — отложенная    
// запись дефолтных банков (аналог 'banks clr'). Одноразовая.    
void af_store_poll(void);  

// Вызывать из af_dsp_poll (Core 1) каждую итерацию — обработка запроса  
// на парковку перед записью flash.  
void af_store_core1_park_poll(void);

// Записать массив из AF_STORE_BANKS записей разом ('banks clr',    
// дефолтные банки при пустом секторе). Возвращается: reset Core 1 +    
// IRQ-off + erase/program + af_set_clk + restartCore1 — без ребута.    
void af_store_write_all(const af_bank_t banks[AF_STORE_BANKS]);  

void af_set_clk(void);          // разгон clk_sys по pll_cfg (RP2040_audio_dsp.cpp)    
void af_set_clk_nominal(void);  // номинал 125 МГц перед flash-записью (тот же reprogram_pll)    
  
// Core 0: припарковать Core 1 (spin в RAM, IRQ off) перед flash-операцией.    
// false = таймаут, Core 1 не припаркован — flash трогать нельзя.    
bool af_store_core1_park(uint32_t timeout_ms);    
  
// Core 0: отпустить Core 1 после flash-операции.    
void af_store_core1_unpark(void);  

// Заводские пресеты всех банков — единый источник дефолтов.  
// Используется af_store_init (пустой/чужой сектор) и CLI 'banks clr'. 
void af_store_fill_defaults(af_bank_t b[AF_STORE_BANKS]);  

// FLASH-TEST: N циклов erase/program/verify в тестовый сектор 0x100000.  
// mode: 0=чистое окно, 1=+park, 2=+downclock, 3=park+downclock.  
void af_store_test(uint8_t mode, uint32_t iters);  