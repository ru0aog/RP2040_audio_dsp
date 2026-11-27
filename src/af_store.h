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

#define AF_CW_MSGS     4      
#define AF_CW_MSG_LEN  256   // включая '\0' — строка до 255 символов      
  
// Мода слота сообщения: 0=пустой(NULL), 1=CW, 2=RTTY, 3=JTTY.  
// Задаётся в CW.TXT записью N=MODE:TEXT; хранится вместе с текстом.  
#define AF_CW_MODE_NULL  0  
#define AF_CW_MODE_CW    1  
#define AF_CW_MODE_RTTY  2  
#define AF_CW_MODE_JTTY  3  
  
// Дефолтный вызов для всех слотов при невалидном CW-блоке во flash      
// (чистая плата / битый crc / старый ver). Пустая строка "" = дефолты      
// не подставлять. Мода дефолтных слотов — AF_CW_MODE_CW.      
#define AF_CW_DEFAULT  "VVV TESTING BEACON K"
  
// Дефолтные слоты при невалидном CW-блоке во flash (чистая плата /  
// битый crc / смена AF_CW_VER). Демонстрационный набор: все три моды  
// + пустой слот NULL. Текст пустой строки "" = слот без сообщения.  
static const struct { uint8_t mode; const char* text; } AF_CW_DEFAULTS[AF_CW_MSGS] = {  
    { AF_CW_MODE_CW,   "CQ CQ CQ DE RU0AOG RU0AOG RU0AOG K" },  
    { AF_CW_MODE_RTTY, "CQ DE RU0AOG K" },  
    { AF_CW_MODE_JTTY, "CQ RU0AOG" },  
    { AF_CW_MODE_NULL, "" },  
};  

extern char    af_cw_msgs[AF_CW_MSGS][AF_CW_MSG_LEN];    
extern uint8_t af_cw_mode[AF_CW_MSGS];   // AF_CW_MODE_* — мода каждого слота  
  
void af_store_cw_init(void);   // чтение из образа сектора (в af_store_init);    
                               // при невалидном блоке — дефолты AF_CW_DEFAULT    
                               // во все слоты + отложенная запись через af_store_poll    
bool af_store_cw_save(void);   // запись во flash (окно как af_store_save)  

struct af_bank_t {    
    // --- RX ---    
    uint16_t lo_hz;         // нижний срез ПФ, Гц    
    uint16_t hi_hz;         // верхний срез ПФ, Гц    
    uint8_t  cw_on;         // 0/1    
    uint16_t cw_f0;         // центральная частота CW, Гц    
    uint16_t cw_bw;         // полоса CW, Гц    
    uint16_t cw_noise;      // уровень добавки шума 0..2000 (не проценты!)    
    uint8_t  cwdec_on;      // CW-декодер (0/1)    
    uint8_t  cw_wpm;        // стартовая скорость декодера, WPM (5..60)    
    uint32_t cw_sql_x100;   // порог сквелча декодера x100 (25 = 0.25, 0 = off)    
    uint32_t cw_agc_max_x100; // потолок АРУ ТЛГ x100 (5000 = x50.00)    
    uint8_t  rtty_on;        // RTTY-декодер (0/1)  
    uint8_t  rtty_scan;      // сканер Гёрцеля (0/1)  
    uint8_t  rtty_rsid;      // захват по RSID (0/1)  
    uint8_t  rtty_rev;       // инверсия полярности (0/1)    
    uint8_t  rtty_dbg;       // диагностика # scan:/# trk: (0/1)    
    uint16_t rtty_ctr;       // ручной центр, Гц (0 = авто)    
    uint32_t rtty_sql_x100;  // сквелч x100 (15 = 0.15, 0 = off)      
    uint8_t  agc_on;        // 0/1 — АРУ; при 0 действует gain_pct        
    uint32_t gain_pct;      // ручное усиление, Q8 (256 = ×1.0, до ×2000)    
    uint16_t voice_vol;     // громкость голосовых сообщений, % (0..400)    
    uint16_t agc_ref;       // целевой уровень АРУ (SSB)      
    uint32_t agc_max_x100;  // потолок АРУ x100 (200000 = ×2000.00)      
    uint32_t agc_min_x100;  // нижний предел АРУ x100      
    uint16_t lim_ceil;      // потолок лимитера    
    uint16_t nr_depth;      // глубина NR, Q15    
    uint32_t nr_makeup;     // makeup NR, Q15 (до 131072 — не влезает в u16)    
    uint16_t nt_mu;         // шаг NLMS нотча  
    uint16_t nr_mu;         // шаг LMS NR  
    uint8_t  rx_pres_on;    // dynamic presence EQ RX (0/1)  
    uint16_t rx_presmax;    // потолок подъёма, дБ x10 (15 = +1.5 дБ)  
    uint16_t rx_pres_freq;  // центр пика, Гц (1000..4000)  
    uint16_t rx_pres_q;     // добротность x100 (100 = 1.00)  
    uint8_t  nb_on;         // noise blanker RX (0/1)    
    uint16_t nb_thr_x10;    // порог NB x10 (30 = x3.0)    
    uint16_t rx_shelf_db;   // RX low-shelf «бас», дБ x10 (0 = off, 30 = +3 дБ)    
    uint16_t rx_shelf_freq; // частота полки, Гц (200..800)    
    uint16_t rx_shelf_q;    // добротность x100 (70 = 0.70)    
    uint8_t  rx_exc_on;     // RX гармонический эксайтер T2 (0/1)      
    uint16_t rx_exc_mix;    // подмес эксайтера, Q15 (3277 = ~10%)      
    uint8_t  rxband_on;     // полосовой FIR RX на входе (0/1, 1 = в тракте)  
    uint16_t lo2_hz;        // нижний срез пост-фильтра band2, Гц  
    uint16_t hi2_hz;        // верхний срез пост-фильтра band2, Гц  
    uint8_t  band2_on;      // пост-фильтр на выходе (0/1, 1 = в тракте)  
    uint8_t  rxeq_on;       // 3-полосный EQ RX (0/1)  
    uint8_t  rxshelf_on;    // low-shelf «бас» RX (0/1; при 0 db не применяется)  
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
    uint8_t  txband_on;     // речевой pre-FIR TX (0/1; post-FIR не гейтится —  
                          // антигармониковая страховка после клиппера)  
    uint8_t  txeq_on;       // 3-полосный EQ TX (0/1)  
    uint8_t  txagc_on;      // TX-АРУ (0/1; при 0 agc_env считается — нужен  
                          // гейту и sidechain presence, gain не применяется)  
    uint8_t  txgate_on;     // шумовой гейт TX (0/1)  
    uint8_t  txcomp_on;     // компрессор TX (0/1)  
    uint8_t  txclip_on;     // клиппер TX (0/1)  
    uint8_t  txshelf_on;    // ВЧ-полка TX (0/1)  
    // Этап 8: 3-полосный EQ (дБx10, -120..+120, 0=off; частоты Гц)      
    int16_t  rxeq_lo_db, rxeq_mid_db, rxeq_hi_db;    
    uint16_t rxeq_lo_f,  rxeq_mid_f,  rxeq_hi_f;    
    int16_t  txeq_lo_db, txeq_mid_db, txeq_hi_db;    
    uint16_t txeq_lo_f,  txeq_mid_f,  txeq_hi_f;    
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

