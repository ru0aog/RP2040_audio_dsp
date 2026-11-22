// src/af_dsp.h  
//  
// DSP-тракт НЧ (Core 1): децимация 250k -> 15.625k, полосовой FIR 127,  
// CW-биквады, автонотч NT, шумоподавление NR, АРУ+лимитер.  
// Порт src/dsp/rx_dsp.cpp и src/audio/rx_audio.cpp из RP2040_receiver (моно).  
  
#pragma once  
  
#include <Arduino.h>  
  
// Инициализация коэффициентов и состояния. Вызывать из setup() (Core 0)  
// ДО af_capture_start().  
void af_dsp_init(void);  
  
// Поллинг: забрать ADC-блоки из очереди захвата, прогнать тракт,    
// опубликовать int16-блоки в выходные очереди. Вызывать из loop1() (Core 1).    
void af_dsp_poll(void);    
    
// Запрос сброса состояния тракта (dc_est, буферы FIR/CIC/HB, веса NT/NR,    
// АРУ/лимитер, S-meter). Вызывать с Core 0 после af_capture_resume() —    
// фактический сброс выполняется на Core 1 в начале af_dsp_poll,    
// чтобы не было гонки с обработкой блока.    
void af_dsp_reset_state(void);    
  
// Полоса ПФ. Возвращает false, если команда отклонена  
// (при включённом CW полоса следует за f0±250 Гц).  
bool     af_dsp_set_band(uint32_t f_lo_hz, uint32_t f_hi_hz);  
uint32_t af_dsp_af_lo_hz(void);  
uint32_t af_dsp_af_hi_hz(void);  
uint32_t af_dsp_band_lo_hz(void);  
uint32_t af_dsp_band_hi_hz(void);  
  
// Автонотч NT. При включении снимает NR (приоритет NT).  
int      af_dsp_set_notch(bool on);  // rc: 0=off, 1=on, 2=on+NR снят, 3=отклонено (CW on).
bool     af_dsp_notch_on(void);  
  
// Шумоподавление NR. Возвращает false, если отклонено (NT on или CW on)  
bool     af_dsp_set_nr(bool on);  
bool     af_dsp_nr_on(void);

// Dynamic presence EQ RX: SNR-адаптивный подъём ~2.3 кГц после АРУ.  
void    af_dsp_set_rxpres(bool on);  
bool    af_dsp_rxpres_on(void);  
void    af_dsp_set_rxpresmax(int32_t db_x10);  // потолок подъёма, дБ x10 (0..60)  
void    af_dsp_set_rxpresf(uint32_t hz);       // центр пика, 1000..4000 Гц  
void    af_dsp_set_rxpresq(uint32_t q_x100);   // добротность x100 (30..300)  
int32_t af_dsp_rxpresmax(void);                // дБ x10 (не Q15!)  
uint32_t af_dsp_rxpresf(void);  
uint32_t af_dsp_rxpresq(void);  
int32_t af_dsp_rxpres_k(void);                 // сглаженный k_dyn, Q15 (stats)  
int32_t af_dsp_rxpres_env(void);               // огибающая ВЧ-детектора (stats)  
int32_t af_dsp_rxpres_nf(void);                // шумовой пол (stats)  
int32_t af_dsp_rx_in_lvl(void);                // уровень входа после FIR (S-meter)  
int32_t af_dsp_rx_in_peak(void);               // пик |x| после FIR с прошлого чтения (сбрасывает)  

// Noise Blanker RX: подавитель импульсных помех на 125 ksps (после CIC).  
void     af_dsp_set_nb(bool on);  
bool     af_dsp_nb_on(void);  
void     af_dsp_set_nbthr(int32_t x10);   // порог x10 (15..100, 30 = x3.0)  
int32_t  af_dsp_nbthr(void);  
uint32_t af_dsp_nb_hits(void);            // срабатываний с прошлого чтения (сбрасывает)  

// Телеграфный фильтр (3 биквада BPF + шум, после FIR).  
// При включении подтягивает полосу FIR к f0±250 Гц, при выключении —  
// восстанавливает сохранённую.  
void     af_dsp_set_cw(bool on);  // При включении снимает NT и NR (приоритет CW).
bool     af_dsp_cw_on(void);  
void     af_dsp_set_cw_freq(uint32_t hz);  
uint32_t af_dsp_cw_freq(void);  
void     af_dsp_set_cw_bw(uint32_t hz);  
uint32_t af_dsp_cw_bw(void);  
void     af_dsp_set_cw_noise(uint32_t lvl);   // уровень подменного шума, 0..2000  
uint32_t af_dsp_cw_noise(void);  
  
// Выходные очереди int16-блоков (AF_OUT_BLOCK_SAMPLES отсчётов).  
// Fan-out: каждый блок публикуется в ОБЕ очереди — PWM и USB получают  
// полный поток независимо друг от друга.  
const volatile int16_t* af_dsp_out_get_pwm(void);  
void                    af_dsp_out_release_pwm(void);  
const volatile int16_t* af_dsp_out_get_usb(void);  
void                    af_dsp_out_release_usb(void);  
  
// Заполнение очередей (число занятых буферов 0..AF_OUT_NUM_BUFFERS).  
uint8_t af_dsp_out_fill_pwm(void);  
uint8_t af_dsp_out_fill_usb(void);  
  
// Диагностика.  
uint32_t af_dsp_load_permille(void);   // загрузка Core 1, ‰  
uint32_t af_dsp_out_dropped(void);     // блоки, не влезшие в выходную очередь  
uint32_t af_dsp_clip_count(void);      // срабатывания ограничителя int16  
uint32_t af_dsp_max_block_us(void);    // пиковое время блока, мкс (чтение сбрасывает)

// АРУ: off -> ручное усиление gain (множитель, потолок = AGC_MAX_GAIN).  
void   af_dsp_set_agc(bool on);  
bool   af_dsp_agc_on(void);  
void   af_dsp_set_gain(float mult);  
float  af_dsp_gain(void);
  
// Bypass: выход = вход (децимированный сигнал без FIR/CW/NT/NR/AGC).  
void     af_dsp_set_bypass(bool on);  
bool     af_dsp_bypass_on(void);
uint32_t af_dsp_out_dropped_usb(void);    
uint32_t af_dsp_out_dropped_pwm(void);    
uint32_t af_dsp_out_empty_reads_usb(void);   // пустые опросы, не пропуски  
uint32_t af_dsp_out_empty_reads_pwm(void);  
uint32_t af_dsp_out_published_usb(void);    
uint32_t af_dsp_out_published_pwm(void);    
uint8_t  af_dsp_out_fill_max_usb(void);    
uint8_t  af_dsp_out_fill_max_pwm(void);
void af_dsp_set_ramp(bool on);  
bool af_dsp_ramp_on(void);

// Режим тракта: false = RX (приём), true = TX (передача).  
// Переключается по GP8 (PTT) или командой "tx on|off".  
void af_dsp_set_tx(bool on);  
bool af_dsp_tx_on(void);  

void     af_dsp_set_rxnotch(bool on);    
bool     af_dsp_rxnotch_on(void);    
  
// Настройка АРУ/лимитера (SSB-ветка; CW-профиль — CW_AGC_* дефайны).  
void    af_dsp_set_agcref(int32_t v);    // целевой уровень, 1000..32767  
int32_t af_dsp_agcref(void);  
void    af_dsp_set_agcmax(float g);      // потолок усиления, 1..1000  
float   af_dsp_agcmax(void);  
void    af_dsp_set_agcmin(float g);      // нижний предел, 0.01..1.0  
float   af_dsp_agcmin(void);  
void    af_dsp_set_limceil(int32_t v);   // потолок лимитера, 1000..32767  
int32_t af_dsp_limceil(void);  
  
// Настройка NT/NR (шаги адаптации и глубина — экспертные).  
void    af_dsp_set_ntmu(int32_t v);      // шаг NLMS нотча, 0..32768  
int32_t af_dsp_ntmu(void);  
void    af_dsp_set_nrmu(int32_t v);      // шаг LMS NR, 0..32768  
int32_t af_dsp_nrmu(void);  
void    af_dsp_set_nrdepth(int32_t v);   // глубина подавления Q15, 0..65536  
int32_t af_dsp_nrdepth(void);  
void    af_dsp_set_nrmakeup(int32_t v);  // makeup-gain Q15, 0..131072    
int32_t af_dsp_nrmakeup(void);    
  
void    af_dsp_set_nrmode(uint8_t m);  // 0=LMS, 1=spectral    
uint8_t af_dsp_nrmode(void);    

void     af_dsp_set_rxshelf(int32_t db_x10);  
void     af_dsp_set_rxshelff(uint32_t hz);  
void     af_dsp_set_rxshelfq(uint32_t q_x100);  
int32_t  af_dsp_rxshelf(void);  
uint32_t af_dsp_rxshelff(void);  
uint32_t af_dsp_rxshelfq(void);  
void     af_dsp_set_rxexc(bool on);  
bool     af_dsp_rxexc_on(void);  
void     af_dsp_set_rxexcmix(int32_t q15);  
int32_t  af_dsp_rxexcmix(void);  
int32_t  af_dsp_rxexc_harm(void);   // EMA уровня гармоник эксайтера  
int32_t  af_dsp_rxexc_hpeak(void);  // пик, чтение сбрасывает  
void     af_dsp_set_cwdec(bool on);  
bool     af_dsp_cwdec_on(void);  
uint32_t af_dsp_cwdec_dropped(void); 

// --- CW-передатчик ---    
void af_dsp_cwtx_start(const char* text);   // текст в очередь + TX on    
bool af_dsp_cwtx_started(void);             // TX поднят командой cw tx    
void af_dsp_cwtx_clear_started(void);       // сброс флага после возврата в RX    
bool af_dsp_cwtx_busy(void);                // посылка ещё играет    