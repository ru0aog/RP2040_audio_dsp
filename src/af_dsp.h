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
  
// Полоса ПФ. Возвращает false, если команда отклонена  
// (при включённом CW полоса следует за f0±250 Гц).  
bool     af_dsp_set_band(uint32_t f_lo_hz, uint32_t f_hi_hz);  
uint32_t af_dsp_af_lo_hz(void);  
uint32_t af_dsp_af_hi_hz(void);  
uint32_t af_dsp_band_lo_hz(void);  
uint32_t af_dsp_band_hi_hz(void);  
  
// Автонотч NT. При включении снимает NR (приоритет NT).  
int      af_dsp_set_notch(bool on);  
bool     af_dsp_notch_on(void);  
  
// Шумоподавление NR. Возвращает false, если отклонено (NT on).  
bool     af_dsp_set_nr(bool on);  
bool     af_dsp_nr_on(void);  
  
// Телеграфный фильтр (3 биквада BPF + шум, после FIR).  
// При включении подтягивает полосу FIR к f0±250 Гц, при выключении —  
// восстанавливает сохранённую.  
void     af_dsp_set_cw(bool on);  
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