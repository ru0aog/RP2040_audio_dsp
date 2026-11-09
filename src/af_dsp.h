// src/af_dsp.h  
//  
// DSP-тракт НЧ (Core 1): децимация 250k -> 31.25k, полосовой FIR,  
// автонотч NT, шумоподавление NR, АРУ+лимитер.  
// Порт src/dsp/rx_dsp.cpp и src/audio/rx_audio.cpp из RP2040_receiver (моно).  
  
#pragma once  
  
#include <Arduino.h>  
  
// Инициализация коэффициентов и состояния. Вызывать из setup() (Core 0)  
// ДО af_capture_start() в setup1().  
void af_dsp_init(void);  
  
// Поллинг: забрать ADC-блоки из очереди захвата, прогнать тракт,  
// опубликовать int16-блоки в выходную очередь. Вызывать из loop1() (Core 1).  
void af_dsp_poll(void);

// Телеграфный фильтр (каскад биквадов BPF, после FIR).  
void     af_dsp_set_cw(bool on);  
bool     af_dsp_cw_on(void);  
void     af_dsp_set_cw_freq(uint32_t hz);  
uint32_t af_dsp_cw_freq(void);  
void     af_dsp_set_cw_bw(uint32_t hz);  
uint32_t af_dsp_cw_bw(void);  
void     af_dsp_set_cw_noise(uint32_t lvl);   // уровень подменного шума, 0..2000  
uint32_t af_dsp_cw_noise(void);
bool     af_dsp_agc_on(void);

// Управление (Core 0, редкие операции).  
void     af_dsp_set_band(uint32_t f_lo_hz, uint32_t f_hi_hz);  
uint32_t af_dsp_af_lo_hz(void);  
uint32_t af_dsp_af_hi_hz(void);  
void     af_dsp_set_notch(bool on);  
void     af_dsp_set_nr(bool on);  
bool     af_dsp_notch_on(void);  
bool     af_dsp_nr_on(void);  
  
// Выходные очереди int16-блоков (AF_OUT_BLOCK_SAMPLES отсчётов).  
// Fan-out: каждый блок публикуется в ОБЕ очереди — PWM и USB получают  
// полный поток независимо друг от друга (конкуренции за блоки нет).  
// Читают af_pwm_poll/af_usb_poll на Core 0.  
const volatile int16_t* af_dsp_out_get_pwm(void);  
void                    af_dsp_out_release_pwm(void);  
const volatile int16_t* af_dsp_out_get_usb(void);  
void                    af_dsp_out_release_usb(void);  
  
// Заполнение USB-очереди (число занятых буферов 0..AF_OUT_NUM_BUFFERS).  
// Используется af_usb_poll для адаптивной подстройки темпа ресемплера.  
uint8_t af_dsp_out_fill_usb(void);
  
// Диагностика.  
uint32_t af_dsp_load_permille(void);   // загрузка Core 1, ‰  
uint32_t af_dsp_out_dropped(void);     // блоки, не влезшие в выходную очередь  
uint32_t af_dsp_clip_count(void);      // срабатывания ограничителя int16
uint32_t af_dsp_cw_in_lvl(void);  
uint32_t af_dsp_cw_gate(void);
uint32_t af_dsp_max_block_us(void);   // пиковое время блока, мкс (чтение сбрасывает)

void     af_dsp_set_notch(bool on);  
void     af_dsp_set_nr(bool on);  
bool     af_dsp_nr_on(void);  
void     af_dsp_set_band(uint32_t lo_hz, uint32_t hi_hz);  
uint32_t af_dsp_band_lo_hz(void);  
uint32_t af_dsp_band_hi_hz(void);
uint8_t  af_dsp_out_fill_pwm(void);