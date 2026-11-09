// src/af_capture.h  
//  
// Захват моно-аудио: ADC0 (GP26) + DMA, 250 ksps, 16-бит переносы.  
// Два DMA-канала A<->B с аппаратным chain_to (порт rx_capture.cpp  
// из RP2040_receiver, моно-версия).  
// DMA_IRQ_0 живёт на Core 0; DSP-поллинг очереди — на Core 1.  
  
#pragma once  
  
#include <Arduino.h>  
  
// Запуск захвата. Вызывать ТОЛЬКО на Core 0 (setup() или инициализация  
// до запуска Core 1) — DMA_IRQ_0 регистрируется на вызывающем ядре.  
void af_capture_start(void);  
  
// Останов DMA+ADC.  
void af_capture_stop(void);  
  
// Взять указатель на заполненный блок (AF_BLOCK_SAMPLES отсчётов uint16,  
// сырые 12-бит значения ADC в младших битах). nullptr если очередь пуста.  
// ТОЛЬКО Core 1 (из af_dsp_poll).  
const volatile uint16_t* af_capture_get_block(void);  
  
// Вернуть блок в пул (после обработки). ТОЛЬКО Core 1.  
void af_capture_release(void);  
  
// Счётчик потерянных блоков (очередь полна / scratch / busy).  
uint32_t af_capture_dropped(void);  
  
bool af_capture_is_running(void);  
  
// Диагностика (extern, читается из CLI/статистики).  
extern volatile uint32_t af_cap_isr_count;    // входов в DMA IRQ  
extern volatile uint32_t af_drop_scratch;     // блок был в scratch  
extern volatile uint32_t af_drop_queue_full;  // очередь переполнена  
extern volatile uint32_t af_drop_busy;        // все рабочие буферы заняты  
extern volatile uint32_t af_adc_fcs_over;     // переполнения ADC FIFO (sticky)  
extern volatile uint32_t af_irq_both_pending; // оба канала в IRQ одновременно
