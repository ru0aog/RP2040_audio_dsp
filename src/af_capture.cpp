// src/af_capture.cpp  
// ============================================================================  
// Захват ADC+DMA, моно: ADC0 (GP26), 16-бит переносы в буферы uint16.  
//  
// Архитектура (порт src/dsp/rx_capture.cpp из RP2040_receiver):  
//   DMA A <-> DMA B: каналы запускают друг друга через chain_to —  
//   мёртвого времени на границах блоков нет.  
//   Core 0 / DMA IRQ0: публикует завершённые буферы в SPSC-очередь.  
//   Core 1 / DSP: получает блок, обрабатывает, вызывает release.  
//  
// Владение памятью:  
//   - DMA пишет только в назначенный ему текущий буфер.  
//   - Опубликованный буфер помечается busy и не назначается DMA.  
//   - Core 1 снимает busy только после завершения чтения блока.  
//   - Если свободного буфера нет, DMA пишет в scratch; блок отбрасывается.  
//  
// ВАЖНО О CHAIN_TO:  
//   Завершившийся канал автоматически запускает напарника. IRQ должен  
//   успеть перенастроить завершившийся канал до того, как напарник  
//   закончит следующий блок (~2 мс при 512 отсчётах на 250 ksps).  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
  
#include "af_capture.h"  
#include "af_config.h"  
  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  
#include "hardware/sync.h"  
#include "hardware/clocks.h"  
  
// Один блок = AF_BLOCK_SAMPLES 16-битных переносов (моно).  
#define AF_BLOCK_TRANSFERS  AF_BLOCK_SAMPLES  
  
// Кольцевая очередь: один пустой слот отличает full от empty (ёмкость 3).  
#define AF_QUEUE_DEPTH      4u  
  
// ---------------------------------------------------------------------------  
// Буферы DMA (volatile — пишутся DMA независимо от CPU)  
// ---------------------------------------------------------------------------  
// У каждого канала своя пара рабочих буферов и свой scratch.  
static volatile uint16_t __attribute__((aligned(4))) af_buf_a0[AF_BLOCK_SAMPLES];  
static volatile uint16_t __attribute__((aligned(4))) af_buf_a1[AF_BLOCK_SAMPLES];  
static volatile uint16_t __attribute__((aligned(4))) af_buf_b0[AF_BLOCK_SAMPLES];  
static volatile uint16_t __attribute__((aligned(4))) af_buf_b1[AF_BLOCK_SAMPLES];  
  
static volatile uint16_t __attribute__((aligned(4))) af_scratch_a[AF_BLOCK_SAMPLES];  
static volatile uint16_t __attribute__((aligned(4))) af_scratch_b[AF_BLOCK_SAMPLES];  
  
// ---------------------------------------------------------------------------  
// SPSC-очередь завершённых блоков: пишет IRQ (Core 0), читает DSP (Core 1)  
// ---------------------------------------------------------------------------  
static const volatile uint16_t* af_queue[AF_QUEUE_DEPTH];  
static volatile uint8_t af_q_head = 0;   // producer: Core 0 / IRQ  
static volatile uint8_t af_q_tail = 0;   // consumer: Core 1 / DSP  
  
static volatile uint32_t af_dropped = 0;  
  
// Диагностика — extern по af_capture.h.    
volatile uint32_t af_cap_isr_count    = 0;    
volatile uint32_t af_drop_scratch     = 0;    
volatile uint32_t af_drop_queue_full  = 0;    
volatile uint32_t af_drop_busy        = 0;    
volatile uint32_t af_adc_fcs_over     = 0;    
volatile uint32_t af_irq_both_pending = 0;    
volatile uint32_t af_q_fill_max       = 0;   // high-water занятых слотов очереди    
volatile uint32_t af_cap_get_empty    = 0;   // get_block вернул NULL (Core 1 голодал)    
volatile uint32_t af_cap_published    = 0;   // блоков опубликовано в очередь    
volatile uint32_t af_cap_released     = 0;   // блоков освобождено DSP    
  
// ---------------------------------------------------------------------------  
// Владение рабочими буферами (busy = опубликован DSP, не назначать DMA)  
// ---------------------------------------------------------------------------  
static volatile bool af_busy_a[2] = { false, false };  
static volatile bool af_busy_b[2] = { false, false };  
static volatile uint8_t af_a_sel = 0;  
static volatile uint8_t af_b_sel = 0;  
  
// Цель текущего/последнего настроенного переноса каждого канала.  
static volatile uint16_t* af_dma_a_target = af_buf_a0;  
static volatile uint16_t* af_dma_b_target = af_buf_b0;  
  
static int af_dma_a = -1;  
static int af_dma_b = -1;  
static volatile bool af_running = false;  
  
// ---------------------------------------------------------------------------  
// Вспомогательные функции  
// ---------------------------------------------------------------------------  
static inline volatile uint16_t* af_buffer_for(bool channel_a, int idx) {  
    if (channel_a) return (idx == 0) ? af_buf_a0 : af_buf_a1;  
    return (idx == 0) ? af_buf_b0 : af_buf_b1;  
}  
  
static inline volatile uint16_t* af_scratch_for(bool channel_a) {  
    return channel_a ? af_scratch_a : af_scratch_b;  
}  
  
static inline volatile bool* af_busy_for(bool channel_a) {  
    return channel_a ? af_busy_a : af_busy_b;  
}  
  
static inline volatile uint8_t* af_select_for(bool channel_a) {  
    return channel_a ? &af_a_sel : &af_b_sel;  
}  
  
// Индекс рабочего буфера или -1 для scratch/неизвестного адреса.  
static int __not_in_flash_func(af_find_buffer)(  
    const volatile uint16_t* target, bool channel_a)  
{  
    if (channel_a) {  
        if (target == af_buf_a0) return 0;  
        if (target == af_buf_a1) return 1;  
    } else {  
        if (target == af_buf_b0) return 0;  
        if (target == af_buf_b1) return 1;  
    }  
    return -1;  
}  
  
// ---------------------------------------------------------------------------  
// Назначить следующую цель DMA и восстановить регистры переноса.  
// Вызывается из IRQ после завершения блока: канал-напарник уже запущен  
// аппаратно через chain_to, этот канал перенастраивается и ждёт пинка.  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(af_rearm_channel)(int channel, bool channel_a) {  
    volatile bool* busy = af_busy_for(channel_a);  
    volatile uint8_t* sel = af_select_for(channel_a);  
  
    // Первый свободный рабочий буфер, начиная с сохранённого выбора.  
    int chosen = -1;  
    for (int k = 0; k < 2; ++k) {  
        uint8_t idx = (uint8_t)((*sel + (uint8_t)k) & 1u);  
        if (!busy[idx]) { chosen = (int)idx; break; }  
    }  
  
    volatile uint16_t* target;  
    if (chosen >= 0) {  
        target = af_buffer_for(channel_a, chosen);  
        *sel = (uint8_t)(chosen ^ 1);  
    } else {  
        // Оба рабочих буфера канала заняты DSP — сливаем в scratch,  
        // данные не рвутся, но блок будет отброшен.  
        target = af_scratch_for(channel_a);  
    }  
  
    if (channel_a) af_dma_a_target = target;  
    else           af_dma_b_target = target;  
  
    // Перенастроить, но НЕ запускать: старт придёт по chain_to от напарника.  
    dma_channel_set_write_addr(channel, (void*)target, false);  
    dma_channel_set_trans_count(channel, AF_BLOCK_TRANSFERS, false);  
}  
  
// ---------------------------------------------------------------------------  
// Обработка одного завершившегося канала  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(af_handle_completed)(int channel, bool channel_a) {  
    dma_channel_acknowledge_irq0(channel);  
    if (!af_running) return;  
  
    // FCS.OVER — sticky-флаг: ADC записал в полный FIFO, отсчёт потерян.  
    if (adc_hw->fcs & ADC_FCS_OVER_BITS) {  
        ++af_adc_fcs_over;  
        const uint32_t fcs_cfg = adc_hw->fcs &  
            (ADC_FCS_EN_BITS | ADC_FCS_SHIFT_BITS | ADC_FCS_ERR_BITS |  
             ADC_FCS_DREQ_EN_BITS | ADC_FCS_THRESH_BITS);  
        adc_hw->fcs = fcs_cfg | ADC_FCS_OVER_BITS;   // OVER — W1C  
    }  
  
    const volatile uint16_t* completed =  
        channel_a ? af_dma_a_target : af_dma_b_target;  
    const int idx = af_find_buffer(completed, channel_a);  
  
    // Блок ушёл в scratch — отбрасываем.  
    if (idx < 0) {  
        ++af_dropped;  
        ++af_drop_scratch;  
        af_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    // Очередь полна — не публикуем.  
    volatile bool* busy = af_busy_for(channel_a);  
    const uint8_t head = af_q_head;  
    const uint8_t next = (uint8_t)((head + 1u) % AF_QUEUE_DEPTH);  
    if (next == af_q_tail) {  
        ++af_dropped;  
        ++af_drop_queue_full;  
        af_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    // Буфер всё ещё busy — сбой протокола владения.  
    if (busy[idx]) {  
        ++af_dropped;  
        ++af_drop_busy;  
        af_rearm_channel(channel, channel_a);  
        return;  
    }  
  
    // Публикация: busy -> указатель -> head.    
    busy[idx] = true;    
    __dmb();    
    af_queue[head] = completed;    
    __dmb();    
    af_q_head = next;    
    ++af_cap_published;    
    
    // High-water заполнения очереди: если упёрлось в ёмкость —    
    // Core 1 систематически не успевает, дропы вопрос времени.    
    const uint8_t fill = (uint8_t)((next - af_q_tail + AF_QUEUE_DEPTH) % AF_QUEUE_DEPTH);    
    if (fill > af_q_fill_max) af_q_fill_max = fill;    
    
    af_rearm_channel(channel, channel_a);    
}  
  
// ---------------------------------------------------------------------------  
// ISR DMA_IRQ_0 (Core 0)  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(af_dma_irq)(void) {  
    ++af_cap_isr_count;  
    const int a = af_dma_a;  
    const int b = af_dma_b;  
  
    const bool a_pending = (a >= 0) && dma_channel_get_irq0_status(a);  
    const bool b_pending = (b >= 0) && dma_channel_get_irq0_status(b);  
  
    // Оба канала одновременно — IRQ был заблокирован дольше блока (~2 мс).  
    if (a_pending && b_pending) ++af_irq_both_pending;  
  
    if (a_pending) af_handle_completed(a, true);  
    if (b_pending) af_handle_completed(b, false);  
}  
  
// ---------------------------------------------------------------------------  
// Запуск захвата. Вызывать ТОЛЬКО на Core 0 — DMA_IRQ_0 регистрируется  
// на вызывающем ядре.  
// ---------------------------------------------------------------------------  
void af_capture_start(void) {  
    if (af_running) return;  
  
    af_q_head = 0;  
    af_q_tail = 0;  
    af_dropped = 0;  
    af_cap_isr_count = 0;  
    af_drop_scratch = 0;  
    af_drop_queue_full = 0;  
    af_drop_busy = 0;  
    af_adc_fcs_over = 0;    
    af_irq_both_pending = 0;    
    af_q_fill_max = 0;    
    af_cap_get_empty = 0;    
    af_cap_published = 0;    
    af_cap_released = 0;    
    af_busy_a[0] = af_busy_a[1] = false;  
    af_busy_b[0] = af_busy_b[1] = false;  
    af_a_sel = 0;  
    af_b_sel = 0;  
    af_dma_a_target = af_buf_a0;  
    af_dma_b_target = af_buf_b0;  
  
    // clk_adc от pll_sys на уровне регистров:      
    //   CTRL.AUXSRC = 0 (clksrc_pll_sys, источник 250 МГц)      
    //   DIV.INT = 5           -> 250/5 = 50 МГц      
    // Порядок по даташиту: сначала AUXSRC, потом DIV (делитель      
    // применяется glitchless к уже идущему клоку).      
    // clk_usb не затрагиваем — он остаётся на pll_usb 48 МГц.      
    // AUXSRC=0 — это pll_sys по даташиту; именованного макроса      
    // для него в этой версии SDK нет, задаём литералом.      
    clocks_hw->clk[clk_adc].ctrl =      
        (0u << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB)      
        | CLOCKS_CLK_ADC_CTRL_ENABLE_BITS;      
    clocks_hw->clk[clk_adc].div =      
        (5u << CLOCKS_CLK_ADC_DIV_INT_LSB);   // INT=5, FRAC=0      
    
    const uint32_t clk_adc_hz = clock_get_hz(clk_adc);     
    Serial.printf("[CAP] clk_adc=%lu clk_sys=%lu\n",    
                  (unsigned long)clk_adc_hz,    
                  (unsigned long)clock_get_hz(clk_sys));    
  
    // ADC: один вход GP26 (ADC0), без round-robin.  
    adc_init();  
    adc_gpio_init(AF_ADC_PIN);  
    adc_select_input(AF_ADC_PIN - 26);      // GP26 -> input 0  
    adc_fifo_setup(true, true, 1, false, false);  
    // DIV задаёт период выборки как (DIV+1) тактов clk_adc — off-by-one        
    // в регистре. Преобразование занимает 96 тактов: период < 96        
    // пропускает каждый второй триггер (половинная частота!), поэтому        
    // минимальный рабочий период = 96 тактов, т.е. DIV >= 95.        
    //        
    // При clk_adc=50 МГц (pll_sys): DIV=99 -> период 100 -> ровно        
    // 500 кс/с. Формула: adc_div = clk_adc/Fs - 1 (DIV = период-1).        
    float adc_div = (float)clk_adc_hz / (float)AF_FS_ADC_HZ - 1.0f;        
    if (adc_div < 95.0f) adc_div = 95.0f;   // минимум 96 тактов периода        
    adc_set_clkdiv(adc_div);       
  
    // Два DMA-канала с аппаратным chain_to: A -> B -> A -> ...  
    af_dma_a = dma_claim_unused_channel(true);  
    af_dma_b = dma_claim_unused_channel(true);  
  
    dma_channel_config ca = dma_channel_get_default_config(af_dma_a);  
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);  
    channel_config_set_read_increment(&ca, false);  
    channel_config_set_write_increment(&ca, true);  
    channel_config_set_dreq(&ca, DREQ_ADC);  
    channel_config_set_chain_to(&ca, af_dma_b);  
  
    dma_channel_config cb = dma_channel_get_default_config(af_dma_b);  
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_16);  
    channel_config_set_read_increment(&cb, false);  
    channel_config_set_write_increment(&cb, true);  
    channel_config_set_dreq(&cb, DREQ_ADC);  
    channel_config_set_chain_to(&cb, af_dma_a);  
  
    dma_channel_configure(af_dma_a, &ca, (void*)af_buf_a0, &adc_hw->fifo,  
                          AF_BLOCK_TRANSFERS, false);  
    dma_channel_configure(af_dma_b, &cb, (void*)af_buf_b0, &adc_hw->fifo,  
                          AF_BLOCK_TRANSFERS, false);  
  
    // IRQ: оба канала на DMA_IRQ_0, обработчик на Core 0.  
    dma_channel_acknowledge_irq0(af_dma_a);  
    dma_channel_acknowledge_irq0(af_dma_b);  
    dma_channel_set_irq0_enabled(af_dma_a, true);  
    dma_channel_set_irq0_enabled(af_dma_b, true);  
    irq_set_exclusive_handler(DMA_IRQ_0, af_dma_irq);  
  
    // Чистый старт: стоп ADC, дренаж FIFO, затем пуск.  
    adc_run(false);  
    adc_fifo_drain();  
    adc_select_input(0);  
  
    af_running = true;  
    irq_set_enabled(DMA_IRQ_0, true);  
  
    dma_channel_start(af_dma_a);   // B стартует аппаратно по chain_to  
    adc_run(true);  
  
    Serial.printf("[CAP] chA=%d chB=%d block=%u transfers=%u\n",  
                  af_dma_a, af_dma_b,  
                  (unsigned)AF_BLOCK_SAMPLES,  
                  (unsigned)AF_BLOCK_TRANSFERS);  
    Serial.printf("[CAP] ADC target=%lu/s div=%ld period=%ld eff=%lu/s\n",    
                  (unsigned long)AF_FS_ADC_HZ, (long)(adc_div + 0.5f),    
                  (long)(adc_div + 1.5f),    
                  (unsigned long)(clk_adc_hz / (uint32_t)(adc_div + 1.0f)));    
}  
  
// ---------------------------------------------------------------------------  
// Останов DMA+ADC  
// ---------------------------------------------------------------------------  
void af_capture_stop(void) {  
    af_running = false;  
    adc_run(false);  
    irq_set_enabled(DMA_IRQ_0, false);  
  
    if (af_dma_a >= 0) {  
        dma_channel_set_irq0_enabled(af_dma_a, false);  
        dma_channel_abort(af_dma_a);  
        dma_channel_acknowledge_irq0(af_dma_a);  
    }  
    if (af_dma_b >= 0) {  
        dma_channel_set_irq0_enabled(af_dma_b, false);  
        dma_channel_abort(af_dma_b);  
        dma_channel_acknowledge_irq0(af_dma_b);  
    }  
  
    adc_fifo_drain();  
    af_q_head = 0;  
    af_q_tail = 0;  
    af_busy_a[0] = af_busy_a[1] = false;  
    af_busy_b[0] = af_busy_b[1] = false;  
    af_a_sel = 0;  
    af_b_sel = 0;  
    af_dma_a_target = af_buf_a0;  
    af_dma_b_target = af_buf_b0;  
  
    if (af_dma_a >= 0) { dma_channel_unclaim(af_dma_a); af_dma_a = -1; }  
    if (af_dma_b >= 0) { dma_channel_unclaim(af_dma_b); af_dma_b = -1; }  
}  
  
// ---------------------------------------------------------------------------  
// API очереди — ТОЛЬКО Core 1 (из af_dsp_poll)  
// ---------------------------------------------------------------------------  
const volatile uint16_t* af_capture_get_block(void) {    
    const uint8_t tail = af_q_tail;    
    if (tail == af_q_head) { ++af_cap_get_empty; return NULL; }    
    __dmb();    
    return af_queue[tail];    
}    
  
void af_capture_release(void) {    
    const uint8_t tail = af_q_tail;    
    if (tail == af_q_head) return;    
    ++af_cap_released;    
    
    const volatile uint16_t* buf = af_queue[tail];    
    __dmb();  
    af_q_tail = (uint8_t)((tail + 1u) % AF_QUEUE_DEPTH);  
    __dmb();  
  
    if      (buf == af_buf_a0) af_busy_a[0] = false;  
    else if (buf == af_buf_a1) af_busy_a[1] = false;  
    else if (buf == af_buf_b0) af_busy_b[0] = false;  
    else if (buf == af_buf_b1) af_busy_b[1] = false;  
}  
  
uint32_t af_capture_dropped(void) { return af_dropped; }  
  
bool af_capture_is_running(void) {  
    return af_running;  
}
