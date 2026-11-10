// src/af_pwm.cpp  
// ============================================================================  
// af_pwm.cpp — выход аудио через PWM на GPIO (Core 0)  
//  
// Схема:  
//   - PWM-слайс на AF_PWM_PIN, wrap = AF_PWM_WRAP, clkdiv подобран так,  
//     чтобы несущая была ~125 кГц (clk_sys / (wrap+1) / div).  
//   - DMA-канал пишет duty-значения (uint16) в pwm_hw->slice[n].cc,  
//     темп выдачи — pacing-таймер DMA на AF_FS_OUT_HZ.  
//   - Кольцо из 4 буферов по AF_OUT_BLOCK_SAMPLES duty-значений;  
//     по завершении блока DMA_IRQ_1 переключает канал на следующий  
//     готовый буфер (4 буфера = ~64 мс slack при 15.625 ksps).  
//   - af_pwm_poll() (Core 0) забирает int16-блоки из af_dsp_out_get_pwm(),  
//     конвертирует int16 -> duty и заполняет свободные буферы кольца.  
//   - DMA стартует отложенно: только когда заполнены >=2 duty-буфера —  
//     убирает фазовую гонку «IRQ раньше заполнения» (underruns каждый блок).  
//  
// Порт src/audio/pwm_audio.cpp из RP2040_receiver.  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
#include "af_config.h"  
#include "af_pwm.h"  
#include "af_dsp.h"  
  
#include "hardware/pwm.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  
#include "hardware/clocks.h"  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static int      pwm_dma_ch    = -1;  
static uint     pwm_slice     = 0;  
static uint     pwm_chan      = 0;  
static uint     pwm_dma_timer = 0;

static volatile bool     pwm_running   = false;  
static volatile uint32_t pwm_underruns = 0;  
static volatile bool     pwm_enabled   = true;   // pwm off -> несущая заглушена
  
// Кольцо duty-буферов: DMA играет duty_sel, poll заполняет свободные.  
static uint16_t duty_buf[4][AF_OUT_BLOCK_SAMPLES];  
static volatile bool    duty_ready[4] = {false, false, false, false};  
static volatile uint8_t duty_sel = 0;  
  
// ---------------------------------------------------------------------------  
// int16 -> duty с дизерингом ошибки (error feedback)  
// ---------------------------------------------------------------------------  
static inline uint16_t __not_in_flash_func(to_duty)(int16_t s, int32_t* err) {  
    // s: -32768..32767 -> duty: 0..AF_PWM_WRAP  
    int32_t v = ((int32_t)s + 32768) * (int32_t)AF_PWM_WRAP;  
    v += *err;                                  // накопленная ошибка  
    int32_t d = v >> 16;                        // /65536  
    if (d < 0) d = 0;  
    if (d > (int32_t)AF_PWM_WRAP) d = AF_PWM_WRAP;  
    *err = v - (d << 16);                       // остаток — в следующий отсчёт  
    return (uint16_t)d;  
}  
  
// ---------------------------------------------------------------------------  
// DMA IRQ: блок доиграл — переключаемся на следующий готовый буфер  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(af_pwm_dma_irq)(void) {  
    dma_hw->ints1 = 1u << pwm_dma_ch;           // ack IRQ  
  
    duty_ready[duty_sel] = false;               // доигранный буфер — свободен  
  
    // Ищем следующий готовый буфер по кольцу.  
    uint8_t next = duty_sel;  
    for (uint8_t k = 1; k <= 4; k++) {  
        uint8_t cand = (uint8_t)((duty_sel + k) & 3u);  
        if (duty_ready[cand]) { next = cand; break; }  
    }  
    if (next == duty_sel) ++pwm_underruns;      // нет готовых — underrun  
    duty_sel = next;  
  
    // Перезапуск канала на выбранный буфер (или повтор старого при underrun).    
    dma_channel_set_read_addr(pwm_dma_ch, duty_buf[duty_sel], false);    
    dma_channel_set_trans_count(pwm_dma_ch, AF_OUT_BLOCK_SAMPLES, true);
}  
  
// ---------------------------------------------------------------------------  
// Инициализация (Core 0)  
// ---------------------------------------------------------------------------  
void af_pwm_init(void) {  
    // --- PWM-слайс ---  
    gpio_set_function(AF_PWM_PIN, GPIO_FUNC_PWM);  
    pwm_slice = pwm_gpio_to_slice_num(AF_PWM_PIN);  
    pwm_chan  = pwm_gpio_to_channel(AF_PWM_PIN);  
  
    pwm_config cfg = pwm_get_default_config();  
    pwm_config_set_wrap(&cfg, AF_PWM_WRAP);  
  
    // clkdiv: несущая = clk_sys / (wrap+1) / div, цель ~125 кГц.  
    float div = (float)clock_get_hz(clk_sys) /  
                ((float)(AF_PWM_WRAP + 1) * 125000.0f);  
    if (div < 1.0f) div = 1.0f;  
    pwm_config_set_clkdiv(&cfg, div);  
    pwm_init(pwm_slice, &cfg, true);  
    pwm_set_chan_level(pwm_slice, pwm_chan, 0);  
  
    // --- DMA-канал: duty_buf -> PWM CC ---  
    pwm_dma_ch = dma_claim_unused_channel(true);  
    dma_channel_config c = dma_channel_get_default_config(pwm_dma_ch);  
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);  
    channel_config_set_read_increment(&c, true);  
    channel_config_set_write_increment(&c, false);  
  
    // Pacing-таймер DMA на AF_FS_OUT_HZ.  
    pwm_dma_timer = dma_claim_unused_timer(true);  
    uint32_t f_sys = clock_get_hz(clk_sys);  
    uint32_t den = 0;  
    if (f_sys % AF_FS_OUT_HZ == 0) {  
        den = f_sys / AF_FS_OUT_HZ;             // точный делитель  
    } else {  
        den = (f_sys + AF_FS_OUT_HZ / 2) / AF_FS_OUT_HZ;  
        if (den > 0xFFFF) den = 0xFFFF;  
    }  
    dma_timer_set_fraction(pwm_dma_timer, 1, (uint16_t)den);  
    channel_config_set_dreq(&c, dma_get_timer_dreq(pwm_dma_timer));  
  
    dma_channel_configure(  
        pwm_dma_ch, &c,  
        &pwm_hw->slice[pwm_slice].cc,           // запись в CC (канал A/B по сдвигу)  
        duty_buf[0],  
        AF_OUT_BLOCK_SAMPLES,  
        false);                                 // НЕ запускаем — ждём данных  
  
    dma_channel_set_irq1_enabled(pwm_dma_ch, true);  
    irq_set_exclusive_handler(DMA_IRQ_1, af_pwm_dma_irq);  
    irq_set_enabled(DMA_IRQ_1, true);  
  
    memset((void*)duty_buf, 0, sizeof(duty_buf));  
    pwm_running = false;                        // запустит af_pwm_poll  
}  
  
// ---------------------------------------------------------------------------  
// Poll (Core 0): заполнение свободных duty-буферов + отложенный старт DMA  
// ---------------------------------------------------------------------------  
void __not_in_flash_func(af_pwm_poll)(void) {  
    static int32_t dither_err = 0;  
  
    if (!pwm_enabled) {  
        // Выход выключен: выкидываем готовые блоки, чтобы очередь  
        // af_dsp не забилась (out_dropped не росла).  
        while (af_dsp_out_get_pwm() != NULL) af_dsp_out_release_pwm();  
        return;  
    }  
  
    for (uint8_t b = 0; b < 4; b++) {
        if (duty_ready[b]) continue;            // буфер ещё играет  
  
        const volatile int16_t* blk = af_dsp_out_get_pwm();  
        if (blk == NULL) break;                 // нет данных — к проверке старта  
  
        for (uint32_t i = 0; i < AF_OUT_BLOCK_SAMPLES; i++)  
            duty_buf[b][i] = to_duty(blk[i], &dither_err);  
  
        __dmb();  
        duty_ready[b] = true;  
        af_dsp_out_release_pwm();  
    }  
  
    // Старт DMA только когда заполнены минимум 2 duty-буфера —  
    // убирает фазовую гонку «IRQ раньше заполнения».  
    if (!pwm_running) {  
        uint8_t n = 0;  
        for (uint8_t b = 0; b < 4; b++) if (duty_ready[b]) ++n;  
        if (n >= 2) {  
            dma_channel_start(pwm_dma_ch);  
            pwm_running = true;  
        }  
    }  
}

// ---------------------------------------------------------------------------  
// Вкл/выкл PWM-выхода (команда 'pwm on|off')  
// ---------------------------------------------------------------------------  
void af_pwm_set_enabled(bool on) {  
    if (on == pwm_enabled) return;  
    pwm_enabled = on;  
  
    if (!on) {  
        // Глушим несущую: стоп DMA, слайс off, пин в SIO low.  
        dma_channel_abort(pwm_dma_ch);  
        pwm_running = false;  
        for (uint8_t b = 0; b < 4; b++) duty_ready[b] = false;  
        pwm_set_chan_level(pwm_slice, pwm_chan, 0);  
        pwm_set_enabled(pwm_slice, false);  
        gpio_set_function(AF_PWM_PIN, GPIO_FUNC_SIO);  
        gpio_set_dir(AF_PWM_PIN, GPIO_OUT);  
        gpio_put(AF_PWM_PIN, 0);  
    } else {  
        // Возвращаем пин слайсу и включаем PWM; DMA перезапустит  
        // af_pwm_poll, когда накопятся >=2 duty-буфера.  
        gpio_set_function(AF_PWM_PIN, GPIO_FUNC_PWM);  
        pwm_set_enabled(pwm_slice, true);  
        pwm_running = false;  
    }  
}  
  
bool af_pwm_is_enabled(void) { return pwm_enabled; }
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
uint32_t af_pwm_underruns(void)  { return pwm_underruns; }  
bool     af_pwm_is_running(void) { return pwm_running; }
