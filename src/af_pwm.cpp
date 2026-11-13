// src/af_pwm.cpp  
// ============================================================================  
// af_pwm.cpp — выход аудио через PWM на GPIO (Core 0)  
//  
// Схема:  
//   - PWM-слайс на AF_PWM_PIN, wrap = AF_PWM_WRAP, clkdiv подобран так,  
//     чтобы несущая была ~125 кГц (clk_sys / (wrap+1) / div).  
//   - DMA-канал пишет duty-значения (uint16) в pwm_hw->slice[n].cc,    
//     темп выдачи — pacing-таймер DMA на AF_PWM_FS_HZ (~61.9 кГц).    
//   - Кольцо из 4 буферов по AF_PWM_BLOCK_SAMPLES duty-значений;    
//     по завершении блока DMA_IRQ_1 переключает канал на следующий    
//     готовый буфер (4 буфера = ~16 мс slack при 61.856 ksps).    
//   - af_pwm_poll() (Core 0) забирает int16-блоки 64 отсчёта @15.464 кс/с    
//     из af_dsp_out_get_pwm(), интерполирует ×4 -> 256 duty @61.856 кс/с    
//     (линейная интерполяция + шейпинг 2-го порядка + TPDF-дизер)    
//     и заполняет свободные буферы кольца.    
//   - DMA стартует отложенно: только когда заполнены >=14 duty-буферов    
//     (~58 мс подушки) — убирает фазовую гонку «IRQ раньше заполнения».    
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
static uint32_t pwm_pace_den  = 0;   // базовый делитель pacing-таймера  
static uint32_t pwm_pace_frac = 0;   // дробная часть делителя (такты на период Fs)  
static uint32_t pwm_pace_acc  = 0;   // накопитель дробной части  

static volatile bool     pwm_running   = false;    
static volatile uint32_t pwm_underruns = 0;    
static volatile bool     pwm_enabled   = true;   // pwm off -> несущая заглушена  
  
// Диагностика разрывов.  
static volatile uint32_t pwm_blocks_filled = 0;   // duty-буферов заполнено poll'ом  
static volatile uint32_t pwm_blocks_played = 0;   // блоков доиграно DMA (переключений)  
static volatile uint32_t pwm_restarts      = 0;   // отложенных стартов DMA  
static volatile uint8_t  duty_fill_max     = 0;   // high-water готовых буферов  
  
// Кольцо последних 8 underrun-событий: метка micros — видна периодичность.  
static volatile uint32_t pwm_ur_t[8];  
static volatile uint8_t  pwm_ur_idx = 0;  
  
// Кольцо duty-буферов: DMA играет duty_sel, poll заполняет свободные.    
static uint16_t duty_buf[AF_OUT_NUM_BUFFERS][AF_PWM_BLOCK_SAMPLES];
static volatile bool    duty_ready[AF_OUT_NUM_BUFFERS];    
static volatile uint8_t duty_sel = 0;    
static uint8_t          duty_fill = 0;   // следующий буфер для записи
  
// ---------------------------------------------------------------------------    
// int16 -> duty: noise shaping 2-го порядка + TPDF-дизер    
//    
// Шейпинг: ошибка квантования подаётся назад как 2*e1 - e2 —      
// спектр шума квантования выталкивается к Fs/2 (~31 кГц при      
// AF_PWM_FS_HZ=61.856 кГц), где его давит внешний RC-фильтр.      
// В полосе 0-3 кГц выигрыш ~25-30 дБ против 1-го порядка @15.5 кс/с.      
//    
// TPDF-дизер ±1 LSB шага duty: размачивает предельные циклы шейпинга    
// (тонкие "свисты" на тихих сигналах) в широкую полку.    
// ---------------------------------------------------------------------------    
static uint32_t dither_lfsr = 0xACE1u;          // xorshift32, не 0    
    
static inline uint16_t __not_in_flash_func(to_duty)(int16_t s,    
                                                    int32_t* e1,    
                                                    int32_t* e2) {    
    // s: -32768..32767 -> duty: 0..AF_PWM_WRAP    
    int32_t v = ((int32_t)s + 32768) * (int32_t)AF_PWM_WRAP;    
    
    // TPDF: (r1 - r2) в [-65536, +65536) = ±1 LSB шага квантования    
    dither_lfsr ^= dither_lfsr << 13;    
    dither_lfsr ^= dither_lfsr >> 17;    
    dither_lfsr ^= dither_lfsr << 5;    
    int32_t r1 = (int32_t)(dither_lfsr & 0xFFFF);    
    dither_lfsr ^= dither_lfsr << 13;    
    dither_lfsr ^= dither_lfsr >> 17;    
    dither_lfsr ^= dither_lfsr << 5;    
    int32_t r2 = (int32_t)(dither_lfsr & 0xFFFF);    
    v += (r1 - r2);    
    
    v += 2 * (*e1) - (*e2);                     // обратная связь 2-го порядка    
    int32_t d = v >> 16;                        // /65536    
    if (d < 0) d = 0;    
    if (d > (int32_t)AF_PWM_WRAP) d = AF_PWM_WRAP;    
    *e2 = *e1;      
    int32_t e = v - (d << 16);                  // остаток — в следующие отсчёты      
    // При клиппинге d (0/WRAP) ошибка растёт неограниченно и после      
    // клипа выстреливает обратно — "выхлоп". Ограничиваем ±2 LSB:      
    // шейпинг сохраняется, накопление на перегрузке — нет.      
    if (e >  131072) e =  131072;      
    if (e < -131072) e = -131072;      
    *e1 = e;      
    return (uint16_t)d;      
}    
  
// ---------------------------------------------------------------------------  
// DMA IRQ: блок доиграл — переключаемся на следующий готовый буфер  
// ---------------------------------------------------------------------------  
static void __not_in_flash_func(af_pwm_dma_irq)(void) {  
    dma_hw->ints1 = 1u << pwm_dma_ch;           // ack IRQ  
  
    // Ищем следующий готовый буфер по кольцу ДО освобождения текущего:  
    // при underrun DMA повторяет duty_sel — он должен остаться ready,  
    // иначе poll перезапишет его прямо под читающим DMA.  
    uint8_t next = duty_sel;  
    for (uint8_t k = 1; k <= AF_OUT_NUM_BUFFERS; k++) {  
        uint8_t cand = (uint8_t)(duty_sel + k);  
        if (cand >= AF_OUT_NUM_BUFFERS) cand -= AF_OUT_NUM_BUFFERS;  
        if (duty_ready[cand]) { next = cand; break; }  
    }  
    if (next == duty_sel) {    
        ++pwm_underruns;                        // нет готовых — повтор текущего    
        pwm_ur_t[pwm_ur_idx] = time_us_32();    // метка события — периодичность    
        pwm_ur_idx = (uint8_t)((pwm_ur_idx + 1) & 7);    
    } else {    
        duty_ready[duty_sel] = false;           // доигранный буфер — свободен    
        duty_sel = next;    
        ++pwm_blocks_played;    
    }   
  
    // Перезапуск канала на выбранный буфер (или повтор старого при underrun).    
    dma_channel_set_read_addr(pwm_dma_ch, duty_buf[duty_sel], false);      
    dma_channel_set_trans_count(pwm_dma_ch, AF_PWM_BLOCK_SAMPLES, true);  
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
  
    // Pacing-таймер DMA на AF_PWM_FS_HZ (~61.856 кГц = 4 x AF_FS_OUT_HZ).      
    // Точный делитель 250e6/61856 = 4041.3 — не целый. Округление до 4041      
    // даёт 61865.6 Гц: DMA жрёт на 9.6 Гц быстрее производства -> underrun      
    // каждые ~26 с. Решение: чередуем den 4041/4042 с шагом 0.3 — средний      
    // темп совпадает с производством, дрейфа нет.      
    pwm_dma_timer = dma_claim_unused_timer(true);      
    uint32_t f_sys = clock_get_hz(clk_sys);      
    uint32_t den = f_sys / AF_PWM_FS_HZ;        // 4041      
    if (den > 0xFFFE) den = 0xFFFE;             // нужен запас +1 для чередования      
    dma_timer_set_fraction(pwm_dma_timer, 1, (uint16_t)den);    
    // Дробная часть делителя: frac = f_sys - den*AF_PWM_FS_HZ (в единицах      
    // AF_PWM_FS_HZ). Накопитель: каждые AF_PWM_FS_HZ тиков добавляем frac      
    // лишних тактов — реализуем в af_pwm_poll счётчиком блоков.      
    pwm_pace_frac = f_sys - den * (uint32_t)AF_PWM_FS_HZ;  // ~0.3*Fs в тактах      
    pwm_pace_den  = den;    
    channel_config_set_dreq(&c, dma_get_timer_dreq(pwm_dma_timer));  
  
    dma_channel_configure(  
        pwm_dma_ch, &c,  
        &pwm_hw->slice[pwm_slice].cc,           // запись в CC (канал A/B по сдвигу)  
        duty_buf[0],    
        AF_PWM_BLOCK_SAMPLES,    
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
    static int32_t dither_e1 = 0, dither_e2 = 0;      
    static int32_t interp_prev = 0;             // предыдущий отсчёт для линейки    
    
    if (!pwm_enabled) {    
        // Выход выключен: выкидываем готовые блоки, чтобы очередь    
        // af_dsp не забилась (out_dropped не росла).    
        while (af_dsp_out_get_pwm() != NULL) af_dsp_out_release_pwm();    
        interp_prev = 0;    
        return;    
    }    
    
    // Кольцевое заполнение: пишем в буфер, который DMA уже прочитал,    
    // а не линейно 0..N — иначе CPU перезаписывает буфер под DMA.    
    for (uint8_t k = 0; k < AF_OUT_NUM_BUFFERS; k++) {    
        if (duty_ready[duty_fill]) break;       // кольцо заполнено    
    
        const volatile int16_t* blk = af_dsp_out_get_pwm();    
        if (blk == NULL) break;    
    
        // Интерполяция x4: между соседними отсчётами — 3 промежуточных    
        // точки линейки (шаги 1/4, 2/4, 3/4). Образы спектра на    
        // 15.5/31/46.5 кГц давятся линейкой ~-12 дБ + RC-фильтром;    
        // шейпинг 2-го порядка работает на выходной частоте 61.856 кГц —    
        // шум квантования уходит к ~31 кГц, за пределы полезной полосы.    
        uint32_t o = 0;    
        for (uint32_t i = 0; i < AF_OUT_BLOCK_SAMPLES; i++) {    
            const int32_t cur = blk[i];    
            const int32_t step = cur - interp_prev;    
            duty_buf[duty_fill][o++] = to_duty((int16_t)(interp_prev + (step >> 2)),       &dither_e1, &dither_e2);    
            duty_buf[duty_fill][o++] = to_duty((int16_t)(interp_prev + (step >> 1)),       &dither_e1, &dither_e2);    
            duty_buf[duty_fill][o++] = to_duty((int16_t)(interp_prev + ((step * 3) >> 2)), &dither_e1, &dither_e2);    
            duty_buf[duty_fill][o++] = to_duty((int16_t)cur,                               &dither_e1, &dither_e2);    
            interp_prev = cur;    
        }    
    
        __dmb();      
        duty_ready[duty_fill] = true;      
        af_dsp_out_release_pwm();      
        ++pwm_blocks_filled;      
    
        // High-water готовых буферов: упёрлось в N — poll успевает,    
        // держится у нуля — DMA жрёт быстрее, чем приходит из af_dsp.    
        uint8_t n = 0;    
        for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++) if (duty_ready[b]) ++n;    
        if (n > duty_fill_max) duty_fill_max = n;    
    
        if (++duty_fill >= AF_OUT_NUM_BUFFERS) duty_fill = 0;    
    }  
    
    // Старт DMA только когда заполнены >= AF_OUT_NUM_BUFFERS-2 (=14)      
    // duty-буферов (~58 мс подушки) — убирает фазовую гонку      
    // «IRQ раньше заполнения».      
    if (!pwm_running) {      
        uint8_t n = 0;      
        for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++) if (duty_ready[b]) ++n;      
        if (n >= AF_OUT_NUM_BUFFERS - 2) {      // ~12 мс подушки перед стартом      
            dma_channel_set_read_addr(pwm_dma_ch, duty_buf[duty_sel], false);      
            dma_channel_set_trans_count(pwm_dma_ch, AF_PWM_BLOCK_SAMPLES, true);      
            pwm_running = true;    
            ++pwm_restarts;                     // каждый рестарт = был полный стоп DMA    
        }      
    }

    // Дробная коррекция pacing: накопитель прибавляет pwm_pace_frac за        
    // каждый доигранный ОТСЧЁТ (по дельте pwm_blocks_played — не по вызовам        
    // poll, их темп произвольный). Когда накапливается AF_PWM_FS_HZ —        
    // следующий блок играем с den+1 (на 1 такт медленнее).        
    // Средний темп = точные AF_PWM_FS_HZ, дрейфа нет.        
    if (pwm_running && pwm_pace_frac) {        
        static uint32_t pace_played_prev = 0;        
        const uint32_t played = pwm_blocks_played;        
        const uint32_t dplay  = played - pace_played_prev;   // блоков с прошлого poll        
        pace_played_prev = played;        
        // За блок дефицит = frac*256/Fs = 165 тактов; блок на den+1 даёт      
        // +256 тактов. acc += frac за блок, порог Fs -> den+1 на ~64.5%      
        // блоков -> средний делитель 4041.645 = точный.      
        pwm_pace_acc += pwm_pace_frac * dplay;          
        if (pwm_pace_acc >= (uint32_t)AF_PWM_FS_HZ) {          
            pwm_pace_acc -= (uint32_t)AF_PWM_FS_HZ;          
            dma_timer_set_fraction(pwm_dma_timer, 1, (uint16_t)(pwm_pace_den + 1));          
        } else {          
            dma_timer_set_fraction(pwm_dma_timer, 1, (uint16_t)pwm_pace_den);          
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
        for (uint8_t b = 0; b < AF_OUT_NUM_BUFFERS; b++) duty_ready[b] = false;    
        duty_fill = 0;    
        duty_sel = 0;    
        duty_fill_max = 0;    
        pwm_ur_idx = 0;    
        memset((void*)pwm_ur_t, 0, sizeof(pwm_ur_t));  
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
uint32_t af_pwm_underruns(void)      { return pwm_underruns; }    
uint32_t af_pwm_blocks_filled(void)  { return pwm_blocks_filled; }    
uint32_t af_pwm_blocks_played(void)  { return pwm_blocks_played; }    
uint32_t af_pwm_restarts(void)       { return pwm_restarts; }    
uint8_t  af_pwm_duty_fill_max(void)  { return duty_fill_max; }    
bool     af_pwm_is_running(void)     { return pwm_running; }    
    
// Копия кольца последних 8 underrun-событий (t — micros).    
void af_pwm_ur_log(uint32_t* t) {    
    for (int i = 0; i < 8; i++) t[i] = pwm_ur_t[i];    
}  