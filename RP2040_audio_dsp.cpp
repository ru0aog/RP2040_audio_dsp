// RP2040_audio_dsp.cpp
  
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/pll.h"
  
#include "src/af_config.h"  
#include "src/af_capture.h"  
#include "src/af_dsp.h"  
#include "src/af_pwm.h"  
#include "src/af_usb.h"  
#include "src/af_cli.h"  
#include "src/af_gpio_ctl.h"
#include "src/af_store.h"
#include "src/af_led.h"
#include "src/af_voice.h"     // подмес голосовых клипов в выход
  
volatile bool core0_ready = false;  
static uint32_t core0_busy_us = 0;  
static uint32_t core0_win_ms = 0;  
static uint32_t core0_load_permille = 0;

// ============================================================================  
// Конфигурация PLL (из vfo_hardware.cpp маяка)  
// ============================================================================  
#define VFO_XOSC_NOMINAL_HZ     12000000ULL  
#define VFO_CLK_SYS_NOMINAL_HZ  133000000ULL  
  
struct PllConfig {  
    uint32_t clk_hz;  
    uint8_t  refdiv;  
    uint16_t fbdiv;  
    uint8_t  postdiv1;  
    uint8_t  postdiv2;  
    uint8_t  vsel;  
    uint8_t  baudr;  
};  
  
// 250 МГц: VCO = 12М * 125 = 1500 МГц, p1=6, p2=1 → 250 МГц  
// VSEL 1.30 В, BAUDR=7 → SCK флеша ~35.7 МГц  
static const PllConfig pll_cfg = { 250000000UL, 1, 125, 6, 1, 7, 7 };  
  
static uint32_t current_clk_sys_hz = VFO_CLK_SYS_NOMINAL_HZ;  
  
// ----------------------------------------------------------------------------  
// Отключение периферийного клока (clk_peri -> clk_sys)  
// ----------------------------------------------------------------------------  
static void __not_in_flash_func(detach_peripheral_clock)(void) {  
    clock_configure(clk_peri,  
        0,  
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,  
        VFO_CLK_SYS_NOMINAL_HZ,  
        VFO_CLK_SYS_NOMINAL_HZ);  
}  
  
// ----------------------------------------------------------------------------  
// Перепрограммирование PLL_SYS (ядро vfo_set_clk_sys из маяка)  
// ----------------------------------------------------------------------------  
static void __not_in_flash_func(reprogram_pll)(const PllConfig *cfg) {  
    detach_peripheral_clock();  
  
    // 1. clk_sys -> XOSC 12 МГц (безопасный источник на время переключения)  
    clock_configure(clk_sys,  
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
        VFO_XOSC_NOMINAL_HZ,  
        VFO_XOSC_NOMINAL_HZ);  
  
    // 2. Остановить PLL_SYS  
    pll_deinit(pll_sys);  
  
    // 3. Новая частота PLL  
    pll_init(pll_sys, cfg->refdiv,  
             cfg->fbdiv * VFO_XOSC_NOMINAL_HZ,  
             cfg->postdiv1, cfg->postdiv2);  
  
    // 4. Ждём LOCK с таймаутом 10 мс  
    uint32_t t0 = time_us_32();  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS)) {  
        if (time_us_32() - t0 > 10000) break;  
        tight_loop_contents();  
    }  
  
    // 5. Делитель флеша: SSI off -> BAUDR -> SSI on  
    ssi_hw->ssienr = 0;  
    ssi_hw->baudr  = cfg->baudr;  
    ssi_hw->ssienr = 1;  
  
    // 6. clk_sys -> PLL_SYS  
    clock_configure(clk_sys,  
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
        cfg->clk_hz,  
        cfg->clk_hz);  
}  
  
// ----------------------------------------------------------------------------  
// Разгон до 250 МГц: VREG -> PLL -> верификация частотомером -> откат при сбое  
// ----------------------------------------------------------------------------  
static void af_set_clk_250(void) {  
    // Напряжение ядра до разгона (1.30 В — обязательно для 250 МГц)  
    vreg_set_voltage(VREG_VOLTAGE_1_30);  
    busy_wait_us(500);  
  
    uint32_t ints = save_and_disable_interrupts();  
    reprogram_pll(&pll_cfg);  
    restore_interrupts(ints);  
  
    // Верификация: измеряем реальную частоту частотомером  
    uint32_t meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    uint32_t meas_hz  = meas_khz * 1000UL;  
    int32_t  err      = (int32_t)meas_hz - (int32_t)pll_cfg.clk_hz;  
    if (err < 0) err = -err;  
  
    if (err > (int32_t)(pll_cfg.clk_hz / 100)) {  
        // Ошибка > 1% — аварийный откат на номинал 133 МГц  
        static const PllConfig safe = { 133000000UL, 1, 133, 6, 2, 1, 4 };  
        ints = save_and_disable_interrupts();  
        reprogram_pll(&safe);  
        restore_interrupts(ints);  
        vreg_set_voltage(VREG_VOLTAGE_1_10);  
        current_clk_sys_hz = VFO_CLK_SYS_NOMINAL_HZ;  
        return;  
    }  
  
    current_clk_sys_hz = meas_hz;  
}  
  
// ============================================================================  
// Arduino  
// ============================================================================  
void setup()  
{  
    af_set_clk_250();          // разгон первым делом, до USB/Serial  
    af_usb_init();

    Serial.begin(115200);

    // Ждём подключения COM-порта (USB CDC), максимум 5 с.  
    // Как в радиомодеме: без USB-хоста ожидание не блокирует запуск.  
    while (!Serial && millis() < 5000) delay(1);  
  
    Serial.printf("# clk_sys=%lu Hz | FLASH=%lu kHz (BAUDR=%u)\n",  
                  (unsigned long)clock_get_hz(clk_sys),  
                  (unsigned long)(clock_get_hz(clk_sys) / ssi_hw->baudr / 1000),  
                  (unsigned)ssi_hw->baudr);  
  
    af_led_init();             // heartbeat (WS2812 на GP16 у Zero)  
    af_dsp_init();  
    af_cli_init();  
    af_store_init();  
    af_gpio_ctl_init();
    af_capture_start();
    af_pwm_init();

    af_voice_init();  
    af_voice_play(VOICE_READY);   // приветствие после инициализации  
  
    watchdog_enable(3000, true);  // сторожевой 3 с

    __dmb();  
    core0_ready = true;  
}  
  
void loop()  
{  
    const uint32_t t0 = micros();  
  
    af_usb_poll();      // первым: TX-FIFO TinyUSB — самый мелкий буфер тракта  
    af_led_poll();    
    af_cli_poll();    
    af_gpio_ctl_poll();    
    af_pwm_poll();      // ×3: заполняем duty-кольцо быстрее, чем один проход loop    
    af_pwm_poll();    
    af_pwm_poll();    
    af_usb_poll();      // и ещё раз в конце — сглаживает джиттер итерации  
  
    watchdog_update();  
  
    // Учёт загрузки Core 0  
    const uint32_t dt = micros() - t0;  
    core0_busy_us += dt;  
    if (millis() - core0_win_ms >= 1000) {  
        core0_load_permille = core0_busy_us / 1000;  
        core0_busy_us = 0;  
        core0_win_ms = millis();  
    }  

    // Раз в 10 сек — сводка загрузки и диагностика разрывов.  
    // Дельты за период (d*) важнее кумулятивных: видно темп потерь.  
    static uint32_t t_min = 0;  
    static uint32_t p_pub = 0, p_dU = 0, p_dP = 0, p_urU = 0, p_urP = 0;    
    static uint32_t p_usb_ur = 0, p_pwm_ur = 0, p_cap = 0, p_isr = 0;    
    if (millis() - t_min >= 10000) {    
        t_min = millis();    
    
        const uint32_t pub   = af_dsp_out_published_usb();    
        const uint32_t dU    = af_dsp_out_dropped_usb();    
        const uint32_t dP    = af_dsp_out_dropped_pwm();    
        const uint32_t urU   = af_dsp_out_underruns_usb();    
        const uint32_t urP   = af_dsp_out_underruns_pwm();    
        const uint32_t uur   = af_usb_underruns();    
        const uint32_t pur   = af_pwm_underruns();    
        const uint32_t cap   = af_capture_dropped();    
        const uint32_t isr   = af_cap_isr_count;   // реальные блоки от DMA    
  
        Serial.printf(    
            "# L0=%u.%u%% L1=%u.%u%% blk=%lu max=%luus |"    
            " cap: isr=%lu d=%lu scr=%lu qf=%lu bs=%lu fcs=%lu both=%lu qmax=%lu |"      
            " out: dU=%lu dP=%lu urU=%lu urP=%lu fU=%u fP=%u |"    
            " usb: ur=%lu short=%lu rs=%ldmHz rpr=%lu gap=%luus ff=%lu ws=%lu av=%lu |"        
            " pwm: ur=%lu rst=%lu dmax=%u | clip=%lu\n",        
            core0_load_permille / 10, core0_load_permille % 10,        
            af_dsp_load_permille() / 10, af_dsp_load_permille() % 10,        
            (unsigned long)(pub - p_pub),        
            (unsigned long)af_dsp_max_block_us(),        
            (unsigned long)(isr - p_isr), (unsigned long)(cap - p_cap),          
            (unsigned long)af_drop_scratch, (unsigned long)af_drop_queue_full,          
            (unsigned long)af_drop_busy, (unsigned long)af_adc_fcs_over,        
            (unsigned long)af_irq_both_pending, (unsigned long)af_q_fill_max,        
            (unsigned long)(dU - p_dU), (unsigned long)(dP - p_dP),        
            (unsigned long)(urU - p_urU), (unsigned long)(urP - p_urP),        
            (unsigned)af_dsp_out_fill_max_usb(), (unsigned)af_dsp_out_fill_max_pwm(),        
            (unsigned long)(uur - p_usb_ur), (unsigned long)af_usb_frames_short(),        
            (long)af_usb_rs_adj_mhz(), (unsigned long)af_usb_reprimes(),        
            (unsigned long)af_usb_poll_gap_max(), (unsigned long)af_usb_fifo_full(),      
            (unsigned long)af_usb_write_short(), (unsigned long)af_usb_avail_max(),      
            (unsigned long)(pur - p_pwm_ur), (unsigned long)af_pwm_restarts(),        
            (unsigned)af_pwm_duty_fill_max(),        
            (unsigned long)af_dsp_clip_count());        
  
        // Метки последних underrun — интервалы показывают периодичность разрывов.  
        uint32_t ut[8]; uint16_t un[8];  
        af_usb_ur_log(ut, un);  
        Serial.printf("# usb_ur_t:");  
        for (int i = 0; i < 8; i++) Serial.printf(" %lu", (unsigned long)ut[i]);  
        Serial.printf(" | n:");  
        for (int i = 0; i < 8; i++) Serial.printf(" %u", (unsigned)un[i]);  
        Serial.println();  
  
        uint32_t pt[8];  
        af_pwm_ur_log(pt);  
        Serial.printf("# pwm_ur_t:");  
        for (int i = 0; i < 8; i++) Serial.printf(" %lu", (unsigned long)pt[i]);  
        Serial.println();  
  
        p_pub = pub; p_dU = dU; p_dP = dP; p_urU = urU; p_urP = urP;    
        p_usb_ur = uur; p_pwm_ur = pur; p_cap = cap; p_isr = isr;    
    }  
}
  
void setup1()  
{  
    const uint32_t t0 = millis();  
    while (!core0_ready && (uint32_t)(millis() - t0) < 10000u) {  
        af_store_core1_park_poll();  
        tight_loop_contents();  
    }  
}
  
void loop1()  
{  
    // Core 1: пустой поллинг — просто крутится  
    tight_loop_contents();
    af_dsp_poll();
}