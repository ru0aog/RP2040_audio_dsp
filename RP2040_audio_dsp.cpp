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
#include "src/af_rec.h"           // цифровой магнитофон: RAM-буфер ~7.8 с (децимация ×2)  
  
volatile bool core0_ready = false;    
static uint32_t core0_busy_us = 0;    
static uint32_t core0_win_ms = 0;    
static uint32_t core0_load_permille = 0;  
  
// Загрузка Core 0, ‰ (busy-время loop за 1 с). Читается из cli_stats.  
uint32_t af_core0_load_permille(void) { return core0_load_permille; }  

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
};  
  
// 250 МГц: VCO = 12М * 125 = 1500 МГц, p1=6, p2=1 → 250 МГц  
// VSEL 1.30 В, BAUDR=6 → SCK флеша ~41.7 МГц  
static const PllConfig pll_cfg = { 250000000UL, 1, 125, 6, 1};
// Номинал для flash-операций: VCO = 12М * 125 = 1500 МГц, p1=6, p2=2 → 125 МГц    
// BAUDR=4 → SCK флеша 31.25 МГц. Тот же путь reprogram_pll, что и разгон.    
static const PllConfig pll_cfg_nominal = { 125000000UL, 1, 125, 6, 2};

static uint32_t current_clk_sys_hz = VFO_CLK_SYS_NOMINAL_HZ;  
  
// ----------------------------------------------------------------------------  
// Отключение периферийного клока (clk_peri -> clk_sys)  
// ----------------------------------------------------------------------------  
static void __not_in_flash_func(detach_peripheral_clock)(void) {  
    clock_configure(clk_peri,  
        0,  
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
        48000000u,  
        48000000u);  
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

    // 5. clk_sys -> PLL_SYS  
    clock_configure(clk_sys,  
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
        cfg->clk_hz,  
        cfg->clk_hz);  

    // 6. Делитель флеша: SCK строго <= 48 МГц, BAUDR чётный (как в маяке)  
    uint32_t ssi_baud = (cfg->clk_hz + 48000000u - 1) / 48000000u;  
    ssi_baud = (ssi_baud + 1u) & ~1u;  
    if (ssi_baud < 2u)  ssi_baud = 2u;  
    if (ssi_baud > 34u) ssi_baud = 34u;  
    ssi_hw->ssienr = 0;  
    ssi_hw->baudr  = ssi_baud;  
    ssi_hw->ssienr = 1;  
}  
  
// ----------------------------------------------------------------------------  
// Разгон: VREG -> PLL -> верификация частотомером -> откат при сбое  
// ----------------------------------------------------------------------------  
void af_set_clk(void) {  
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
        static const PllConfig safe = { 133000000UL, 1, 133, 6, 2 };  
        ints = save_and_disable_interrupts();  
        reprogram_pll(&safe);  
        restore_interrupts(ints);  
        vreg_set_voltage(VREG_VOLTAGE_1_10);  
        current_clk_sys_hz = VFO_CLK_SYS_NOMINAL_HZ;  
        return;  
    }  
  
    current_clk_sys_hz = meas_hz;  
}

// Переход на номинал 125 МГц перед flash-записью — тем же самодельным    
// путём reprogram_pll (XOSC -> pll_deinit/init -> LOCK -> PLL -> BAUDR).    
// НЕ использовать set_sys_clock_khz: SDK-путь переконфигурирует clk_peri/    
// usb/adc и PLL по своей логике, ломая состояние для обратного af_set_clk.    
void __not_in_flash_func(af_set_clk_nominal)(void) {    
    uint32_t ints = save_and_disable_interrupts();    
    reprogram_pll(&pll_cfg_nominal);    
    restore_interrupts(ints);    
    current_clk_sys_hz = pll_cfg_nominal.clk_hz;    
}    
  
// ============================================================================  
// Arduino  
// ============================================================================  
static void core1_main(void);   // рабочий цикл Core 1, RAM-only  

void setup()  
{  
    af_set_clk();              // разгон первым делом, до USB/Serial  
    af_usb_init();

    Serial.begin(115200);
    af_led_init();             // heartbeat (WS2812 на GP16 у Zero)
    af_led_set(10, 0, 0);

    // Ждём подключения COM-порта (USB CDC), максимум 3 с.  
    // Как в радиомодеме: без USB-хоста ожидание не блокирует запуск.  
    while (!Serial && millis() < 3000) delay(1);  

    watchdog_enable(8000, true);  // сторожевой 8 с
    
    Serial.printf("# clk_sys=%lu Hz | FLASH=%lu kHz (BAUDR=%u)\n",  
                  (unsigned long)clock_get_hz(clk_sys),  
                  (unsigned long)(clock_get_hz(clk_sys) / ssi_hw->baudr / 1000),  
                  (unsigned)ssi_hw->baudr);  
  
    af_dsp_init();  
    af_cli_init();  
    Serial.println("# before store_init"); Serial.flush();  
    af_store_init();  
    Serial.println("# after store_init");  Serial.flush(); 
    af_rec_init();
    af_led_set(0, 0, 10);
    af_gpio_ctl_init();
    af_capture_start();
    af_pwm_init();
    af_voice_init();  
    af_voice_play(VOICE_READY);   // приветствие после инициализации  
    af_led_set(0, 10, 0);

    __dmb();  
    core0_ready = true;  
}  
  
void loop()  
{  
    const uint32_t t0 = micros();  
  
    af_usb_poll();      // первым: TX-FIFO TinyUSB — самый мелкий буфер тракта  
    af_led_poll();    
    af_cli_poll();  
    af_store_poll();    // запись дефолтных банков при пустом секторе (одноразово)  
    af_rec_poll();      // магнитофон: контроль записи (авто-стоп по буферу)  
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
    // Периодическая сводка убрана — диагностика по команде 'stats'.  
}
  
void setup1()    
{    
    const uint32_t t0 = millis();    
    while (!core0_ready && (uint32_t)(millis() - t0) < 10000u) {    
        af_store_core1_park_poll();    
        tight_loop_contents();    
    }    
    // Весь рабочий цикл Core 1 — в RAM. Инвариант: во время flash    
    // erase/program (af_store, ~45 мс, XIP off) Core 1 не должна    
    // обращаться к flash — ни кодом, ни данными. loop1() Arduino-ядра    
    // лежит во flash, поэтому крутим свой цикл из RAM-функции.    
    core1_main();   // не возвращается  
}  
  
// Рабочий цикл Core 1 — строго в RAM. Запрещено вызывать отсюда    
// (и из af_dsp_poll) любые функции без __not_in_flash_func и читать    
// const-данные/XIP-указатели — иначе зависание во время записи af_store.    
static void __not_in_flash_func(core1_main)(void)  
{  
    for (;;) {  
        af_store_core1_park_poll();   // парковка по запросу af_store (RAM)  
        af_dsp_poll();                // вся DSP-цепочка — __not_in_flash_func  
    }  
}  
  
void loop1()  
{  
    // Не используется: рабочий цикл Core 1 — core1_main() в RAM,  
    // вызывается из setup1(). Оставлена для Arduino-ядра.  
}  