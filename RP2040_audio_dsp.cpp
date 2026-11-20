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
#include "src/af_fm.h"            // файловый менеджер: FAT12-диск 16 КБ, SET.TXT
  
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
    uint8_t  baudr;
};  
  
// 250 МГц: VCO = 12М * 125 = 1500 МГц, p1=6, p2=1 → 250 МГц  
// VSEL 1.30 В, BAUDR=6 → SCK флеша ~41.7 МГц  
// static const PllConfig pll_cfg_boosted = { 250000000UL, 1, 125, 6, 1, 6};

// 320 МГц: VCO = 12М * 80 = 960 МГц, p1=3, p2=1 → 320 МГц    
// VSEL 1.30 В, BAUDR=8 → SCK флеша 40 МГц    
static const PllConfig pll_cfg_boosted = { 320000000UL, 1, 80, 3, 1, 8}; 


// Номинал для flash-операций: VCO = 12М * 125 = 1500 МГц, p1=6, p2=2 → 125 МГц    
// BAUDR=4 → SCK флеша 31.25 МГц. Тот же путь reprogram_pll, что и разгон.    
static const PllConfig pll_cfg_nominal = { 125000000UL, 1, 125, 6, 2, 4};

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
    
    // Напряжение ядра для разгона (1.30 В — обязательно для 250 МГц)  
    vreg_set_voltage(VREG_VOLTAGE_1_30);  
    busy_wait_us(500);

    //  периферийная шина - на 48 МГц
    clock_configure(clk_peri, 0,  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
        48000000u, 48000000u); 
    
    // сохраняем прерывания
    uint32_t ints = save_and_disable_interrupts(); 

    // пересадка clk_sys -> XOSC 12 МГц (безопасный источник на время переключения)  
    clock_configure(clk_sys,  
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
        VFO_XOSC_NOMINAL_HZ,  
        VFO_XOSC_NOMINAL_HZ);  
  
    // Новая частота PLL  
    pll_init(pll_sys, cfg->refdiv,  
             cfg->fbdiv * VFO_XOSC_NOMINAL_HZ,  
             cfg->postdiv1, cfg->postdiv2);  
  
    // Ждем взвода бита LOCK в регистре CS системной PLL
    // Если аналоговая петля не успеет стабилизироваться за 2000 итераций — мы разорвем цикл, 
    // чтобы ядро не зависло намертво в глухом дедлоке.
    uint32_t timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    bool pll_locked = (timeout != 0);

    // === BAUDR: ОГРАНИЧЕНИЕ НА ТАКТИРОВАНИЕ ФЛЭША СТРОГО <= 48 МГц ===  
    // Меняем VFO_FLASH_SCK_MAX_HZ на жесткие 48 000 000 Гц
    const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;
    uint32_t ssi_baud = cfg->baudr;
    
    ssi_hw->ssienr = 0;                 // выключить SSI — иначе baudr не запишется  
    ssi_hw->baudr  = ssi_baud;  
    ssi_hw->ssienr = 1;                 // включить обратно  

    // возвращаем clk_sys -> PLL_SYS (целевая частота)  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    cfg->clk_hz, cfg->clk_hz);  
    
    // возвращаем прерывания
    restore_interrupts(ints);

}  
  
// ----------------------------------------------------------------------------  
// Разгон: VREG -> PLL -> верификация частотомером -> откат при сбое  
// ----------------------------------------------------------------------------  
void __not_in_flash_func(af_set_clk)(void) {  
    // перенастраиваем на разогнанную частоту
    reprogram_pll(&pll_cfg_boosted);
    // измеряем частоту шины
    current_clk_sys_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000UL;  
}

// Переход на номинал 125 МГц 
void __not_in_flash_func(af_set_clk_nominal)(void) {    
    // перенастраиваем на нормальную частоту
    reprogram_pll(&pll_cfg_nominal);
    // измеряем частоту шины
    current_clk_sys_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000UL; 
}
  
// ============================================================================  
// Arduino  
// ============================================================================  
static void core1_main(void);   // рабочий цикл Core 1, RAM-only  

void setup() {

    af_fm_init();              // образ диска ДО af_usb_init — первые READ'ы  
                               // хоста при монтировании должны получить FAT12  
    af_usb_init();             // монтируем диск, потом звук  

    Serial.begin(115200);
    af_led_init();             // heartbeat (WS2812 на GP16 у Zero)
    af_led_set(10, 0, 0);

    // Ждём подключения COM-порта (USB CDC), максимум 3 с.  
    // Как в радиомодеме: без USB-хоста ожидание не блокирует запуск.  
    while (!Serial && millis() < 3000) delay(1);  

    af_set_clk();              // разгон

    watchdog_enable(3000, true);  // сторожевой 3 с
    
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
    af_fm_sync();              // SET.TXT = актуальная таблица banks  
                               // (после store_init + gpio_ctl: cur финальный)  
  
    // Хост энумерировался в af_usb_init и закешировал dir с size=0.  
    // Пере-энумерация — перечитает FAT/каталог с реальным размером.  
    if (TinyUSBDevice.mounted()) {  
        TinyUSBDevice.detach();  
        delay(100);  
        TinyUSBDevice.attach();  
    }  
  
    af_capture_start();  
    af_pwm_init();
    af_voice_init();
    af_voice_play(VOICE_READY);   // приветствие после инициализации
    af_led_set(0, 10, 0);

    __dmb();  
    core0_ready = true;
    af_led_set(0, 0, 0);
}  
  
void loop() {
    const uint32_t t0 = micros();  

    af_usb_poll();      // первым: TX-FIFO TinyUSB — самый мелкий буфер тракта  
    af_led_poll();    
    af_cli_poll();  
    af_store_poll();    // запись дефолтных банков при пустом секторе (одноразово)
    af_fm_poll();       // отложенная регенерация SET.TXT (stale-флаг) 
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

  
// Рабочий цикл Core 1 — строго в RAM. Запрещено вызывать отсюда    
// (и из af_dsp_poll) любые функции без __not_in_flash_func и читать    
// const-данные/XIP-указатели — иначе зависание во время записи af_store.    
static void __not_in_flash_func(core1_main)(void) {
    for (;;) {  
        af_store_core1_park_poll();   // парковка по запросу af_store (RAM)  
        af_dsp_poll();                // вся DSP-цепочка — __not_in_flash_func  
    }  
}

// Спящий цикл Core 1 — строго в RAM: во время flash-окна XIP off,  
// фетч из flash = зависание SSI. tight_loop_contents() в setup1()  
// исполнялся бы из flash — поэтому отдельная __not_in_flash_func.  
static void __not_in_flash_func(core1_idle_spin)(void) {
    for (;;) {  
        af_store_core1_park_poll();   // чтобы ftest 1/3 могли парковать Core 1  
        tight_loop_contents();  
    }    
}  



void setup1() {
    const uint32_t t0 = millis();    
    while (!core0_ready && (uint32_t)(millis() - t0) < 10000u) {    
        af_store_core1_park_poll();    
        tight_loop_contents();    
    }    
    // Весь рабочий цикл Core 1 — в RAM. Инвариант: во время flash    
    // erase/program (af_store, ~45 мс, XIP off) Core 1 не должна    
    // обращаться к flash — ни кодом, ни данными. loop1() Arduino-ядра    
    // лежит во flash, поэтому крутим свой цикл из RAM-функции.    
    
    //core1_idle_spin();   // FLASH-TEST стенд: RAM-спин без DSP
    core1_main();   // не возвращается; внутри: park_poll + af_dsp_poll
}  

void loop1() {
    // Не используется: рабочий цикл Core 1 — core1_main() в RAM,  
    // вызывается из setup1(). Оставлена для Arduino-ядра.  
}  
