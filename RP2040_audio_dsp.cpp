// RP2040_audio_dsp.cpp  
//  
// Моноканальный НЧ аудио DSP-тракт радиостанции на RP2040.  
//  
// Тракт:  
//   ADC (GP26, 250 ksps, DMA) -> DC-EMA -> CIC/2 -> HB/2 -> HB/2 (31.25 ksps)  
//   -> полосовой FIR -> автонотч NT -> шумоподавление NR -> АРУ+лимитер  
//   -> выходная очередь -> PWM на GPIO + UAC-микрофон 44100/16  
//  
// Распределение по ядрам:  
//   Core 1: af_capture (DMA IRQ) + af_dsp_poll()  — весь DSP, real-time.  
//   Core 0: af_pwm_poll() + af_usb_poll() + af_cli_poll() + af_gpio_ctl_poll()  
//           + диагностика.  
//  
// Синхронизация запуска (как в RP2040_radiomodem):  
//   Core 0 регистрирует UAC-интерфейс и монтирует TinyUSB-стек ДО  
//   Serial.begin(), ждёт COM-порт, завершает инициализацию и только потом  
//   взводит core0_ready. Core 1 в setup1() ждёт этот флаг (таймаут 10 с)  
//   и лишь затем запускает захват ADC+DMA.  
  
#include <Arduino.h>  
#include <stdio.h>  
#include <string.h>  
#include "pico/stdlib.h"  
#include "hardware/clocks.h"
#include <hardware/vreg.h>
  
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
  
// ---------------------------------------------------------------------------  
// Флаг готовности Core 0. Core 1 в setup1() ждёт его перед стартом захвата.  
// ---------------------------------------------------------------------------  
volatile bool core0_ready = false;  
  
// ---------------------------------------------------------------------------  
// Core 0: setup()  
// ---------------------------------------------------------------------------  
void setup()  
{
    // Разгон: сначала поднимаем VREG до 1.20 В, потом тактовую.  
    // Порядок важен — на 1.10 В при 250 МГц плата не стартует.  
    vreg_set_voltage(VREG_VOLTAGE_1_25);  
    set_sys_clock_khz(220000, true);   // 133 -> 250 МГц (запас под NT+NR) 

    // UAC-интерфейс регистрируется ДО Serial.begin — как в радиомодеме:  
    // дескрипторы не должны меняться в окне энумерации, иначе  
    // «USB-устройство не опознано» / зависание.  
    // Внутри usb_out.begin() -> TinyUSBDevice.begin() — маунт стека.  
    af_usb_init();  
  
    Serial.begin(115200);   // CDC — последним, после регистрации UAC  
  
    // Ждём подключения COM-порта (USB CDC), максимум 5 с.  
    // Как в радиомодеме: без USB-хоста ожидание не блокирует запуск.  
    while (!Serial && millis() < 5000) delay(1);  
  
    Serial.println();  
    Serial.println("# RP2040_audio_dsp");  
    Serial.printf("# clk_sys=%lu Hz\n", (unsigned long)clock_get_hz(clk_sys));  
    Serial.printf("[UAC] 44100/16/stereo: %s\n",  
                  af_usb_mounted() ? "mounted" : "waiting host");  
  
    // Инициализация DSP (коэффициенты фильтров, состояние LMS/AGC).  
    // Должна быть выполнена ДО запуска захвата на Core 1.  
    af_dsp_init();  
  
    // Основной выход: PWM на GPIO.  
    af_pwm_init();
  
    // CLI по Serial (команды: band/nt/nr/stats/help).  
    af_cli_init(); 
    af_led_init();
    af_store_init();        // EEPROM + формат при первом старте  
    af_gpio_ctl_init();     // применит банк/фильтры по пинам
    af_capture_start();   // DMA_IRQ_0 живёт на Core 0
    af_voice_init();
  
    watchdog_enable(3000, true);   // WDT 3 с; true = пауза WDT при отладке  
    // Захват ADC+DMA стартует в setup1() на Core 1 —  
    // там же живёт DMA_IRQ_0 и DSP-поллинг.  
    Serial.println("# INIT_DONE (core0)");  
    Serial.flush();  

    af_voice_play(VOICE_READY);   // "радиотракт готов к работе"
  
    // Разрешаем Core 1 выйти из ожидания и запустить захват.  
    // Взводим ПОСЛЕДНИМ — вся инициализация Core 0 к этому моменту завершена.  
    __dmb();  
    core0_ready = true;  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: loop() — вывод звука, CLI, GPIO, диагностика  
// ---------------------------------------------------------------------------  
void loop()  
{  
    // ---- Учёт загрузки Core 0 (как в RP2040_receiver) ----  
    static uint32_t core0_busy_us         = 0;  
    static uint32_t core0_window_start_us = 0;  
    static uint32_t core0_load_permille   = 0;  
    static bool     core0_timer_init      = false;  
  
    uint32_t now = time_us_32();  
  
    if (!core0_timer_init) {  
        core0_window_start_us = now;  
        core0_timer_init      = true;  
    }  
  
    const uint32_t busy_start = now;  
  
    // --- Прикладные задачи Core 0 ---  

    watchdog_update();      // сброс WDT — Core 0 жив  
  
    // Выгрузка обработанных блоков в PWM-DMA.  
    af_pwm_poll();  
  
    // Выгрузка в UAC-микрофон (ресемплер 31250 -> 44100).  
    af_usb_poll();  
  
    // Serial CLI — опрос каждый проход, внутри свой разбор по строкам.  
    af_cli_poll();  
  
    // Опрос управляющих пинов (NT/NR/полоса), внутри свой таймер ~10 мс.  
    af_gpio_ctl_poll();

    // af_led_poll();
  
    // --- Конец измеряемой работы ---  
    now = time_us_32();  
    core0_busy_us += now - busy_start;  
  
    const uint32_t window_us = now - core0_window_start_us;  
    if (window_us >= 1000000u) {  
        uint32_t busy = core0_busy_us;  
        if (busy > window_us) busy = window_us;  
        core0_load_permille = (uint32_t)(((uint64_t)busy * 1000u) / window_us);  
        core0_busy_us         = 0;  
        core0_window_start_us = now;  
    }  
  
    // ---- Диагностика раз в секунду ----  
    static uint32_t t_dbg = 0;  
    if (millis() - t_dbg >= 60000) {  
        t_dbg = millis();  
  
        const uint32_t core1_load = af_dsp_load_permille();  
  
        Serial.printf(  
            "# CORE0_LOAD=%lu.%lu%% "  
            "CORE1_LOAD=%lu.%lu%% "  
            "cap_dropped=%lu "  
            "out_dropped=%lu "  
            "clip=%lu\n",  
            (unsigned long)(core0_load_permille / 10u),  
            (unsigned long)(core0_load_permille % 10u),  
            (unsigned long)(core1_load / 10u),  
            (unsigned long)(core1_load % 10u),  
            (unsigned long)af_capture_dropped(),  
            (unsigned long)af_dsp_out_dropped(),  
            (unsigned long)af_dsp_clip_count());  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Core 1: setup1() — ожидание Core 0, затем запуск захвата  
// ---------------------------------------------------------------------------  
void setup1()  
{  
    // Ждём, пока Core 0 поднимет USB и закончит инициализацию.  
    // Таймаут 10 с по времени (как в радиомодеме): счётный цикл на nop  
    // сгорает раньше, чем Core 0 поднимет флаг.  
    const uint32_t t0 = millis();  
    while (!core0_ready && (uint32_t)(millis() - t0) < 10000u) {
        af_store_core1_park_poll();   // отвечаем на park_core1 из af_store_init 
        tight_loop_contents();  
    }  
  

    // NB: Serial с Core 1 не используем (правило радиомодема —  
    // Core 1 никогда не блокируется). Старт виден по CORE1_LOAD > 0.  
}  
  
// ---------------------------------------------------------------------------  
// Core 1: loop1() — DSP-поллинг очереди блоков  
// ---------------------------------------------------------------------------  
void __not_in_flash_func(loop1)(void)  
{  
    // Сторож: если primask залип — разблокировать (порт из радиомодема).  
    static bool pm_checked = false;  
    if (!pm_checked) {  
        pm_checked = true;  
        uint32_t pm;  
        __asm volatile ("mrs %0, primask" : "=r"(pm));  
        if (pm) __asm volatile ("cpsie i" ::: "memory");  
    }  
  
    // Забирает заполненные ADC-блоки из SPSC-очереди, прогоняет  
    // весь DSP-тракт и публикует int16-блоки в выходную очередь.  
    // Внутри — учёт загрузки (af_dsp_load_permille).  
    af_dsp_poll();  
}