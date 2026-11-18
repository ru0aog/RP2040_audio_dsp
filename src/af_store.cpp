// src/af_store.cpp  
// ============================================================================  
// af_store.cpp — банки настроек во flash (Core 0)  
//  
// Хранилище — последний сектор flash (4 КБ), тот же, что использует  
// EEPROM-эмуляция ядра Philhower. Чтение — через EEPROM (RAM-буфер),  
// запись — напрямую flash_range_erase/flash_range_program через  
// flash_safe_execute(): он сам паркует Core 1 и исполняет запись из RAM.  
// Это устраняет зависания EEPROM.commit() при разгоне clk_sys до 220 МГц.  
//  
// Раскладка сектора:  
//   [0..1]   magic 'A','F'  
//   [2]      версия формата  
//   [3]      резерв  
//   [4..]    7 записей по sizeof(af_bank_rec_t)  
//  
// Запись — только по команде 'save x' (flash ~100к циклов стирания,  
// автосохранения нет). Чтение — по 'load x' и при смене кода на GP2..GP4.  
// ============================================================================  
#include <Arduino.h>  
#include <EEPROM.h>  
#include "af_store.h"  
#include "af_config.h"  
#include "af_dsp.h"  
#include "af_tx.h"  
#include "af_voice.h"  
  
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/watchdog.h"
#include "hardware/flash.h"
#include "pico/multicore.h"
  
#define AF_STORE_MAGIC0   'A'    
#define AF_STORE_MAGIC1   'F'    
#define AF_STORE_VER      4   // было 3: банк = полный снимок RX+TX  
#define AF_EEPROM_SIZE    1024  // 7 записей по ~90 байт + заголовок  
  
// Смещение сектора EEPROM от начала flash (последний сектор).
// #define AF_FLASH_OFF      (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  
// Фиксированный адрес: последний сектор 2-МБ flash. Работает и на 4-МБ  
// платах (сектор внутри чипа). НЕ привязывать к PICO_FLASH_SIZE_BYTES —  
// при сборке под 4 МБ адрес уедет за пределы 2-МБ чипа.  
#define AF_FLASH_OFF      0x1FF000u  

// Кооперативная парковка Core 1: idleOtherCore не работает, т.к. loop1  
// не возвращается в Arduino-обёртку и не обрабатывает FIFO-handshake.  
static volatile bool g_park_req = false;  
static volatile bool g_parked   = false;  
  
// Вызывается с Core 1 из af_dsp_poll. Вся функция в RAM — во время  
// erase/program flash недоступен, spin-цикл обязан быть не во flash.  
void __not_in_flash_func(af_store_core1_park_poll)(void) {    
    if (!g_park_req) return;    
    uint32_t irq = save_and_disable_interrupts();  // IRQ Core 1 off —    
                                                   // ISR лежит во flash    
    g_parked = true;    
    __dmb();    
    while (g_park_req) { /* spin в RAM, без IRQ */ }    
    g_parked = false;    
    restore_interrupts(irq);    
}
  
// Core 0: запросить парковку, ждать ack с таймаутом.    
static bool park_core1(void) {      
    g_park_req = true;      
    __dmb();      
    uint32_t t0 = millis();      
    while (!g_parked) {      
        if (millis() - t0 > 500) { g_park_req = false; return false; }      
    }      
    return true;      
}    
  
// Экспорт для af_rec: припарковать Core 1 перед flash-операцией.    
// Возвращает false по таймауту — Core 1 продолжает работать.    
bool af_store_core1_park(uint32_t timeout_ms) {    
    g_park_req = true;    
    __dmb();    
    uint32_t t0 = millis();    
    while (!g_parked) {    
        if (millis() - t0 > timeout_ms) { g_park_req = false; return false; }    
    }    
    return true;    
}    
  
// Отпустить Core 1 — spin в af_store_core1_park_poll завершится,    
// Core 1 вернётся в af_dsp_poll.    
void af_store_core1_unpark(void) {    
    g_park_req = false;    
    __dmb();    
}    
  
#pragma pack(push, 1)  
struct af_bank_rec_t {  
    af_bank_t d;  
    uint8_t   crc;        // XOR всех байт d; 0xFF-пустая ячейка имеет crc=0xFF  
};  
#pragma pack(pop)  
  
// Образ всего сектора — читаем, патчим, пишем целиком.  
static uint8_t s_sector_img[FLASH_SECTOR_SIZE];  
  
static uint8_t crc8(const void* p, size_t n) {  
    const uint8_t* b = (const uint8_t*)p;  
    uint8_t c = 0;  
    for (size_t i = 0; i < n; i++) c ^= b[i];  
    return c;  
}  
  
static int rec_addr(uint8_t idx) {           // idx = 1..7  
    return 4 + (idx - 1) * (int)sizeof(af_bank_rec_t);  
}  

// ---------------------------------------------------------------------------  
// Запись сектора во flash. Выполняется внутри flash_safe_execute —  
// Core 1 припаркован, код исполняется из RAM, flash недоступен для чтения.  
// ---------------------------------------------------------------------------  
// Вся функция в RAM: во время erase/program XIP недоступен,    
// вызов из flash-кода = зависание Core 0.    
static void __not_in_flash_func(do_flash_write)(void* param) {    
    (void)param;    
    flash_range_erase(AF_FLASH_OFF, FLASH_SECTOR_SIZE);    
    flash_range_program(AF_FLASH_OFF, s_sector_img, FLASH_SECTOR_SIZE);    
}    
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void af_store_init(void) {      
    // Читаем НАШ сектор напрямую из XIP — не через EEPROM.    
    memcpy(s_sector_img, (const void*)(XIP_BASE + AF_FLASH_OFF), FLASH_SECTOR_SIZE);    
    
    // Диагностика: что лежит в секторе и куда смотрим.    
    Serial.printf("# store: off=0x%x img=%02x %02x %02x %02x | rec[1]=%02x %02x\n",    
        (unsigned)AF_FLASH_OFF,    
        s_sector_img[0], s_sector_img[1], s_sector_img[2], s_sector_img[3],    
        s_sector_img[rec_addr(1)], s_sector_img[rec_addr(1) + 1]);    
    Serial.printf("# store: PICO_FLASH_SIZE_BYTES=0x%x\n",    
        (unsigned)PICO_FLASH_SIZE_BYTES);    
    
    if (s_sector_img[0] != AF_STORE_MAGIC0 ||      
        s_sector_img[1] != AF_STORE_MAGIC1 ||      
        s_sector_img[2] != AF_STORE_VER) {      
        // Boot-путь — только чтение. Никакой записи во flash здесь:  
        // park/erase на этапе init периодически вешает плату.  
        // Пустой/битый сектор = пустые банки; формат — через 'banks clr'.  
        memset(s_sector_img, 0xFF, sizeof(s_sector_img));      
        Serial.println("[STORE] sector blank/foreign — banks empty (use 'banks clr')");    
    }      
}  
  
bool af_store_read(uint8_t idx, af_bank_t* out) {    
    if (idx < 1 || idx > AF_STORE_BANKS) return false;    
    // Читаем из s_sector_img — наш сектор AF_FLASH_OFF, заполнен    
    // af_store_init напрямую из XIP. EEPROM.get читал бы чужой    
    // сектор (PICO_FLASH_SIZE_BYTES - 4096) — на 2-МБ плате он    
    // не совпадает с AF_FLASH_OFF, и банки всегда "empty".    
    af_bank_rec_t r;    
    memcpy(&r, &s_sector_img[rec_addr(idx)], sizeof(r));    
    if (r.crc == 0xFF) return false;                       // пустая    
    if (r.crc != crc8(&r.d, sizeof(r.d))) return false;    // битая    
    *out = r.d;    
    return true;    
}  
  
bool af_store_save(uint8_t idx) {  
    if (idx < 1 || idx > AF_STORE_BANKS) return false;  
  
    // Текущие настройки DSP -> запись (полный снимок RX+TX).    
    af_bank_rec_t r;    
    // RX    
    r.d.lo_hz    = (uint16_t)af_dsp_band_lo_hz();    
    r.d.hi_hz    = (uint16_t)af_dsp_band_hi_hz();    
    r.d.cw_on    = af_dsp_cw_on() ? 1 : 0;    
    r.d.cw_f0    = (uint16_t)af_dsp_cw_freq();    
    r.d.cw_bw    = (uint16_t)af_dsp_cw_bw();    
    r.d.cw_noise = (uint16_t)af_dsp_cw_noise();  
    r.d.agc_on   = af_dsp_agc_on() ? 1 : 0;    
    r.d.gain_pct = (uint16_t)(af_dsp_gain() * 256.0f + 0.5f);  
    r.d.voice_vol = af_voice_volume();  
    r.d.agc_ref      = (uint16_t)af_dsp_agcref();    
    r.d.agc_max_x100 = (uint16_t)(af_dsp_agcmax() * 100.0f + 0.5f);    
    r.d.agc_min_x100 = (uint16_t)(af_dsp_agcmin() * 100.0f + 0.5f);    
    r.d.lim_ceil     = (uint16_t)af_dsp_limceil();    
    r.d.nr_depth     = (uint16_t)af_dsp_nrdepth();    
    r.d.nr_makeup    = (uint32_t)af_dsp_nrmakeup();    
    r.d.nt_mu        = (uint16_t)af_dsp_ntmu();    
    r.d.nr_mu        = (uint16_t)af_dsp_nrmu();    
    // TX    
    r.d.tx_lo_hz      = (uint16_t)af_tx_lo_hz();      
    r.d.tx_hi_hz      = (uint16_t)af_tx_hi_hz();      
    r.d.tx_ratio_x10  = (uint16_t)af_tx_ratio();    
    r.d.tx_clip       = (uint16_t)af_tx_clip();    
    r.d.tx_agc_target = (uint16_t)af_tx_agc();    
    r.d.tx_agc_max_x10= (uint16_t)af_tx_agcmax();    
    r.d.tx_gate_thr   = (uint16_t)af_tx_gate();    
    r.d.tx_gate_hold_ms  = (uint16_t)af_tx_gatehold();    
    r.d.tx_gate_hyst_x10 = (uint8_t)af_tx_gatehyst();    
    r.d.tx_gfloor     = (uint16_t)af_tx_gfloor();    
    r.d.tx_ap_on      = af_tx_ap() ? 1 : 0;    
    r.d.tx_softclip   = af_tx_softclip() ? 1 : 0;    
    r.d.tx_2band_on   = af_tx_2band() ? 1 : 0;    
    r.d.tx_split_hz   = (uint16_t)af_tx_split();    
    r.d.tx_lr2_on     = af_tx_lr2() ? 1 : 0;    
    r.d.tx_comp_thr    = (uint16_t)af_tx_compthr();    
    r.d.tx_comp_thr_lo = (uint16_t)af_tx_compthrlo();    
    r.d.tx_comp_thr_hi = (uint16_t)af_tx_compthrhi();    
    r.d.tx_duck_on    = af_tx_duck() ? 1 : 0;    
    r.d.tx_duckmax    = (uint16_t)af_tx_duckmax();    
    r.d.tx_intel_on   = af_tx_intel() ? 1 : 0;    
    r.d.tx_intel_tgt   = (uint16_t)af_tx_inteltgt();    
    r.d.tx_intel_cut   = (uint16_t)af_tx_intelcut();    
    r.d.tx_intel_boost = (uint16_t)af_tx_intelboost();    
    r.d.tx_pres_on    = af_tx_pres() ? 1 : 0;    
    r.d.tx_presmax    = (uint16_t)af_tx_presmax();    
    r.d.tx_pres_freq  = (uint16_t)af_tx_presfreq();    
    r.d.tx_pres_q     = (uint16_t)af_tx_presq();    
    r.d.tx_shelf_db   = (uint16_t)af_tx_shelf();    
    r.d.tx_shelf_freq = (uint16_t)af_tx_shelffreq();    
    r.d.tx_shelf_q    = (uint16_t)af_tx_shelfq();    
    r.d.tx_exc_on     = af_tx_exc() ? 1 : 0;    
    r.d.tx_excmix     = (uint16_t)af_tx_excmix();    
    r.d.tx_notch_on   = af_tx_notch() ? 1 : 0;    
    r.d.tx_notch1_hz  = (uint16_t)af_tx_notch1();    
    r.d.tx_notch2_hz  = (uint16_t)af_tx_notch2();    
    r.crc = crc8(&r.d, sizeof(r.d));    
  
    // Читаем текущий сектор из flash напрямую (XIP), патчим запись.    
    memcpy(s_sector_img, (const void*)(XIP_BASE + AF_FLASH_OFF), sizeof(s_sector_img));    
    // Гарантируем заголовок: если сектор был пустой/чужой, без magic    
    // af_store_init при следующем boot сотрёт образ — банк потеряется.    
    s_sector_img[0] = AF_STORE_MAGIC0;    
    s_sector_img[1] = AF_STORE_MAGIC1;    
    s_sector_img[2] = AF_STORE_VER;    
    s_sector_img[3] = 0xFF;    
    memcpy(s_sector_img + rec_addr(idx), &r, sizeof(r));     

    // Как в af_store_write_all: жёсткий reset Core 1 — детерминированно,  
    // без гонки с флаговой парковкой. Восстанавливать не нужно — дальше reboot.
    multicore_reset_core1();  
    busy_wait_us(100);  
  
    set_sys_clock_khz(125000, false);   // номинал, как vfo_clk_boost_exit  

    uint32_t ints = save_and_disable_interrupts();    
    do_flash_write(NULL);               // RAM: erase+program, XIP off    
    restore_interrupts(ints);    
    flash_flush_cache();                // сброс XIP-кэша после записи    
  
    // Возврат без ребута: разгон + перезапуск Core 1.    
    af_set_clk();                       // обратно на 250 МГц (PLL + vreg + verify)    
    rp2040.restartCore1();              // Core 1: setup1 -> loop1 
  
    Serial.printf("# OK save=%d\n", idx);      
    return true;  
}  
  
bool af_store_load(uint8_t idx) {  
    af_bank_t b;  
    if (!af_store_read(idx, &b)) return false;  
  
    // Порядок важен: сначала CW-параметры и флаг (при cw on полоса    
    // подтянется к f0±250), затем полоса — при cw off она встанет как    
    // сохранённая, при cw on set_band отклонится и сработает follow.    
    af_dsp_set_cw_freq(b.cw_f0);    
    af_dsp_set_cw_bw(b.cw_bw);    
    af_dsp_set_cw_noise(b.cw_noise);  
    af_dsp_set_agc(b.agc_on != 0);    
    af_dsp_set_gain((float)b.gain_pct / 256.0f);  
    af_voice_set_volume(b.voice_vol);  
    af_dsp_set_cw(b.cw_on != 0);    
    if (!b.cw_on) af_dsp_set_band(b.lo_hz, b.hi_hz);    
    // RX: АРУ/лимитер/NR-параметры (on/off NT/NR/rxnotch не трогаем —    
    // они включаются по обстановке с GP5/GP6 и консоли).    
    af_dsp_set_agcref((int32_t)b.agc_ref);    
    af_dsp_set_agcmax((float)b.agc_max_x100 / 100.0f);    
    af_dsp_set_agcmin((float)b.agc_min_x100 / 100.0f);    
    af_dsp_set_limceil((int32_t)b.lim_ceil);    
    af_dsp_set_nrdepth((int32_t)b.nr_depth);    
    af_dsp_set_nrmakeup((int32_t)b.nr_makeup);    
    af_dsp_set_ntmu((int32_t)b.nt_mu);    
    af_dsp_set_nrmu((int32_t)b.nr_mu);    
    // TX: весь тракт. Порядок — как в tx_apply_profile: сначала    
    // параметры, от которых зависят пересчёты (split, presfreq/q,    
    // shelffreq/q, notchf), затем on/off и уровни.    
    af_tx_set_band(b.tx_lo_hz, b.tx_hi_hz);    
    af_tx_set_ratio(b.tx_ratio_x10);    
    af_tx_set_clip(b.tx_clip);    
    af_tx_set_agc(b.tx_agc_target);    
    af_tx_set_agcmax(b.tx_agc_max_x10);    
    af_tx_set_gate(b.tx_gate_thr);    
    af_tx_set_gatehold(b.tx_gate_hold_ms);    
    af_tx_set_gatehyst(b.tx_gate_hyst_x10);    
    af_tx_set_gfloor(b.tx_gfloor);    
    af_tx_set_ap(b.tx_ap_on != 0);    
    af_tx_set_softclip(b.tx_softclip != 0);    
    af_tx_set_2band(b.tx_2band_on != 0);    
    af_tx_set_split(b.tx_split_hz);    
    af_tx_set_lr2(b.tx_lr2_on != 0);    
    af_tx_set_compthr((int32_t)b.tx_comp_thr);    
    af_tx_set_compthrlo((int32_t)b.tx_comp_thr_lo);    
    af_tx_set_compthrhi((int32_t)b.tx_comp_thr_hi);    
    af_tx_set_duck(b.tx_duck_on != 0);    
    af_tx_set_duckmax(b.tx_duckmax);    
    af_tx_set_intel(b.tx_intel_on != 0);    
    af_tx_set_inteltgt((int32_t)b.tx_intel_tgt);    
    af_tx_set_intelcut((int32_t)b.tx_intel_cut);    
    af_tx_set_intelboost((int32_t)b.tx_intel_boost);    
    af_tx_set_pres(b.tx_pres_on != 0);    
    af_tx_set_presmax(b.tx_presmax);    
    af_tx_set_presfreq(b.tx_pres_freq);    
    af_tx_set_presq(b.tx_pres_q);    
    af_tx_set_shelffreq(b.tx_shelf_freq);    
    af_tx_set_shelfq(b.tx_shelf_q);    
    af_tx_set_shelf(b.tx_shelf_db);       // после freq/q — пересчёт биквада    
    af_tx_set_exc(b.tx_exc_on != 0);    
    af_tx_set_excmix(b.tx_excmix);    
    af_tx_set_notch(b.tx_notch_on != 0);    
    af_tx_set_notchf(b.tx_notch1_hz, b.tx_notch2_hz);    
    return true;    
}

void af_store_write_all(const af_bank_t banks[AF_STORE_BANKS]) {  
    // Читаем текущий сектор, патчим все записи, пишем целиком.  
    memcpy(s_sector_img, (const void*)(XIP_BASE + AF_FLASH_OFF), FLASH_SECTOR_SIZE);  
  
    s_sector_img[0] = 'A';  
    s_sector_img[1] = 'F';  
    s_sector_img[2] = AF_STORE_VER;  
    s_sector_img[3] = 0xFF;  
  
    for (uint8_t i = 1; i <= AF_STORE_BANKS; i++) {  
        af_bank_rec_t r;  
        r.d = banks[i - 1];  
        r.crc = crc8((const uint8_t*)&r.d, sizeof(r.d));  
        memcpy(&s_sector_img[rec_addr(i)], &r, sizeof(r));  
    }  
  
    // Как в маяке: жёсткий сброс Core 1 вместо флаговой парковки —  
    // 100% гарантия, что Core 1 не трогает flash во время erase.  
    multicore_reset_core1();  
    busy_wait_us(100);  
  
    set_sys_clock_khz(125000, false);   // номинал, как vfo_clk_boost_exit  
  
    uint32_t ints = save_and_disable_interrupts();    
    do_flash_write(NULL);               // RAM: erase+program, XIP off    
    restore_interrupts(ints);    
    flash_flush_cache();                // сброс XIP-кэша после записи    
    
    // Возврат без ребута: разгон + перезапуск Core 1.    
    af_set_clk();                       // обратно на 250 МГц (PLL + vreg + verify)    
    rp2040.restartCore1();              // Core 1: setup1 -> loop1 
    
    Serial.println("# OK banks written");    
}
