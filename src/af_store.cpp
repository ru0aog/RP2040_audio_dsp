// src/af_store.cpp  
// ============================================================================  
// af_store.cpp — банки настроек во flash (Core 0)  
//  
// Хранилище — последний сектор flash (4 КБ), тот же, что использует  
// EEPROM-эмуляция ядра Philhower. Чтение — через EEPROM (RAM-буфер),  
// запись — напрямую flash_range_erase/flash_range_program из RAM-функции    
// do_flash_write: multicore_reset_core1 + clk 125 МГц + IRQ off,    
// затем af_set_clk + restartCore1 (без ребута).    
//  
// Раскладка сектора:  
//   [0..1]   magic 'A','F'  
//   [2]      версия формата  
//   [3]      резерв  
//   [4..]    7 записей по sizeof(af_bank_rec_t)  
//  
// Запись — по 'save x' / 'banks clr' и один раз при старте, если сектор    
// пустой/чужой: af_store_poll пишет дефолтные банки (аналог banks clr).    
// Запись отложена в loop() — в boot-пути af_store_init она висла (BUGS.MD).    
// Чтение — по 'load x' и при смене кода на GP2..GP4.    
// ============================================================================  
#include <Arduino.h>  
#include <EEPROM.h>  
#include "af_store.h"  
#include "af_config.h"  
#include "af_dsp.h"  
#include "af_tx.h"  
#include "af_voice.h"
#include "af_led.h"
#include "af_capture.h"               // af_capture_pause/resume — DMA-окно  
  
#include "pico/stdlib.h"
#include "hardware/flash.h"  
#include "pico/flash.h"  
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
// #define AF_FLASH_OFF      0x1FF000u  
#define AF_FLASH_OFF  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) 

// Кооперативная парковка Core 1: idleOtherCore не работает, т.к. loop1  
// не возвращается в Arduino-обёртку и не обрабатывает FIFO-handshake.  
static volatile bool g_park_req = false;  
static volatile bool g_parked   = false;  

static uint8_t s_save_done = 0;   // idx успешного save — OK из poll 
  
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
  
// Сектор пустой/чужой — af_store_poll запишет дефолтные банки    
// на первой итерации loop (не в init: там запись висла).    
static bool s_need_defaults = false;    
  
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
        memset(s_sector_img, 0xFF, sizeof(s_sector_img));          
        Serial.println("[STORE] sector blank/foreign");        
    }          
  
    // Таблица банков — всегда из кода: перезаписываем записи      
    // дефолтами (RX-пресеты + TX-профили 1..7). Flash-ячейки      
    // не трогаем; save N запишет текущее, когда запись починится.      
    {      
        af_bank_t def[AF_STORE_BANKS];      
        af_store_fill_defaults(def);      
        s_sector_img[0] = AF_STORE_MAGIC0;      
        s_sector_img[1] = AF_STORE_MAGIC1;      
        s_sector_img[2] = AF_STORE_VER;      
        s_sector_img[3] = 0xFF;      
        for (uint8_t i = 1; i <= AF_STORE_BANKS; i++) {      
            af_bank_rec_t r;      
            r.d = def[i - 1];      
            r.crc = crc8(&r.d, sizeof(r.d));      
            memcpy(&s_sector_img[rec_addr(i)], &r, sizeof(r));      
        }      
    }      
    s_need_defaults = false;   // не писать дефолты во flash при старте      
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
  
// Снимок текущих настроек DSP в af_bank_t (полный снимок RX+TX).    
// Используется af_store_save и таблицей banks (столбец "cur").    
void af_store_snapshot(af_bank_t* out) {    
    // RX      
    out->lo_hz    = (uint16_t)af_dsp_band_lo_hz();      
    out->hi_hz    = (uint16_t)af_dsp_band_hi_hz();      
    out->cw_on    = af_dsp_cw_on() ? 1 : 0;      
    out->cw_f0    = (uint16_t)af_dsp_cw_freq();      
    out->cw_bw    = (uint16_t)af_dsp_cw_bw();      
    out->cw_noise = (uint16_t)af_dsp_cw_noise();    
    out->agc_on   = af_dsp_agc_on() ? 1 : 0;      
    out->gain_pct = (uint16_t)(af_dsp_gain() * 256.0f + 0.5f);    
    out->voice_vol = af_voice_volume();    
    out->agc_ref      = (uint16_t)af_dsp_agcref();      
    out->agc_max_x100 = (uint16_t)(af_dsp_agcmax() * 100.0f + 0.5f);      
    out->agc_min_x100 = (uint16_t)(af_dsp_agcmin() * 100.0f + 0.5f);      
    out->lim_ceil     = (uint16_t)af_dsp_limceil();      
    out->nr_depth     = (uint16_t)af_dsp_nrdepth();      
    out->nr_makeup    = (uint32_t)af_dsp_nrmakeup();      
    out->nt_mu        = (uint16_t)af_dsp_ntmu();      
    out->nr_mu        = (uint16_t)af_dsp_nrmu();      
    // TX      
    out->tx_lo_hz      = (uint16_t)af_tx_lo_hz();        
    out->tx_hi_hz      = (uint16_t)af_tx_hi_hz();        
    out->tx_ratio_x10  = (uint16_t)af_tx_ratio();      
    out->tx_clip       = (uint16_t)af_tx_clip();      
    out->tx_agc_target = (uint16_t)af_tx_agc();      
    out->tx_agc_max_x10= (uint16_t)af_tx_agcmax();      
    out->tx_gate_thr   = (uint16_t)af_tx_gate();      
    out->tx_gate_hold_ms  = (uint16_t)af_tx_gatehold();      
    out->tx_gate_hyst_x10 = (uint8_t)af_tx_gatehyst();      
    out->tx_gfloor     = (uint16_t)af_tx_gfloor();      
    out->tx_ap_on      = af_tx_ap() ? 1 : 0;      
    out->tx_softclip   = af_tx_softclip() ? 1 : 0;      
    out->tx_2band_on   = af_tx_2band() ? 1 : 0;      
    out->tx_split_hz   = (uint16_t)af_tx_split();      
    out->tx_lr2_on     = af_tx_lr2() ? 1 : 0;      
    out->tx_comp_thr    = (uint16_t)af_tx_compthr();      
    out->tx_comp_thr_lo = (uint16_t)af_tx_compthrlo();      
    out->tx_comp_thr_hi = (uint16_t)af_tx_compthrhi();      
    out->tx_duck_on    = af_tx_duck() ? 1 : 0;      
    out->tx_duckmax    = (uint16_t)af_tx_duckmax();      
    out->tx_intel_on   = af_tx_intel() ? 1 : 0;      
    out->tx_intel_tgt   = (uint16_t)af_tx_inteltgt();      
    out->tx_intel_cut   = (uint16_t)af_tx_intelcut();      
    out->tx_intel_boost = (uint16_t)af_tx_intelboost();      
    out->tx_pres_on    = af_tx_pres() ? 1 : 0;      
    out->tx_presmax    = (uint16_t)af_tx_presmax();      
    out->tx_pres_freq  = (uint16_t)af_tx_presfreq();      
    out->tx_pres_q     = (uint16_t)af_tx_presq();      
    out->tx_shelf_db   = (uint16_t)af_tx_shelf();      
    out->tx_shelf_freq = (uint16_t)af_tx_shelffreq();      
    out->tx_shelf_q    = (uint16_t)af_tx_shelfq();      
    out->tx_exc_on     = af_tx_exc() ? 1 : 0;      
    out->tx_excmix     = (uint16_t)af_tx_excmix();      
    out->tx_notch_on   = af_tx_notch() ? 1 : 0;      
    out->tx_notch1_hz  = (uint16_t)af_tx_notch1();      
    out->tx_notch2_hz  = (uint16_t)af_tx_notch2();      
}    
    
bool af_store_save(uint8_t idx) {    
    if (idx < 1 || idx > AF_STORE_BANKS) return false;    
    
    af_bank_rec_t r;      
    af_store_snapshot(&r.d);          // текущие настройки -> запись    
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

    Serial.println("# save: park core1"); Serial.flush();    
    if (!af_store_core1_park(5000)) {  
        Serial.println("# save: park TIMEOUT — abort");  
        return false;  
    }  
    Serial.println("# save: core1 parked"); Serial.flush(); 

    //Serial.println("# save: clk -> nominal"); Serial.flush();    
    //af_set_clk_nominal();    

    uint32_t ints = save_and_disable_interrupts();    
    do_flash_write(NULL);               // RAM: erase+program, XIP off    
    restore_interrupts(ints);    
    flash_flush_cache();                // сброс XIP-кэша после записи 
    
    //af_set_clk();  
    af_store_core1_unpark();
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


    Serial.println("# save: park core1"); Serial.flush();    
    if (!af_store_core1_park(5000)) {  
        Serial.println("# save: park TIMEOUT — abort");  
        return;  
    }  
    Serial.println("# save: core1 parked"); Serial.flush(); 

    Serial.println("# save: clk -> nominal"); Serial.flush();    
    af_set_clk_nominal();    

    uint32_t ints = save_and_disable_interrupts();    
    do_flash_write(NULL);               // RAM: erase+program, XIP off    
    restore_interrupts(ints);    
    flash_flush_cache();                // сброс XIP-кэша после записи 
    
    af_set_clk();  
    af_store_core1_unpark();
    Serial.println("# OK banks written");    
}

// ---------------------------------------------------------------------------    
// Дефолтные банки — те же пресеты, что в 'banks clr' (af_cli.cpp).    
// ---------------------------------------------------------------------------    
// Единая таблица дефолтов — используется и af_store_poll (пустой сектор),    
// и 'banks clr' в af_cli.cpp. Все поля af_bank_t заполняются явно.    
void af_store_fill_defaults(af_bank_t b[AF_STORE_BANKS]) {        
    memset(b, 0, sizeof(af_bank_t) * AF_STORE_BANKS);        
    for (int i = 0; i < AF_STORE_BANKS; i++) {        
        // RX — из af_config.h      
        b[i].lo_hz = 300;  b[i].hi_hz = 3000;      
        b[i].cw_on = 0;  b[i].cw_f0 = 700;  b[i].cw_bw = 150;  b[i].cw_noise = 400;      
        b[i].agc_on = 1;  b[i].gain_pct = 256;  b[i].voice_vol = 300;      
        b[i].agc_ref = AGC_REF;      
        b[i].agc_max_x100 = (uint16_t)(AGC_MAX_GAIN * 100.0f + 0.5f);      
        b[i].agc_min_x100 = (uint16_t)(AGC_MIN_GAIN * 100.0f + 0.5f);      
        b[i].lim_ceil = LIMIT_CEIL;      
        b[i].nr_depth = NR_DEPTH_Q15;  b[i].nr_makeup = NR_MAKEUP_Q15;      
        b[i].nt_mu = NOTCH_MU_Q;  b[i].nr_mu = NR_MU_Q;      
        // TX — рабочая точка (как базис tx_apply_profile)      
        b[i].tx_lo_hz = 300;  b[i].tx_hi_hz = 2700;      
        b[i].tx_ratio_x10 = 40;  b[i].tx_clip = 26000;      
        b[i].tx_agc_target = 18000;  b[i].tx_agc_max_x10 = AF_TX_AGC_MAX_X10;      
        b[i].tx_gate_thr = AF_TX_GATE_THR;      
        b[i].tx_gate_hold_ms = AF_TX_GATE_HOLD_MS;      
        b[i].tx_gate_hyst_x10 = AF_TX_GATE_HYST_X10;      
        b[i].tx_gfloor = AF_TX_GATE_FLOOR_Q16;      
        b[i].tx_ap_on = 1;  b[i].tx_softclip = 1;  b[i].tx_2band_on = 1;      
        b[i].tx_split_hz = AF_TX_SPLIT_HZ;  b[i].tx_lr2_on = 1;      
        b[i].tx_comp_thr = 8192;      
        b[i].tx_comp_thr_lo = AF_TX_COMP_THR_LO;      
        b[i].tx_comp_thr_hi = AF_TX_COMP_THR_HI;      
        b[i].tx_duck_on = 1;  b[i].tx_duckmax = 30;      
        b[i].tx_intel_on = 1;      
        b[i].tx_intel_tgt = AF_TX_INTEL_TGT_Q15;      
        b[i].tx_intel_cut = AF_TX_INTEL_CUT_Q15;      
        b[i].tx_intel_boost = AF_TX_INTEL_BOOST_Q15;      
        b[i].tx_pres_on = 1;  b[i].tx_presmax = 15;      
        b[i].tx_pres_freq = AF_TX_PRES_FREQ_HZ;  b[i].tx_pres_q = AF_TX_PRES_Q_X100;      
        b[i].tx_shelf_db = 0;      
        b[i].tx_shelf_freq = AF_TX_SHELF_FREQ_HZ;  b[i].tx_shelf_q = AF_TX_SHELF_Q_X100;      
        b[i].tx_exc_on = 1;  b[i].tx_excmix = 3277;      
        b[i].tx_notch_on = 1;      
        b[i].tx_notch1_hz = AF_TX_NOTCH1_HZ;  b[i].tx_notch2_hz = AF_TX_NOTCH2_HZ;
                // TX — профиль i+1 (1 flat .. 7 max), как tx_apply_profile в af_cli.cpp.  
        // Общее для всех профилей:  
        b[i].tx_softclip = 1;  
        b[i].tx_split_hz = 900;  
        b[i].tx_comp_thr = 8192;  
        b[i].tx_gate_hyst_x10 = 7;  
        b[i].tx_pres_freq = 1900;  b[i].tx_pres_q = 90;  
        b[i].tx_shelf_freq = 1800; b[i].tx_shelf_q = 70;  
        b[i].tx_notch1_hz = 1000;  b[i].tx_notch2_hz = 2000;  
        b[i].tx_intel_tgt = 15400; b[i].tx_intel_cut = 16384; b[i].tx_intel_boost = 32768;  
  
        // Per-профиль отличия — таблица по индексу банка:  
        //   {lo,hi, ratio,clip, agc,agcmax, gate,hold,gfloor, ap,2band,lr2,  
        //    cthr_lo,cthr_hi, pres,pmax, exc,emix, duck,dmax, intel, shelf, notch}  
        static const struct {  
            uint16_t lo, hi, ratio, clip, agc, agcmax, gate, hold, gfloor;  
            uint8_t  ap, b2, lr2;  
            uint16_t cthr_lo, cthr_hi;  
            uint8_t  pres;  uint16_t pmax;  
            uint8_t  exc;   uint16_t emix;  
            uint8_t  duck;  uint16_t dmax;  
            uint8_t  intel; uint16_t shelf; uint8_t notch;  
        } TXP[AF_STORE_BANKS] = {  
            // 1 FLAT — линейный тракт  
            { 300,2700, 10,32767, 18000,200,   0,  0, 65535, 0,0,0, 8192,6144, 0, 0, 0,3277, 0,30, 0,  0, 0 },  
            // 2 NATURAL — мягкий повседневный  
            { 300,2700, 30,20000, 16000,200, 250,180, 4120, 1,1,1, 8192,6144, 1,15, 0,3277, 0,30, 1,  0, 1 },  
            // 3 DX — пробив на слабом сигнале  
            { 300,2700, 60,12000, 18000,200, 250,200, 4120, 1,1,1, 8192,6144, 1,30, 1,3277, 1,30, 1, 20, 1 },  
            // 4 BASS — низкий голос  
            { 350,2700, 40,26000, 16000,200, 250,180, 4120, 1,1,1, 8192,6144, 1,20, 0,3277, 1,40, 1, 10, 1 },  
            // 5 WHISPER — тихая речь  
            { 300,2700, 50,16000, 20000,200, 150,250, 4120, 1,1,1, 8192,6144, 1,30, 1,1638, 0,30, 1,  0, 1 },  
            // 6 LOUD — громкий голос  
            { 300,2700, 40,24000, 14000,200, 350,150, 4120, 1,1,1, 8192,6144, 1,15, 0,3277, 1,30, 1,  0, 1 },  
            // 7 MAX — максимум плотности  
            { 300,2400, 80, 6144, 18000,300, 250,180, 4120, 1,1,1, 6144,6144, 1,30, 1,3277, 1,30, 1, 40, 1 },  
        };  
        b[i].tx_lo_hz = TXP[i].lo;        b[i].tx_hi_hz = TXP[i].hi;  
        b[i].tx_ratio_x10 = TXP[i].ratio; b[i].tx_clip = TXP[i].clip;  
        b[i].tx_agc_target = TXP[i].agc;  b[i].tx_agc_max_x10 = TXP[i].agcmax;  
        b[i].tx_gate_thr = TXP[i].gate;   b[i].tx_gate_hold_ms = TXP[i].hold;  
        b[i].tx_gfloor = TXP[i].gfloor;  
        b[i].tx_ap_on = TXP[i].ap;        b[i].tx_2band_on = TXP[i].b2;  
        b[i].tx_lr2_on = TXP[i].lr2;  
        b[i].tx_comp_thr_lo = TXP[i].cthr_lo;  b[i].tx_comp_thr_hi = TXP[i].cthr_hi;  
        b[i].tx_pres_on = TXP[i].pres;    b[i].tx_presmax = TXP[i].pmax;  
        b[i].tx_exc_on = TXP[i].exc;      b[i].tx_excmix = TXP[i].emix;  
        b[i].tx_duck_on = TXP[i].duck;    b[i].tx_duckmax = TXP[i].dmax;  
        b[i].tx_intel_on = TXP[i].intel;  
        b[i].tx_shelf_db = TXP[i].shelf;  
        b[i].tx_notch_on = TXP[i].notch;  
    }        
    // Пресеты полос RX — как в 'banks clr':        
    b[0].lo_hz = 100;  b[0].hi_hz = 3100;   // 1: SSB широкая        
    b[1].lo_hz = 300;  b[1].hi_hz = 2200;   // 2: SSB узкая        
    b[2].lo_hz = 100;  b[2].hi_hz = 4000;   // 3: широкая        
    b[3].lo_hz = 450;  b[3].hi_hz = 950;        
    b[3].cw_on = 1;                         // 4: CW 700 Гц        
    b[4].lo_hz = 300;  b[4].hi_hz = 900;    // 5: резерв узкая        
    b[5].lo_hz = 300;  b[5].hi_hz = 3000;   // 6: AM, АРУ off        
    b[5].agc_on = 0;   b[5].gain_pct = 384;        
    b[6].lo_hz = 200;  b[6].hi_hz = 3500;   // 7: резерв        
}        
  
// Записать дефолтные банки — вызывается из af_store_poll при пустом секторе.    
static void af_store_write_defaults(void) {    
    af_bank_t preset[AF_STORE_BANKS];    
    af_store_fill_defaults(preset);      
    Serial.println("# store: no banks — writing defaults");    
    Serial.flush();    
    af_store_write_all(preset);   // reset Core1 + erase/program + restartCore1    
}    
  
// Core 0, из loop(): отложенная запись дефолтных банков при пустом секторе.    
void af_store_poll(void) {      
    if (s_save_done) {      
        Serial.printf("# OK save=%d\n", s_save_done);      
        s_save_done = 0;      
    }      
    if (!s_need_defaults) return;      
    s_need_defaults = false;      
    af_store_write_defaults();      
}

// ============================================================================  
// FLASH-TEST: отработка механизма записи в изоляции.  
// Тестовый сектор 0x100000 — свободная зона по fmap (0x0ef000..0x13ffff).  
// Не пересекается ни с прошивкой, ни с банками (0x1ff000), ни с rec-слотом.  
// Четыре варианта окна — чтобы найти, какой фактор вносит зависание:  
//   mode 0: чистое окно (IRQ-off -> erase -> program -> IRQ-on)  
//   mode 1: + парковка Core 1 (af_store_core1_park/unpark)  
//   mode 2: + даунклок (af_set_clk_nominal -> окно -> af_set_clk)  
//   mode 3: парковка + даунклок (полная схема, как af_store_save)  
// ============================================================================  
  
#define AF_TEST_FLASH_OFF   0x100000u  
#define AF_TEST_MAGIC0      'T'  
#define AF_TEST_MAGIC1      'E'  
  
static uint8_t s_test_img[FLASH_SECTOR_SIZE];  
  
// Окно записи тестового сектора — вся в RAM, как do_flash_write.  
static void __not_in_flash_func(do_test_write)(void* param) {  
    (void)param;  
    flash_range_erase(AF_TEST_FLASH_OFF, FLASH_SECTOR_SIZE);  
    flash_range_program(AF_TEST_FLASH_OFF, s_test_img, FLASH_SECTOR_SIZE);  
}  
  
// Одна итерация: заполнить образ паттерном, записать, верифицировать.  
// Возвращает true при совпадении readback.  
static bool af_store_test_iter(uint8_t mode, uint32_t iter) {  
    // Паттерн: сигнатура + номер итерации + пилообразные данные.  
    s_test_img[0] = AF_TEST_MAGIC0;  
    s_test_img[1] = AF_TEST_MAGIC1;  
    s_test_img[2] = (uint8_t)(iter & 0xFF);  
    s_test_img[3] = (uint8_t)(iter >> 8);  
    for (uint32_t i = 4; i < FLASH_SECTOR_SIZE; i++) {  
        s_test_img[i] = (uint8_t)(i + iter);   // детерминированный паттерн  
    }  
  
    bool parked = false;  
    if (mode == 1 || mode == 3) {  
        if (!af_store_core1_park(5000)) {  
            Serial.println("# ftest: park TIMEOUT");  
            return false;  
        }  
        parked = true;  
    }  
    if (mode == 2 || mode == 3) {  
        af_set_clk_nominal();  
    }  
  
    af_capture_pause();               // DMA-окно: ADC+DMA стоп до XIP-off  
    uint32_t ints = save_and_disable_interrupts();    
    do_test_write(NULL);    
    flash_flush_cache();              // ДО restore_interrupts: иначе pending  
                                      // DMA IRQ прыгнет в ISR на протухшем  
                                      // XIP-кэше -> вис (гипотеза проверяется)  
    restore_interrupts(ints);    
    af_capture_resume();              // рестарт цепочки A->B->A  
  
    if (mode == 2 || mode == 3) {  
        af_set_clk();  
    }  
    if (parked) {  
        af_store_core1_unpark();  
    }  
  
    // Верификация: readback через XIP, сравнение с образом.  
    const uint8_t* rb = (const uint8_t*)(XIP_BASE + AF_TEST_FLASH_OFF);  
    if (rb[0] != AF_TEST_MAGIC0 || rb[1] != AF_TEST_MAGIC1) return false;  
    for (uint32_t i = 4; i < FLASH_SECTOR_SIZE; i++) {  
        if (rb[i] != (uint8_t)(i + iter)) return false;  
    }  
    return true;  
}  
  
// Публичный вход из CLI: af_store_test(mode, iters).  
// mode 0..3 (см. выше), iters — число циклов erase/program/verify.  
void af_store_test(uint8_t mode, uint32_t iters) {  
    Serial.printf("# ftest: mode=%u iters=%lu off=0x%x\n",  
                  (unsigned)mode, (unsigned long)iters, (unsigned)AF_TEST_FLASH_OFF);  
    Serial.flush();  
    uint32_t fails = 0;  
    for (uint32_t i = 0; i < iters; i++) {  
        if (!af_store_test_iter(mode, i)) {  
            fails++;  
            Serial.printf("# ftest: iter %lu FAIL\n", (unsigned long)i);  
            Serial.flush();  
        } else if ((i % 10) == 0 || i == iters - 1) {  
            Serial.printf("# ftest: iter %lu ok\n", (unsigned long)i);  
            Serial.flush();  
        }  
    }  
    Serial.printf("# ftest: done, %lu/%lu ok\n",  
                  (unsigned long)(iters - fails), (unsigned long)iters);  
    Serial.flush();  
}
// =======================   КОНЕЦ БЛОКА ТЕСТОВ   =============================
