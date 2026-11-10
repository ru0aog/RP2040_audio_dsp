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
#include "af_voice.h"
  
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/watchdog.h"
  
#define AF_STORE_MAGIC0   'A'  
#define AF_STORE_MAGIC1   'F'  
#define AF_STORE_VER      3   // было 2: добавлено voice_vol
#define AF_EEPROM_SIZE    256
  
// Смещение сектора EEPROM от начала flash (последний сектор).  
#define AF_FLASH_OFF      (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  

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
static void do_flash_write(void* param) {  
    (void)param;  
    flash_range_erase(AF_FLASH_OFF, FLASH_SECTOR_SIZE);  
    flash_range_program(AF_FLASH_OFF, s_sector_img, FLASH_SECTOR_SIZE);  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void af_store_init(void) {  
    EEPROM.begin(AF_EEPROM_SIZE);  
    // Если magic не совпал — форматируем: все ячейки пустые (0xFF).  
    if (EEPROM.read(0) != AF_STORE_MAGIC0 ||  
        EEPROM.read(1) != AF_STORE_MAGIC1 ||  
        EEPROM.read(2) != AF_STORE_VER) {  
        memset(s_sector_img, 0xFF, sizeof(s_sector_img));  
        s_sector_img[0] = AF_STORE_MAGIC0;  
        s_sector_img[1] = AF_STORE_MAGIC1;  
        s_sector_img[2] = AF_STORE_VER;  
        // Паркуем Core 1 — во время erase/program flash недоступна,  
        // а af_dsp_poll исполняется из неё. Без парковки Core 1 виснет.  
        if (!park_core1()) {  
            Serial.println("[STORE] core1 park timeout, format skipped");  
            return;   // не форматируем — чтение покажет мусор, но плата жива  
        }  
        noInterrupts();    
        flash_range_erase(AF_FLASH_OFF, FLASH_SECTOR_SIZE);    
        flash_range_program(AF_FLASH_OFF, s_sector_img, FLASH_SECTOR_SIZE);    
        interrupts();  
        g_park_req = false;   // отпустить Core 1
  
        // Синхронизируем RAM-буфер EEPROM с записанным сектором.  
        for (int i = 0; i < AF_EEPROM_SIZE; i++) EEPROM.write(i, s_sector_img[i]);  
        Serial.println("[STORE] flash formatted (7 banks empty)");
    }  
}  
  
bool af_store_read(uint8_t idx, af_bank_t* out) {  
    if (idx < 1 || idx > AF_STORE_BANKS) return false;  
    af_bank_rec_t r;  
    EEPROM.get(rec_addr(idx), r);  
    if (r.crc == 0xFF) return false;                       // пустая  
    if (r.crc != crc8(&r.d, sizeof(r.d))) return false;    // битая  
    *out = r.d;  
    return true;  
}  
  
bool af_store_save(uint8_t idx) {  
    if (idx < 1 || idx > AF_STORE_BANKS) return false;  
  
    // Текущие настройки DSP -> запись.  
    af_bank_rec_t r;  
    r.d.lo_hz    = (uint16_t)af_dsp_band_lo_hz();  
    r.d.hi_hz    = (uint16_t)af_dsp_band_hi_hz();  
    r.d.cw_on    = af_dsp_cw_on() ? 1 : 0;  
    r.d.cw_f0    = (uint16_t)af_dsp_cw_freq();  
    r.d.cw_bw    = (uint16_t)af_dsp_cw_bw();  
    r.d.cw_noise = (uint16_t)af_dsp_cw_noise();
    r.d.agc_on   = af_dsp_agc_on() ? 1 : 0;  
    r.d.gain_pct = (uint16_t)af_dsp_gain_pct();
    r.d.voice_vol = af_voice_volume();
    r.crc = crc8(&r.d, sizeof(r.d));  
  
    // Читаем текущий сектор из flash напрямую (XIP), патчим запись.  
    memcpy(s_sector_img, (const void*)(XIP_BASE + AF_FLASH_OFF), sizeof(s_sector_img));  
    memcpy(s_sector_img + rec_addr(idx), &r, sizeof(r));  
  
    // Паркуем Core 1 и пишем сектор. После записи — ребут:  
    // возврат на 220 МГц и выход из парковки ненадёжны,  
    // а данные уже во flash — после ребута всё поднимется чисто.  
    if (!park_core1()) {  
        Serial.println("# save: core1 park timeout");  
        return false;  
    }  
    noInterrupts();  
    flash_range_erase(AF_FLASH_OFF, FLASH_SECTOR_SIZE);  
    flash_range_program(AF_FLASH_OFF, s_sector_img, FLASH_SECTOR_SIZE);  
  
    // Сюда доходим только если запись завершилась.  
    watchdog_reboot(0, 0, 0);  
    while (true) { tight_loop_contents(); }   // на случай задержки ребута
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
    af_dsp_set_gain(b.gain_pct);
    af_voice_set_volume(b.voice_vol);
    af_dsp_set_cw(b.cw_on != 0);  
    if (!b.cw_on) af_dsp_set_band(b.lo_hz, b.hi_hz);  
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
  
    if (!park_core1()) {  
        Serial.println("# bank clr: core1 park timeout");  
        return;  
    }  
    noInterrupts();  
    flash_range_erase(AF_FLASH_OFF, FLASH_SECTOR_SIZE);  
    flash_range_program(AF_FLASH_OFF, s_sector_img, FLASH_SECTOR_SIZE);  
    interrupts();  
  
    Serial.flush();  
    watchdog_reboot(0, 0, 0);   // не возвращается  
}