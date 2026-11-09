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
  
#include "pico/stdlib.h"  
#include "hardware/flash.h"  
#include "pico/flash.h"  
  
#define AF_STORE_MAGIC0   'A'  
#define AF_STORE_MAGIC1   'F'  
#define AF_STORE_VER      1  
#define AF_EEPROM_SIZE    256  
  
// Смещение сектора EEPROM от начала flash (последний сектор).  
#define AF_FLASH_OFF      (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  
  
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
        int rc = flash_safe_execute(do_flash_write, NULL, 500);  
        if (rc == PICO_OK) {  
            // Синхронизируем RAM-буфер EEPROM с записанным сектором.  
            for (int i = 0; i < AF_EEPROM_SIZE; i++) EEPROM.write(i, s_sector_img[i]);  
            Serial.println("[STORE] flash formatted (7 banks empty)");  
        } else {  
            Serial.printf("[STORE] format FAILED rc=%d\n", rc);  
        }  
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
    r.d.cw_noise = (uint8_t)af_dsp_cw_noise();  
    r.crc = crc8(&r.d, sizeof(r.d));  
  
    // Читаем текущий сектор из flash напрямую (XIP), патчим запись.  
    memcpy(s_sector_img, (const void*)(XIP_BASE + AF_FLASH_OFF), sizeof(s_sector_img));  
    memcpy(s_sector_img + rec_addr(idx), &r, sizeof(r));  
  
    // Erase+program: flash_safe_execute сам паркует Core 1 и гоняет  
    // запись из RAM — безопасно при любом clk_sys.  
    int rc = flash_safe_execute(do_flash_write, NULL, 500);  
    if (rc != PICO_OK) {  
        Serial.printf("# save: flash write failed rc=%d\n", rc);  
        return false;  
    }  
  
    // Синхронизируем RAM-буфер EEPROM, чтобы af_store_read видел запись.  
    for (int i = 0; i < AF_EEPROM_SIZE; i++) EEPROM.write(i, s_sector_img[i]);  
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
    af_dsp_set_cw(b.cw_on != 0);  
    if (!b.cw_on) af_dsp_set_band(b.lo_hz, b.hi_hz);  
    return true;  
}