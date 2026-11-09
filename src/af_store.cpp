// src/af_store.cpp  
// ============================================================================  
// af_store.cpp — банки настроек во flash (EEPROM-эмуляция, Core 0)  
//  
// Раскладка EEPROM (Earle Philhower core, эмуляция на последнем секторе):  
//   [0..1]   magic 'A','F'  
//   [2]      версия формата  
//   [3]      резерв  
//   [4..]    7 записей по sizeof(af_bank_rec_t)  
//  
// Запись — только по команде 'save x' (flash стирается ~100к циклов,  
// автосохранения нет). Чтение — по 'load x' и при смене кода на GP2..GP4.  
// ============================================================================  
#include <Arduino.h>  
#include <EEPROM.h>  
#include "af_store.h"  
#include "af_config.h"  
#include "af_dsp.h"  
  
#define AF_STORE_MAGIC0   'A'  
#define AF_STORE_MAGIC1   'F'  
#define AF_STORE_VER      1  
#define AF_EEPROM_SIZE    256  
  
#pragma pack(push, 1)  
struct af_bank_rec_t {  
    af_bank_t d;  
    uint8_t   crc;        // XOR всех байт d; 0xFF-пустая ячейка имеет crc=0xFF  
};  
#pragma pack(pop)  
  
static uint8_t crc8(const void* p, size_t n) {  
    const uint8_t* b = (const uint8_t*)p;  
    uint8_t c = 0;  
    for (size_t i = 0; i < n; i++) c ^= b[i];  
    return c;  
}  
  
void af_store_init(void) {  
    EEPROM.begin(AF_EEPROM_SIZE);  
    // Если magic не совпал — форматируем: все ячейки пустые (0xFF).  
    if (EEPROM.read(0) != AF_STORE_MAGIC0 ||  
        EEPROM.read(1) != AF_STORE_MAGIC1 ||  
        EEPROM.read(2) != AF_STORE_VER) {  
        for (int i = 0; i < AF_EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);  
        EEPROM.write(0, AF_STORE_MAGIC0);  
        EEPROM.write(1, AF_STORE_MAGIC1);  
        EEPROM.write(2, AF_STORE_VER);  
        EEPROM.commit();  
        Serial.println("[STORE] EEPROM formatted (7 banks empty)");  
    }  
}  
  
static int rec_addr(uint8_t idx) {           // idx = 1..7  
    return 4 + (idx - 1) * (int)sizeof(af_bank_rec_t);  
}  
  
bool af_store_read(uint8_t idx, af_bank_t* out) {  
    if (idx < 1 || idx > AF_BANK_COUNT) return false;  
    af_bank_rec_t r;  
    EEPROM.get(rec_addr(idx), r);  
    if (r.crc == 0xFF) return false;                       // пустая  
    if (r.crc != crc8(&r.d, sizeof(r.d))) return false;    // битая  
    *out = r.d;  
    return true;  
}  
  
bool af_store_save(uint8_t idx) {  
    if (idx < 1 || idx > AF_BANK_COUNT) return false;  
    af_bank_rec_t r;  
    r.d.lo_hz    = (uint16_t)af_dsp_band_lo_hz();  
    r.d.hi_hz    = (uint16_t)af_dsp_band_hi_hz();  
    r.d.cw_on    = af_dsp_cw_on() ? 1 : 0;  
    r.d.cw_f0    = (uint16_t)af_dsp_cw_freq();  
    r.d.cw_bw    = (uint16_t)af_dsp_cw_bw();  
    r.d.cw_noise = (uint8_t)af_dsp_cw_noise();  
    r.crc = crc8(&r.d, sizeof(r.d));  
    EEPROM.put(rec_addr(idx), r);  
  
    rp2040.idleOtherCore();      // Core 1 на паузу — flash будет недоступен  
    EEPROM.commit();  
    rp2040.resumeOtherCore();  
  
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