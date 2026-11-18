// src/af_rec.cpp    
// ============================================================================  
// af_rec.cpp — цифровой магнитофон, один слот ~25 с во flash.  
//  
// Раскладка слота (AF_REC_FLASH_OFF, 191 сектор = 782336 байт):  
//   [0..255]    заголовок: magic 'R','C', ver, flags, len_samples  
//   [256..]     данные int16, до AF_REC_MAX_SAMP отсчётов  
// Заголовок пишется ПОСЛЕДНИМ при финализации — прерванная запись  
// (питание, ребут) читается как пустой слот.  
//  
// Старт записи: стираются только сектора 0 и 1 (~90 мс), остальные  
// стирает erase_ahead() по мере записи — конвейер «пишем сектор N,  
// стираем N+1». Инвариант: сектора [0, erase_off) всегда стёрты.  
//  
// Flash I/O — SDK-обёртки flash_range_erase/flash_range_program  
// (hardware/flash.h): они сами делают connect_internal_flash,  
// flush_cache и enter_cmd_xip. Прямые ROM-вызовы rom_flash_range_*  
// ЗАПРЕЩЕНЫ — без enter_cmd_xip XIP остаётся выключенным и следующее  
// обращение к flash вешает оба ядра (см. BUGS.MD, af_store).  
//  
// Core 1 НЕ паркуется и не сбрасывается на время erase/program.  
// ИНВАРИАНТ: вся цепочка loop1()/af_dsp_poll должна быть в RAM —  
// запрещено вызывать оттуда любые функции без __not_in_flash_func.  
// Нарушение инварианта = зависание во время записи.  
// ============================================================================  
#include <Arduino.h>    
#include <string.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
  
#include "pico/stdlib.h"        
#include "hardware/flash.h"        
#include "hardware/sync.h"     // save_and_disable_interrupts        

  
#define AF_REC_MAGIC0     'R'    
#define AF_REC_MAGIC1     'C'    
#define AF_REC_VER        1    
#define AF_REC_HDR_BYTES  256u                       // одна flash-страница    
#define AF_REC_BUF_SAMP   2048u                      // 4 КБ = один сектор    
  
typedef struct {    
    uint8_t  magic0, magic1, ver, flags;             // flags bit0 = источник TX    
    uint32_t len;                                    // отсчётов в записи    
    uint8_t  rsv[AF_REC_HDR_BYTES - 8];    
} rec_hdr_t;    
  
typedef enum { ST_IDLE = 0, ST_REC, ST_PLAY } rec_state_t;    
  
static volatile rec_state_t state = ST_IDLE;    
static volatile uint32_t    rec_len = 0;             // валидных отсчётов в слоте    
static volatile uint32_t    pos     = 0;             // позиция записи/чтения    
static volatile uint8_t     rec_src_tx = 0;    
  
// Двойной буфер записи: Core 1 пилит один, Core 0 программирует другой.    
static int16_t            buf[2][AF_REC_BUF_SAMP];    
static volatile uint8_t   buf_wr      = 0;           // буфер наполнения    
static volatile uint16_t  buf_fill    = 0;    
static volatile uint8_t   buf_rdy[2]  = { 0, 0 };    // готов к program    
static volatile uint32_t  flash_wr    = 0;           // байт в data-области      
static volatile uint32_t  erase_off   = 0;           // стёрто до этого смещения      
static volatile uint32_t  rec_dropped = 0;           // блоки без свободного буфера      
  
static const uint8_t* slot_base(void) {    
    return (const uint8_t*)(XIP_BASE + AF_REC_FLASH_OFF);    
}    
  
// --- чтение заголовка -------------------------------------------------------    
static bool hdr_read(rec_hdr_t* h) {    
    memcpy(h, slot_base(), sizeof(rec_hdr_t));    
    return h->magic0 == AF_REC_MAGIC0 && h->magic1 == AF_REC_MAGIC1 &&    
           h->ver == AF_REC_VER && h->len <= AF_REC_MAX_SAMP;    
}    
  
// --- flash I/O: SDK-обёртки, запрет прерываний только на Core 0 -------------  
// flash_range_* сами делают connect/flush/enter_cmd_xip — ручной  
// flash_flush_cache() не нужен. Core 1 не паркуется — условие: вся  
// цепочка loop1() в RAM (__not_in_flash_func).  
static void slot_erase_sector(uint32_t off) {        // off кратно 4096, внутри слота  
    uint32_t ints = save_and_disable_interrupts();  
    flash_range_erase(AF_REC_FLASH_OFF + off, 4096);  
    restore_interrupts(ints);  
}  
static void slot_program(uint32_t off, const void* data, uint32_t len) {  
    uint32_t ints = save_and_disable_interrupts();  
    flash_range_program(AF_REC_FLASH_OFF + off, (const uint8_t*)data, len);  
    restore_interrupts(ints);  
}  
// Держим erase_off на сектор впереди следующей записи.      
static void erase_ahead(void) {      
    const uint32_t need = AF_REC_HDR_BYTES + flash_wr + 4096;      
    while (erase_off < need && erase_off < AF_REC_FLASH_LEN) {      
        slot_erase_sector(erase_off);      
        erase_off += 4096;      
    }      
}      
  
// --- API --------------------------------------------------------------------    
void af_rec_init(void) {    
    rec_hdr_t h;    
    rec_len = hdr_read(&h) ? h.len : 0;    
    Serial.printf("# rec: slot %s, %lu samp (%lu.%lu s)\n",      
                  rec_len ? "used" : "empty",      
                  (unsigned long)rec_len,      
                  (unsigned long)(rec_len / AF_FS_OUT_HZ),      
                  (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ));      
}    
  
bool af_rec_start(void) {    
    if (state != ST_IDLE) return false;    
    // Стираем сектор 0 (заголовок) и сектор 1 (первые данные) — ~90 мс.      
    // Остальные сектора стирает erase_ahead() по мере записи.      
    slot_erase_sector(0);      
    slot_erase_sector(4096);      
    erase_off = 8192;      
    rec_len = 0;      
    pos = 0; flash_wr = 0; buf_fill = 0; buf_wr = 0;
    buf_rdy[0] = buf_rdy[1] = 0; rec_dropped = 0;
    rec_src_tx = af_dsp_tx_on() ? 1 : 0;    
    state = ST_REC;    
    Serial.printf("# rec: recording (%s)\n", rec_src_tx ? "tx" : "rx");    
    return true;    
}    
  
void af_rec_stop(void) {        
    if (state == ST_PLAY) { af_rec_play_stop(); return; }   // stop универсальный    
    if (state != ST_REC) return;        
    state = ST_IDLE;        
    for (int i = 0; i < 2; i++)             // сначала готовые буферы — они раньше в потоке    
        if (buf_rdy[i]) {      
            slot_program(AF_REC_HDR_BYTES + flash_wr, buf[i], AF_REC_BUF_SAMP * 2);      
            flash_wr += AF_REC_BUF_SAMP * 2;      
            buf_rdy[i] = 0;      
        }      
    // Затем остаток текущего буфера (выравнивание на страницу 256 байт).      
    uint32_t tail = buf_fill * 2;      
    if (tail) {      
        uint32_t pad = (256 - (tail & 255)) & 255;      
        memset((uint8_t*)buf[buf_wr] + tail, 0xFF, pad);      
        slot_program(AF_REC_HDR_BYTES + flash_wr, buf[buf_wr], tail + pad);      
        flash_wr += tail;      
    }      
    rec_len = pos;    
    rec_hdr_t h;    
    memset(&h, 0xFF, sizeof(h));    
    h.magic0 = AF_REC_MAGIC0; h.magic1 = AF_REC_MAGIC1;    
    h.ver = AF_REC_VER; h.flags = rec_src_tx; h.len = rec_len;    
    slot_program(0, &h, AF_REC_HDR_BYTES);  // заголовок — последним  
    // flush/enter_xip уже сделаны внутри flash_range_program  
    Serial.printf("# rec: saved %lu samp (%lu.%lu s), dropped=%lu\n",        
                  (unsigned long)rec_len,      
                  (unsigned long)(rec_len / AF_FS_OUT_HZ),      
                  (unsigned long)((rec_len % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ),      
                  (unsigned long)rec_dropped);      
}    
  
bool af_rec_play(void) {    
    if (state != ST_IDLE || rec_len == 0) return false;    
    pos = 0;    
    state = ST_PLAY;    
    Serial.printf("# rec: play %lu samp\n", (unsigned long)rec_len);    
    return true;    
}    
  
void af_rec_play_stop(void) {    
    if (state == ST_PLAY) { state = ST_IDLE; Serial.println("# rec: play stop"); }    
}    
  
// Геттеры — в RAM: вызываются из af_voice_mix/af_dsp_poll (Core 1)  
// во время erase/program, когда XIP выключен.  
bool     __not_in_flash_func(af_rec_is_recording)(void)  { return state == ST_REC; }      
bool     __not_in_flash_func(af_rec_is_playing)(void)    { return state == ST_PLAY; }      
uint32_t __not_in_flash_func(af_rec_len_samples)(void)   { return rec_len; }      
uint32_t __not_in_flash_func(af_rec_pos_samples)(void)   { return pos; }      
  
// --- Core 1, горячий путь (RAM) ----------------------------------------------    
void __not_in_flash_func(af_rec_push)(int16_t s) {      
    if (state != ST_REC) return;      
    if (pos >= AF_REC_MAX_SAMP) return;   // слот полон — финализацию сделает af_rec_poll      
    buf[buf_wr][buf_fill++] = s;    
    ++pos;    
    if (buf_fill >= AF_REC_BUF_SAMP) {    
        if (buf_rdy[buf_wr]) { ++rec_dropped; }   // Core 0 не успел — блок теряется    
        else buf_rdy[buf_wr] = 1;    
        buf_wr ^= 1;    
        buf_fill = 0;    
    }    
}    
  
int16_t __not_in_flash_func(af_rec_mix)(int16_t s) {    
    if (state != ST_PLAY) return s;    
    if (pos >= rec_len) { state = ST_IDLE; return s; }    
    const int16_t* d = (const int16_t*)(slot_base() + AF_REC_HDR_BYTES);    
    int32_t v = s + d[pos++];    
    if (v > 32767) v = 32767; else if (v < -32768) v = -32768;    
    return (int16_t)v;    
}    
  
int32_t __not_in_flash_func(af_rec_src)(int32_t mic) {    
    if (state != ST_PLAY) return mic;    
    if (pos >= rec_len) { state = ST_IDLE; return mic; }    
    const int16_t* d = (const int16_t*)(slot_base() + AF_REC_HDR_BYTES);    
    return (int32_t)d[pos++];              // замещение микрофона    
}    
  
// --- Core 0: программирование готовых буферов ---------------------------------    
void af_rec_poll(void) {        
    if (state != ST_REC) return;        
    if ((af_dsp_tx_on() ? 1 : 0) != rec_src_tx) {   // тракт сменился — финализируем    
        Serial.println("# rec: src changed (rx<->tx) — stop");    
        af_rec_stop();    
        return;    
    }    
    for (int i = 0; i < 2; i++) {          
        if (!buf_rdy[i]) continue;        
        slot_program(AF_REC_HDR_BYTES + flash_wr, buf[i], AF_REC_BUF_SAMP * 2);        
        flash_wr += AF_REC_BUF_SAMP * 2;        
        buf_rdy[i] = 0;        
    }        
    erase_ahead();                          // стираем сектор N+1, пока пишется N      
    if (pos >= AF_REC_MAX_SAMP) af_rec_stop();   // слот заполнен — дописать хвост и заголовок      
}  
