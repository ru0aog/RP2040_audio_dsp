// src/af_rec.cpp    
// ============================================================================    
// af_rec.cpp — цифровой магнитофон, один слот 25 с во flash.    
//    
// Раскладка слота (AF_REC_FLASH_OFF, 189 секторов = 774144 байт):      
//   [0..255]    заголовок: magic 'R','C', ver, flags, len_samples    
//   [256..]     данные int16, до AF_REC_MAX_SAMPLES отсчётов    
// Заголовок пишется ПОСЛЕДНИМ при финализации — прерванная запись    
// (питание, ребут) читается как пустой слот.    
//    
// Этап 1: стирание всего слота при старте записи через    
// flash_safe_execute (Core 1 паркуется, пауза ~8.6 с при перезаписи    
// занятого слота; пустой слот — стирание пропускается).    
// Этап 2 (TODO): конвейер «пишем сектор N — стираем N+1» без парковки,    
// DSP-цепочка под __not_in_flash_func.    
// ============================================================================    
#include <Arduino.h>    
#include <string.h>    
#include "af_rec.h"    
#include "af_config.h"    
#include "af_dsp.h"    
  
#include "pico/stdlib.h"    
#include "hardware/flash.h"    
#include "pico/flash.h"    
  
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
  
// --- flash I/O через flash_safe_execute (Core 1 паркуется) ------------------    
static void do_erase(void* p) {    
    flash_range_erase(AF_REC_FLASH_OFF, AF_REC_FLASH_LEN);    
    (void)p;    
}    
static void do_program(void* p) {    
    const uint32_t* a = (const uint32_t*)p;    
    flash_range_program(a[0], (const uint8_t*)a[1], a[2]);    
}    
static void slot_erase(void) {    
    flash_safe_execute(do_erase, NULL, 20000);    
}    
static void slot_program(uint32_t off, const void* data, uint32_t len) {    
    uint32_t a[3] = { AF_REC_FLASH_OFF + off, (uint32_t)data, len };    
    flash_safe_execute(do_program, a, 5000);    
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
    if (rec_len != 0) {                     // слот занят — стираем (этап 1: целиком)    
        Serial.println("# rec: erasing slot...");    
        slot_erase();    
        rec_len = 0;    
    } else {                                // пустой — проверяем, стёрт ли    
        const uint32_t* p = (const uint32_t*)slot_base();    
        for (uint32_t i = 0; i < AF_REC_FLASH_LEN / 4; i += 1024)    
            if (p[i] != 0xFFFFFFFF) { slot_erase(); break; }    
    }    
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
  
bool     af_rec_is_recording(void)  { return state == ST_REC; }    
bool     af_rec_is_playing(void)    { return state == ST_PLAY; }    
uint32_t af_rec_len_samples(void)   { return rec_len; }    
uint32_t af_rec_pos_samples(void)   { return pos; }    
  
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
    if (pos >= AF_REC_MAX_SAMP) af_rec_stop();   // слот заполнен — дописать хвост и заголовок    
}  
