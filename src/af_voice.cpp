// src/af_voice.cpp
// ============================================================================    
// af_voice.cpp — голосовые сообщения (плеер на Core 1, запросы с Core 0)    
//    
// Клипы: PCM int16 mono 15625 Гц, const-массивы во flash.    
// Подмес в выходной тракт после АРУ, до out_emit — слышно и в PWM, и в USB.    
//    
// ВАЖНО: данные клипов и таблица s_clips лежат во flash и читаются через    
// XIP. Во время записи магнитофона (af_rec) XIP периодически выключается    
// на erase/program — поэтому при af_rec_is_recording() подмес и подхват    
// очереди запрещены: af_voice_mix возвращает вход без чтения flash.    
// Текущий клип ставится на паузу (s_cur_pos не двигается), очередь    
// сохраняется и доигрывает после stop.    
// ============================================================================    
#include <Arduino.h>  
#include "af_voice.h"  
#include "af_config.h"
#include "af_rec.h"            // af_rec_is_recording — запрет чтения клипов при rec  
#include "voice/voice_01.inc"  // 01: "радиотракт готов к работе"  
#include "voice/voice_02.inc"  // 02: "автоматический режекторный фильтр включен"  
#include "voice/voice_03.inc"  // 03: "телеграфный фильтр включен"  
#include "voice/voice_04.inc"  // 04: "адаптивное шумоподавление включено" 
#include "voice/voice_05.inc"  // 05: "режим обхода включен" 
#include "voice/voice_06.inc"  // 06: "АРУ выключена"  
#include "voice/voice_07.inc"  // 07: "профиль один" 
#include "voice/voice_08.inc"  // 08: "профиль два"
#include "voice/voice_09.inc"  // 09: "профиль три"
#include "voice/voice_10.inc"  // 10: "профиль четыре"
#include "voice/voice_11.inc"  // 11: "профиль пять" 
#include "voice/voice_12.inc"  // 12: "профиль шесть" 
#include "voice/voice_13.inc"  // 13: "профиль семь"   
  
// Таблица клипов: id -> {данные, длина}. Порядок = af_voice_id_t.  
typedef struct { const int16_t* data; uint32_t len; } voice_clip_t;  
static const voice_clip_t s_clips[VOICE_COUNT] = {  
    { NULL, 0 },                              // VOICE_NONE  
    { clip_voice_01, clip_voice_01_len },     // VOICE_READY  
    { clip_voice_02, clip_voice_02_len },     // VOICE_NT_ON  
    { clip_voice_03, clip_voice_03_len },     // VOICE_CW_ON  
    { clip_voice_04, clip_voice_04_len },     // VOICE_NR_ON  
    { clip_voice_05, clip_voice_05_len },     // VOICE_BYPASS_ON  
    { clip_voice_06, clip_voice_06_len },     // VOICE_AGC_OFF    
    { clip_voice_07, clip_voice_07_len },     // VOICE_PROF_1 
    { clip_voice_08, clip_voice_08_len },     // VOICE_PROF_2    
    { clip_voice_09, clip_voice_09_len },     // VOICE_PROF_3    
    { clip_voice_10, clip_voice_10_len },     // VOICE_PROF_4    
    { clip_voice_11, clip_voice_11_len },     // VOICE_PROF_5    
    { clip_voice_12, clip_voice_12_len },     // VOICE_PROF_6    
    { clip_voice_13, clip_voice_13_len },     // VOICE_PROF_7    
};  
  
// --- Состояние --------------------------------------------------------------  
static volatile af_voice_id_t s_queue[4];   // очередь запросов (Core0 -> Core1)  
static volatile uint8_t s_q_head = 0, s_q_tail = 0;  
  
static const int16_t* volatile s_cur = NULL;         // текущий клип (Core 1)  
static uint32_t s_cur_len = 0, s_cur_pos = 0;  

static volatile uint16_t s_vol_pct = 300;  
  
void af_voice_init(void) { s_cur = NULL; s_q_head = s_q_tail = 0; }  

void af_voice_set_volume(uint16_t pct) {  
    if (pct > 400) pct = 400;  
    s_vol_pct = pct;  
}  
  
void af_voice_play(af_voice_id_t id) {  
    if (id <= VOICE_NONE || id >= VOICE_COUNT) return;  
    if (s_clips[id].data == NULL || s_clips[id].len == 0) return;  
    uint8_t next = (s_q_head + 1) & 3;  
    if (next == s_q_tail) return;           // очередь полна — дроп  
    s_queue[s_q_head] = id;  
    __dmb();  
    s_q_head = next;  
}  
  
bool af_voice_busy(void) { return s_cur != NULL || s_q_head != s_q_tail; }  
  
int16_t __not_in_flash_func(af_voice_mix)(int16_t s) {    
    // Запись магнитофона: XIP периодически off — не трогаем flash    
    // (ни s_clips, ни данные клипа). Клип на паузе, очередь ждёт stop.    
    if (af_rec_is_recording()) return s;    
    
    // Подхват следующего клипа из очереди    
    if (s_cur == NULL && s_q_tail != s_q_head) {    
        af_voice_id_t id = s_queue[s_q_tail];  
        s_q_tail = (s_q_tail + 1) & 3;  
        s_cur = s_clips[id].data;  
        s_cur_len = s_clips[id].len;  
        s_cur_pos = 0;  
    }  
    if (s_cur == NULL) return s;  
  
    int32_t vs = ((int32_t)s_cur[s_cur_pos++] * s_vol_pct) / 100;  
    int32_t v  = (int32_t)s + vs;
    if (s_cur_pos >= s_cur_len) s_cur = NULL;  
    if (v > 32767) v = 32767;  
    if (v < -32768) v = -32768;  
    return (int16_t)v;  
}

uint16_t af_voice_volume(void) { return s_vol_pct; }