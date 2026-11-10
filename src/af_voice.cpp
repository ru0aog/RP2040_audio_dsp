// ============================================================================  
// af_voice.cpp — голосовые сообщения (плеер на Core 1, запросы с Core 0)  
//  
// Клипы: PCM int16 mono 15625 Гц, const-массивы во flash.  
// Подмес в выходной тракт после АРУ, до out_emit — слышно и в PWM, и в USB.  
// Очередь запросов: AF_VOICE_Q_DEPTH, события не теряются.  
// ============================================================================  
#include <Arduino.h>  
#include "af_voice.h"  
#include "af_config.h"
#include "voice/voice_01.inc"  
#include "voice/voice_02.inc"  
#include "voice/voice_03.inc" 
  
// --- Клипы (заглушки; заменить на реальную речь) ----------------------------  
// Тестовый клип: тон 800 Гц, 200 мс — пока вместо записанной речи.  
static const int16_t clip_test[] = {  
    // сгенерировать: python -c "import math; print(','.join(str(int(12000*math.sin(2*math.pi*800*i/15625))) for i in range(3125)))"  
    0  // заполнить сгенерированными данными  
};  
  
// Таблица клипов: id -> {данные, длина}. Порядок = af_voice_id_t.  
typedef struct { const int16_t* data; uint32_t len; } voice_clip_t;  
static const voice_clip_t s_clips[VOICE_COUNT] = {  
    { NULL, 0 },                              // VOICE_NONE  
    { clip_voice_01, clip_voice_01_len },     // VOICE_TEST  
    { clip_voice_02, clip_voice_02_len },     // VOICE_BANK_1  
    { clip_voice_03, clip_voice_03_len },     // VOICE_BANK_2  
    { NULL, 0 },                              // VOICE_BANK_4  
    { NULL, 0 },                              // VOICE_BYPASS_ON  
    { NULL, 0 },                              // VOICE_BYPASS_OFF  
};
  
// --- Состояние --------------------------------------------------------------  
static volatile af_voice_id_t s_queue[4];   // очередь запросов (Core0 -> Core1)  
static volatile uint8_t s_q_head = 0, s_q_tail = 0;  
  
static const int16_t* s_cur = NULL;         // текущий клип (Core 1)  
static uint32_t s_cur_len = 0, s_cur_pos = 0;  
  
void af_voice_init(void) { s_cur = NULL; s_q_head = s_q_tail = 0; }  
  
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
  
int16_t af_voice_mix(int16_t s) {  
    // Подхват следующего клипа из очереди  
    if (s_cur == NULL && s_q_tail != s_q_head) {  
        af_voice_id_t id = s_queue[s_q_tail];  
        s_q_tail = (s_q_tail + 1) & 3;  
        s_cur = s_clips[id].data;  
        s_cur_len = s_clips[id].len;  
        s_cur_pos = 0;  
    }  
    if (s_cur == NULL) return s;  
  
    int32_t v = (int32_t)s + s_cur[s_cur_pos++];  
    if (s_cur_pos >= s_cur_len) s_cur = NULL;  
    if (v > 32767) v = 32767;  
    if (v < -32768) v = -32768;  
    return (int16_t)v;  
}