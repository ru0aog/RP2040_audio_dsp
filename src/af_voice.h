#pragma once  
#include <stdint.h>  
  
// Идентификаторы голосовых сообщений. Расширяется по мере добавления клипов.  
typedef enum {  
    VOICE_NONE = 0,  
    VOICE_READY,          // 01: "радиотракт готов к работе"  
    VOICE_NT_ON,          // 02: "автоматический режекторный фильтр включен"  
    VOICE_CW_ON,          // 03: "телеграфный фильтр включен"  
    VOICE_NR_ON,          // 04: "адаптивное шумоподавление включено"  
    VOICE_BYPASS_ON,      // 05: "режим обхода включен"  
    VOICE_AGC_OFF,        // 06: "АРУ выключена"  
    VOICE_BANK_1,         // заготовки — клипов пока нет  
    VOICE_BANK_2,  
    VOICE_BANK_4,  
    VOICE_BYPASS_OFF,  
    VOICE_COUNT  
} af_voice_id_t;
  
void af_voice_init(void);  
// Поставить сообщение в очередь воспроизведения (Core 0, из GPIO/CLI).  
void af_voice_play(af_voice_id_t id);  
// Идёт воспроизведение или очередь не пуста.  
bool af_voice_busy(void);  
// Подмес в выходной сэмпл. Вызывается из af_dsp_poll на каждый отсчёт.  
// Возвращает s + голос с насыщением.  
int16_t af_voice_mix(int16_t s);

// Громкость голосовых клипов, % (0..400). 100 = как в файле.  
void     af_voice_set_volume(uint16_t pct);  
uint16_t af_voice_volume(void);