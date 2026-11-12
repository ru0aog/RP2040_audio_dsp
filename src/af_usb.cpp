// src/af_usb.cpp  
// ============================================================================  
// af_usb.cpp — UAC-микрофон 44100/16 через arduino-audio-tools (Core 0)  
//  
// Тракт: int16-блоки 31.25 ksps из af_dsp_out_get_usb() -> кубический
// ресемплер Фарроу 31250 -> 44100 -> антиалиасинг-ФНЧ (биквады Q13,  
// Баттерворт 4-го порядка, fc~5.3 кГц) -> USBAudioDeviceTinyUSB  
// (UAC2, стерео: моно-сигнал дублируется в L/R).  
//  
// Порт src/rx/rx_usb_audio.cpp из RP2040_radiomodem (режим UAC_MODE_SSB).  
// ============================================================================  
#include "af_config.h"  
  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "af_usb.h"  
#include "af_dsp.h"  
  
#include "AudioTools.h"  
#include "AudioTools/Communication/USB/USBAudioStream.h"  
  
// ---------------------------------------------------------------------------  
// Диагностика разрывов (объявлены до get_input — используются в нём)  
// ---------------------------------------------------------------------------  
static volatile uint32_t usb_dropped      = 0;   // (резерв) блоки, потерянные в out_usb  
static volatile uint32_t usb_underruns    = 0;   // разы, когда источник пуст при avail>0  
static volatile uint32_t usb_frames_short = 0;   // суммарная недостача фреймов  
static volatile uint32_t usb_blocks_rx    = 0;   // блоков получено из out_usb  
static volatile int32_t  usb_rs_adj_mhz   = 0;   // текущая подстройка rs_step, мГц  
  
// Кольцо последних 8 underrun-событий: метка micros + недостача фреймов.  
// По интервалам между метками видна периодичность разрывов.  
static volatile uint32_t usb_ur_t[8];    
static volatile uint16_t usb_ur_n[8];    
static volatile uint8_t  usb_ur_idx = 0;    
    
// Гистерезис заправки: не отдаём данные хосту, пока out_usb не накопит      
// половину кольца. Иначе очередь живёт на дне (fill=0..1), регулятор      
// прижат к -LIMIT, а TinyUSB пустеет между 4-мс блоками → разрывы.      
static bool     usb_primed    = false;      
static volatile uint32_t usb_reprimes = 0;   // разы ушли на перезаправку      
  
// Нижняя отметка: в out_usb всегда остаётся минимум LOW блоков подушки.    
// Без неё внутренний цикл сливает всю очередь в буфер TinyUSB за один    
// poll — подушка уходит туда, где регулятор её не видит, fill падает    
// в 0 и каждые ~36 мс случается reprime (слышимая пауза).    
#define USB_LOW_MARK  4u    // блоков; TARGET=8, гистерезис 8 -> 4      
  
// Диагностика джиттера подачи в TinyUSB:    
// gap_max — макс. интервал между вызовами af_usb_poll (если > размера    
// TX-FIFO в мкс, FIFO гарантированно пустеет → дырка в изохронном потоке).    
// fifo_full — сколько раз poll встретил полный FIFO (здоровое состояние).    
// write_short — write() принял меньше байт, чем просили (потеря фреймов).    
static volatile uint32_t usb_poll_gap_max = 0;      
static volatile uint32_t usb_fifo_full    = 0;      
static volatile uint32_t usb_write_short  = 0;      
static uint32_t usb_last_poll_us = 0;      
  
// Максимум avail_frames за всё время ≈ ёмкость TX-FIFO TinyUSB в фреймах    
// (×22.7 мкс = мс аудио). Если ~48–96 — FIFO ~1–2 мс, джиттер loop()    
// больше этой величины гарантированно даёт дырку в изохронном потоке.    
static volatile uint32_t usb_avail_max = 0;    
  
// ---------------------------------------------------------------------------  
// Источник: выходная очередь af_dsp (int16-блоки 31.25 ksps)  
// ---------------------------------------------------------------------------  
static const volatile int16_t* cur_blk = nullptr;  
static uint16_t pos = 0;  
  
static bool get_input(int32_t* out) {  
    if (out == nullptr) return false;  
  
    while (cur_blk == nullptr || pos >= AF_OUT_BLOCK_SAMPLES) {  
        if (cur_blk != nullptr) {  
            af_dsp_out_release_usb();  
            cur_blk = nullptr;  
            pos = 0;  
        }  
        cur_blk = af_dsp_out_get_usb();    
        if (cur_blk == nullptr) return false;    
        ++usb_blocks_rx;    
        pos = 0;    
    }  
  
    *out = (int32_t)cur_blk[pos];  
    ++pos;  
    return true;
}  
  
// ---------------------------------------------------------------------------  
// Ресемплер 31250 -> 44100 Гц (кубический Фарроу, порт из rx_usb_audio.cpp)  
// ---------------------------------------------------------------------------  
static int32_t  rs_xm1 = 0, rs_x0 = 0, rs_x1 = 0, rs_x2 = 0;  
static bool     rs_started = false;  
static uint8_t  rs_init_count = 0;  
static bool     rs_advance_pending = false;  
  
static uint32_t rs_phase_q = 0;  
static uint32_t rs_step_q  = 0;  
static float    rs_step    = (float)AF_FS_OUT_HZ;   // адаптивно подстраивается  
  
// ---- Антиалиасинг-ФНЧ на 44100 Гц: Баттерворт 4-го порядка, fc~5.3 кГц.  
// Коэффициенты Q13, Direct Form I, int64-аккумулятор.  
#define BQ_Q 13  
  
typedef struct {  
    int32_t b0, b1, b2, a1, a2;  
    int64_t z1, z2;  
} biquad_t;  
  
static biquad_t bq_af[2] = {  
    { 682, 1363, 682, -7305, 1840, 0, 0 },  
    { 882, 1764, 882, -9452, 4788, 0, 0 },  
};  
  
static inline int32_t __not_in_flash_func(biquad_run)(biquad_t* s, int32_t x) {  
    int64_t acc = (int64_t)s->b0 * x + s->z1;  
    int32_t y   = (int32_t)(acc >> BQ_Q);  
    s->z1 = (int64_t)s->b1 * x - (int64_t)s->a1 * y + s->z2;  
    s->z2 = (int64_t)s->b2 * x - (int64_t)s->a2 * y;  
    return y;  
}  
  
static inline int32_t biquad_cascade(biquad_t* sec, int32_t x) {  
    x = biquad_run(&sec[0], x);  
    x = biquad_run(&sec[1], x);  
    return x;  
}  
  
static inline void biquad_reset(biquad_t* sec) {  
    sec[0].z1 = sec[0].z2 = 0;  
    sec[1].z1 = sec[1].z2 = 0;  
}  
  
// Коэффициенты кубического полинома Фарроу по 4 точкам (целочисленные).  
static int32_t rs_c0 = 0, rs_c1 = 0, rs_c2 = 0, rs_c3 = 0;  
  
static inline void farrow_calc_i(int32_t ym1, int32_t y0, int32_t y1, int32_t y2,  
                                 int32_t* c0, int32_t* c1, int32_t* c2, int32_t* c3) {  
    *c0 = 2 * y0;  
    *c1 = y1 - ym1;  
    *c2 = 2 * ym1 - 5 * y0 + 4 * y1 - y2;  
    *c3 = 3 * (y0 - y1) + y2 - ym1;  
}  
  
static inline int32_t farrow_eval_i(int32_t c0, int32_t c1, int32_t c2, int32_t c3,  
                                    uint32_t mu_q15) {  
    int32_t t = c3;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c2;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c1;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c0;  
    return t;  
}  
  
// Один выходной отсчёт 44.1k (моно). false — входная очередь пуста.  
static bool next_frame(int16_t* s) {  
    if (s == nullptr) return false;  
  
    if (!rs_started) {  
        if (rs_init_count == 0) { if (!get_input(&rs_xm1)) return false; rs_init_count = 1; }  
        if (rs_init_count == 1) { if (!get_input(&rs_x0))  return false; rs_init_count = 2; }  
        if (rs_init_count == 2) { if (!get_input(&rs_x1))  return false; rs_init_count = 3; }  
        if (rs_init_count == 3) { if (!get_input(&rs_x2))  return false; rs_init_count = 4; }  
        farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3);  
        rs_phase_q = 0;  
        rs_advance_pending = false;  
        rs_started = true;  
    }  
  
    if (rs_advance_pending) {  
        int32_t next_x;  
        if (!get_input(&next_x)) return false;  
        rs_xm1 = rs_x0;  
        rs_x0  = rs_x1;  
        rs_x1  = rs_x2;  
        rs_x2  = next_x;  
        farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3);  
        rs_advance_pending = false;  
    }  
  
    const uint32_t mu_q15 = rs_phase_q >> 17;  
    const int32_t  y2x    = farrow_eval_i(rs_c0, rs_c1, rs_c2, rs_c3, mu_q15);  
    int32_t        y      = (y2x + 1) >> 1;  
    y = biquad_cascade(bq_af, y);  
  
    const uint32_t prev = rs_phase_q;  
    rs_phase_q += rs_step_q;  
    if (rs_phase_q < prev) rs_advance_pending = true;  
  
    if (y >  32767) y =  32767;  
    if (y < -32768) y = -32768;  
    *s = (int16_t)y;  
    return true;  
}  
  
// ---------------------------------------------------------------------------  
// UAC-вывод через arduino-audio-tools  
// ---------------------------------------------------------------------------  
USBAudioDeviceTinyUSB usb_out;  
  
static int16_t pcm_buffer[192 * 2];             // стерео-фреймы L/R  
  
void af_usb_init(void) {  
    memset(pcm_buffer, 0, sizeof(pcm_buffer));  
    rs_step    = (float)AF_FS_OUT_HZ;  
    rs_step_q  = (uint32_t)((double)rs_step * (4294967296.0 / (double)AF_USB_FS_HZ));  
    rs_phase_q = 0;  
    rs_started = false;  
    rs_init_count = 0;  
    rs_advance_pending = false;  
    biquad_reset(bq_af);    
    usb_dropped      = 0;    
    usb_underruns    = 0;    
    usb_frames_short = 0;    
    usb_blocks_rx    = 0;    
    usb_rs_adj_mhz   = 0;    
    usb_ur_idx       = 0;      
    memset((void*)usb_ur_t, 0, sizeof(usb_ur_t));      
    memset((void*)usb_ur_n, 0, sizeof(usb_ur_n));      
    usb_primed    = false;      
    usb_reprimes  = 0;      
    usb_poll_gap_max = 0;      
    usb_fifo_full    = 0;      
    usb_write_short  = 0;      
    usb_last_poll_us = 0;      
    usb_avail_max    = 0;    
  
    // Конфигурация USB Audio Class 2.0.  
    auto cfg = usb_out.defaultConfig(TX_MODE);  
    cfg.sample_rate     = AF_USB_FS_HZ;  
    cfg.channels        = 2;  
    cfg.bits_per_sample = 16;  
  
    // NB: вызывается ДО Serial.begin — отсюда не печатаем.  
    // Статус смотрим через af_usb_mounted() из setup()/stats.  
    // detach/attach не нужен: стек ещё не смонтирован, mounted()==false.  
    usb_out.begin(cfg);  
}  
  
// Вызов из loop() на Core 0.    
void af_usb_poll(void) {    
    // Джиттер вызовов: если gap > ёмкости TX-FIFO в мкс — в потоке дырка.    
    const uint32_t now_us = time_us_32();    
    const uint32_t gap = now_us - usb_last_poll_us;    
    usb_last_poll_us = now_us;    
    if (gap > usb_poll_gap_max && gap < 1000000u) usb_poll_gap_max = gap;    
    
    // Адаптивная подстройка темпа под часы USB-хоста: держим заполнение    
    // выходной очереди около середины (AF_OUT_NUM_BUFFERS=4 -> TARGET=2).  
    // Компенсирует расхождение кварца RP2040 и часов USB-хоста — без этого  
    // через минуты набегает underrun/overrun очереди.  
    const float   TARGET = (float)AF_OUT_NUM_BUFFERS / 2.0f;   // 8.0    
    const float   K      = 6.0f;    
    // ±500 Гц: реальный дефицит производства ~325 Гц (blk=2416/10с вместо    
    // 2441 — ADC выдаёт ~494.6 кс/с вместо 500к). При LIMIT=60 регулятор    
    // упирался в -60 Гц, сток оставался быстрее источника → reprime-цикл.    
    const float   LIMIT  = 500.0f;    
    const uint8_t fill   = af_dsp_out_fill_usb();
  
    float adj = K * ((float)fill - TARGET);  
    if (adj >  LIMIT) adj =  LIMIT;  
    if (adj < -LIMIT) adj = -LIMIT;  
    rs_step   = (float)AF_FS_OUT_HZ + adj;      
    rs_step_q = (uint32_t)((double)rs_step * (4294967296.0 / (double)AF_USB_FS_HZ));      
    usb_rs_adj_mhz = (int32_t)(adj * 1000.0f);   // телеметрия: упор в ±LIMIT = регулятор не тянет      
      
    // Гистерезис заправки: стартуем отдачу только с подушкой >= TARGET    
    // блоков (~33 мс при N=16). Пока не заправлены — TinyUSB получает    
    // ничего, но это однократная задержка старта, а не периодические дыры.    
    if (!usb_primed) {    
        if (fill < (uint8_t)(AF_OUT_NUM_BUFFERS / 2)) return;    
        usb_primed = true;    
    }    
    if (fill == 0) {    
        // Доели до дна — сток быстрее источника. НЕ уходим на перезаправку:    
        // 33 мс молчания слышны сильнее, чем повторы последнего фрейма.    
        // Продолжаем кормить — next_frame вернёт false, кадр допишется    
        // паддингом, а очередь тем временем накопится.    
        ++usb_reprimes;    
    }    
      
    for (;;) {  
        // Без нижней отметки: заполняем буфер AudioTools до отказа —  
        // это 46 мс подушки у самого эндпоинта, она и глотает джиттер  
        // loop(). Подушку держит буфер AudioTools, а не out_usb.  
        int avail_frames = usb_out.availableForWrite() / 4;  
        if (avail_frames <= 0) { ++usb_fifo_full; break; }   // FIFO полон — норма          
        if ((uint32_t)avail_frames > usb_avail_max) usb_avail_max = (uint32_t)avail_frames;    
        if (avail_frames > 192) avail_frames = 192;        
    
        int frames = 0;    
        int16_t s = 0;    
    
        while (frames < avail_frames) {    
            if (!next_frame(&s)) {    
                // Источник пуст: underrun. НЕ обрываем кадр — дописываем    
                // последний фрейм (s сохраняет прошлое значение). Дефицит    
                // ~1% = ~10 мс повторов в секунду, размазанных по потоку —    
                // неслышно, в отличие от дыр до следующей заправки.    
                ++usb_underruns;    
                usb_frames_short += (uint32_t)(avail_frames - frames);    
                usb_ur_t[usb_ur_idx] = time_us_32();    
                usb_ur_n[usb_ur_idx] = (uint16_t)(avail_frames - frames);    
                usb_ur_idx = (uint8_t)((usb_ur_idx + 1) & 7);    
                while (frames < avail_frames) {    
                    pcm_buffer[frames * 2]     = s;    
                    pcm_buffer[frames * 2 + 1] = s;    
                    ++frames;    
                }    
                break;    
            }    
            pcm_buffer[frames * 2]     = s;                   // L    
            pcm_buffer[frames * 2 + 1] = s;                   // R = дубль моно    
            ++frames;    
        }    
    
        if (frames > 0) {    
            // write() может принять меньше, чем просили — это потерянные    
            // фреймы и сдвиг фазы у хоста. Считаем.    
            const size_t want = (size_t)frames * 4;    
            const size_t sent = usb_out.write((uint8_t*)pcm_buffer, want);    
            if (sent < want) usb_write_short += (uint32_t)(want - sent);    
        }    
        // Кадр всегда заполняется целиком (реальные данные или паддинг) —    
        // условие выхода только по avail_frames/FIFO.    
    }    
}  
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
uint32_t af_usb_dropped(void)      { return usb_dropped; }    
uint32_t af_usb_underruns(void)    { return usb_underruns; }    
uint32_t af_usb_frames_short(void) { return usb_frames_short; }    
uint32_t af_usb_blocks_rx(void)    { return usb_blocks_rx; }      
uint32_t af_usb_reprimes(void)     { return usb_reprimes; }      
int32_t  af_usb_rs_adj_mhz(void)   { return usb_rs_adj_mhz; }        
uint32_t af_usb_fifo_full(void)    { return usb_fifo_full; }      
uint32_t af_usb_write_short(void)  { return usb_write_short; }      
uint32_t af_usb_avail_max(void)    { return usb_avail_max; }   // ~ёмкость TX-FIFO, фреймы    
    
// Пиковый интервал между вызовами af_usb_poll, мкс. Чтение сбрасывает пик.    
uint32_t af_usb_poll_gap_max(void) {    
    uint32_t v = usb_poll_gap_max;    
    usb_poll_gap_max = 0;    
    return v;    
}    
bool     af_usb_mounted(void)      { return TinyUSBDevice.mounted(); }    
    
// Копия кольца последних 8 underrun-событий (t — micros, n — недостача фреймов).    
void af_usb_ur_log(uint32_t* t, uint16_t* n) {    
    for (int i = 0; i < 8; i++) { t[i] = usb_ur_t[i]; n[i] = usb_ur_n[i]; }    
}  