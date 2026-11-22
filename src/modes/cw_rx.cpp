// src/modes/cw_rx.cpp — декодер CW (Морзе) по одному тону.  
// Порт src/modes/cw/cw_rx.cpp из RP2040_radiomodem (verbatim-порт  
// rx_cw.cpp из RP2040_receiver; детекция — порт FLDigi cw.cxx).  
//  
// Вход: моно AF-отсчёты 15625 Гц. Тон сносится в DC собственным  
// гетеродином + однополюсный НЧ. Детекция ключа:  
//   sig_avg / noise_floor / agc_peak (быстрый attack, медленный decay),  
//   нормализация value /= agc_peak, асимметричные пороги гистерезиса,  
//   отсев шумовых всплесков, порог точка/тире two_dots от WPM + трекинг.  
// Границы ключа дают точки/тире -> символы Морзе -> Serial.  
//  
// Обвязка cwdec_*: декодер на float, поэтому живёт на Core 0.  
// Core 1 кладёт отсчёты в SPSC-очередь (cwdec_push), Core 0 дренирует  
// её в cwdec_poll() из loop(). Переполнение -> cwdec_dropped_cnt.  
#include "cw_rx.h"  
#include <math.h>  
#include <string.h>  
  
// ---- параметры фронта ----  
#define CW_RX_FS_HZ      15625.0f   // фиксированный Fs тракта  
#define CW_LP_FC_HZ      80.0f      // полоса НЧ детектора тона  
#define CW_WIN_MS        5.0f       // окно огибающей (мс)  
  
// ---- параметры тайминга (в окнах по CW_WIN_MS) ----  
#define CW_AGC_ATTACK    4.0f       // ~20 мс — агрессивно к пику/провалу  
#define CW_AGC_DECAY     64.0f      // ~320 мс — медленный возврат  
  
static float fs_hz = CW_RX_FS_HZ;  
static bool  cw_active = false;  
  
// гетеродин + НЧ (один тон)  
static float osc_re, osc_im, rot_re, rot_im, lp_re, lp_im, lp_alpha;  
  
// накопление окна  
static int   win_div = 78, win_cnt = 0;  
static float win_acc = 0.0f;  
  
// AGC (порт FLDigi decode_stream)  
static float sig_avg, noise_floor, agc_peak;  
  
// тайминг Морзе (в окнах)  
static uint32_t win_idx;          // счётчик окон  
static uint32_t tone_start;       // окно начала тона  
static uint32_t key_up_at;        // окно последнего спада ключа  
static float    dot_win;          // длина точки (окон), адаптивная  
static float    two_dots;         // порог точка/тире (окон)  
static float    spike_win;        // мин. длительность элемента (окон)  
static int      last_elem;        // длительность предыдущего элемента (окон)    
static uint8_t  code;             // сборка: 1 = старт-бит, dot=0, dash=1    
static bool     space_sent;    
static bool     word_started;     // была напечатана буква — иначе пробел не нужен    
static bool     sq_open;          // сквелч: сигнал над шумом      
static float    sq_level = 0.25f; // порог открытия (norm_sig - norm_noise), 0=off      
static uint8_t  cw_wpm = CW_DEFAULT_WPM;  // стартовая скорость (для банка)    
  
// состояние приёма (как cw_receive_state в FLDigi)  
enum { RS_IDLE = 0, RS_IN_TONE, RS_AFTER_TONE };  
static int rx_state;  
  
static inline float decayavg(float a, float in, float w) {  
    if (w < 1.0f) w = 1.0f;  
    return a + (in - a) / w;  
}  
  
// ---- таблица Морзе: индекс = собранный код (старт-бит слева) ----  
static char morse_decode(uint8_t c) {  
    switch (c) {  
    case 0b101: return 'A'; case 0b11000: return 'B'; case 0b11010: return 'C';  
    case 0b1100: return 'D'; case 0b10: return 'E'; case 0b10010: return 'F';  
    case 0b1110: return 'G'; case 0b10000: return 'H'; case 0b100: return 'I';  
    case 0b10111: return 'J'; case 0b1101: return 'K'; case 0b10100: return 'L';  
    case 0b111: return 'M'; case 0b110: return 'N'; case 0b1111: return 'O';  
    case 0b10110: return 'P'; case 0b11101: return 'Q'; case 0b1010: return 'R';  
    case 0b1000: return 'S'; case 0b11: return 'T'; case 0b1001: return 'U';  
    case 0b10001: return 'V'; case 0b1011: return 'W'; case 0b11001: return 'X';  
    case 0b11011: return 'Y'; case 0b11100: return 'Z';  
    case 0b111111: return '0'; case 0b101111: return '1'; case 0b100111: return '2';  
    case 0b100011: return '3'; case 0b100001: return '4'; case 0b100000: return '5';  
    case 0b110000: return '6'; case 0b111000: return '7'; case 0b111100: return '8';  
    case 0b111110: return '9';  
    // знаки препинания (как в FLDigi morse.cxx)  
    case 0b110010:  return '/';  case 0b1110011: return ','; case 0b1001100: return '?';  
    case 0b110001:  return '=';  case 0b101010:  return '+'; case 0b1010101: return '.';  
    case 0b1100001: return '-';  case 0b1011010: return '@'; case 0b110110:  return '(';  
    case 0b1101101: return ')';  case 0b1111000: return ':'; case 0b1101010: return ';';  
    case 0b1001101: return '_';  case 0b1011110: return '\''; case 0b1010010: return '\\';  
    case 0b1101011: return '!';  case 0b10001001: return '$';  
    default: return 0;  
    }  
}  
  
static void emit_char(void) {    
    if (code > 1) {    
        char ch = morse_decode(code);    
        if (ch && sq_open) { Serial.write(ch); word_started = true; }    
    }    
    code = 1;    
}    
  
// Подстройка two_dots по парам точка-тире / тире-точка (FLDigi update_tracking)  
static void update_tracking(int d1, int d2) {  
    if (d1 <= 0 || d2 <= 0) return;  
    if (d1 > d2 && d1 > 4 * d2) return;  
    if (d2 > d1 && d2 > 4 * d1) return;  
    float avg = (float)(d1 + d2) * 0.5f;      // ≈ (dot+dash)/2 ≈ 2*dot  
    two_dots = decayavg(two_dots, avg, 16.0f);  
    dot_win  = two_dots * 0.5f;  
    if (dot_win < 1.0f) dot_win = 1.0f;  
    spike_win = dot_win * 0.5f;               // отсев всплесков < полуточки  
}  
  
static void cw_reset_timing(float wpm) {  
    dot_win   = (1200.0f / wpm) / CW_WIN_MS;  // ≈12 окон @20WPM, 16 @15WPM  
    if (dot_win < 1.0f) dot_win = 1.0f;  
    two_dots  = dot_win * 2.0f;  
    spike_win = dot_win * 0.5f;  
  
    win_idx = 0; tone_start = 0; key_up_at = 0;  
    last_elem = 0; code = 1; space_sent = true; word_started = false;    
    rx_state = RS_IDLE;  
}  
  
void cw_rx_init(uint32_t center_hz) {  
    // Центр 0 -> дефолтный тон 600 Гц  
    float tone_hz = (center_hz > 0) ? (float)center_hz  
                                    : (float)CW_TONE_OFFSET_HZ;  
    fs_hz = CW_RX_FS_HZ;  
    float w = 2.0f * (float)M_PI * tone_hz / fs_hz;  
    rot_re =  cosf(w);  
    rot_im = -sinf(w);           // сносит +tone_hz в DC  
    osc_re = 1.0f; osc_im = 0.0f;  
    lp_re = lp_im = 0.0f;  
    lp_alpha = 1.0f - expf(-2.0f * (float)M_PI * CW_LP_FC_HZ / fs_hz);  
  
    win_div = (int)(fs_hz * CW_WIN_MS / 1000.0f + 0.5f);  
    if (win_div < 1) win_div = 1;  
    win_cnt = 0; win_acc = 0.0f;  
  
    // AGC: agc_peak стартует с 1, чтобы деление было корректным сразу  
    sig_avg = 0.0f; noise_floor = 0.0f; agc_peak = 1.0f;  
  
    cw_reset_timing((float)cw_wpm);    
    cw_active = true;    
    Serial.printf("[CW] init tone=%lu Hz wpm=%u\n",    
                  (unsigned long)tone_hz, (unsigned)cw_wpm);    
}  
  
void cw_rx_set_wpm(uint8_t wpm) {    
    if (wpm < 5 || wpm > 60) return;   // защита от мусора    
    cw_wpm = wpm;    
    cw_reset_timing((float)wpm);    
}    
  
static void process_window(float value) {  
    // --- AGC (FLDigi decode_stream) ---  
    sig_avg = decayavg(sig_avg, value, CW_AGC_DECAY);  
  
    if (value < sig_avg) {  
        noise_floor = decayavg(noise_floor, value,  
                     (value < noise_floor) ? CW_AGC_ATTACK : CW_AGC_DECAY);  
    }  
    if (value > sig_avg) {  
        agc_peak = decayavg(agc_peak, value,  
                   (value > agc_peak) ? CW_AGC_ATTACK : CW_AGC_DECAY);  
    }  
    if (agc_peak < 1e-6f) agc_peak = 1e-6f;  
  
    float norm_noise = noise_floor / agc_peak;  
    float norm_sig   = sig_avg     / agc_peak;  
    float v          = value       / agc_peak;  
  
    float diff  = norm_sig - norm_noise;    
    float upper = norm_sig   - 0.2f * diff;   // порог keydown    
    float lower = norm_noise + 0.7f * diff;   // порог keyup (ближе к шуму)    
  
    // Сквелч с гистерезисом: открывается при явном превышении сигнала    
    // над шумом, закрывается ниже — на чистом шуме diff ≈ 0, печать глуха.    
    if (diff > sq_level)        sq_open = true;    
    else if (diff < sq_level * 0.5f) sq_open = false;    
  
    win_idx++;  
  
    // --- гистерезисный детектор ключа ---  
    if (v > upper && rx_state != RS_IN_TONE) {  
        // keydown  
        if (rx_state == RS_IDLE) { code = 1; }  
        tone_start = win_idx;  
        rx_state   = RS_IN_TONE;  
    } else if (v < lower && rx_state == RS_IN_TONE) {  
        // keyup — измеряем длительность элемента  
        int elem = (int)(win_idx - tone_start);  
        key_up_at = win_idx;  
  
        if ((float)elem < spike_win) {        // шумовой всплеск — игнор  
            rx_state = RS_IDLE;  
            return;  
        }  
        // трекинг скорости по паре элементов  
        if (last_elem > 0) {  
            if (elem > 2 * last_elem && elem < 4 * last_elem)  
                update_tracking(last_elem, elem);  
            if (last_elem > 2 * elem && last_elem < 4 * elem)  
                update_tracking(elem, last_elem);  
        }  
        last_elem = elem;  
  
        bool dash = ((float)elem > two_dots);  
        code = (uint8_t)((code << 1) | (dash ? 1 : 0));  
        space_sent = false;  
        rx_state = RS_AFTER_TONE;  
    }  
  
    // --- тайминг пауз (char gap / word space) ---  
    if (rx_state != RS_IN_TONE) {  
        float gap = (float)(win_idx - key_up_at);  
        if (rx_state == RS_AFTER_TONE && gap >= two_dots) {  
            emit_char();                       // пауза >= 2*dot -> конец буквы  
            rx_state  = RS_IDLE;  
            last_elem = 0;  
        }  
        // Пробел: только после напечатанной буквы и паузы ~7 точек    
        // (стандартный word gap). 4*dot давал лишние пробелы внутри    
        // слов при неровной манипуляции и на шумовых срабатываниях.    
        if (!space_sent && word_started && gap > two_dots * 3.5f) {    
            Serial.write(' ');    
            space_sent = true;    
            word_started = false;    
        }    
    }  
}  
  
void cw_rx_push(int16_t x) {  
    if (!cw_active) return;  
    float xi = (float)x;  
    // снос в DC + НЧ (один тон)  
    float yr = xi * osc_re;  
    float yi = xi * osc_im;  
    float nr = osc_re * rot_re - osc_im * rot_im;  
    float ni = osc_re * rot_im + osc_im * rot_re;  
    osc_re = nr; osc_im = ni;  
    lp_re += lp_alpha * (yr - lp_re);  
    lp_im += lp_alpha * (yi - lp_im);  
    win_acc += lp_re * lp_re + lp_im * lp_im;  
  
    if (++win_cnt >= win_div) {  
        // огибающая = модуль, sqrtf раз в окно (~200/с)  
        float mag = sqrtf(win_acc / (float)win_div);  
        win_cnt = 0; win_acc = 0.0f;  
        process_window(mag);  
        // ренорм фазора  
        float m = osc_re*osc_re + osc_im*osc_im;  
        float g = 1.5f - 0.5f * m;  
        osc_re *= g; osc_im *= g;  
    }  
}  
  
// Останов: флаш незавершённого символа + сброс состояния.  
void cw_rx_stop(void) {  
    if (!cw_active) return;  
    if (rx_state != RS_IDLE && code > 1) {  
        emit_char();                        // добить последнюю букву  
        if (!space_sent && word_started) Serial.write(' '); // закрыть слово    
    }  
    cw_active = false;  
    cw_reset_timing((float)cw_wpm);    
    sig_avg = 0.0f; noise_floor = 0.0f; agc_peak = 1.0f;  
    win_cnt = 0; win_acc = 0.0f;  
    lp_re = lp_im = 0.0f;  
    osc_re = 1.0f; osc_im = 0.0f;  
}  
  
// ============================================================================  
// Обвязка cwdec_* — граница Core 1 (push) / Core 0 (poll).  
// SPSC-очередь int16: 256 отсчётов ≈ 16 мс запаса при 15.6 ksps.  
// ============================================================================  
#define CWDEC_Q_LEN 256  
  
static volatile int16_t  cwdec_q[CWDEC_Q_LEN];  
static volatile uint16_t cwdec_q_head = 0;   // пишет Core 1  
static volatile uint16_t cwdec_q_tail = 0;   // читает Core 0  
static volatile uint32_t cwdec_dropped_cnt = 0;  
  
void cwdec_init(uint32_t center_hz) {  
    cwdec_q_head = cwdec_q_tail = 0;  
    cwdec_dropped_cnt = 0;  
    cw_rx_init(center_hz);  
}  
  
// Core 1, горячий путь: только запись в очередь, без float.  
void cwdec_push(int16_t x) {  
    uint16_t h = cwdec_q_head;  
    uint16_t n = (uint16_t)((h + 1) & (CWDEC_Q_LEN - 1));  
    if (n == cwdec_q_tail) { ++cwdec_dropped_cnt; return; }  // полна — drop  
    cwdec_q[h] = x;  
    __dmb();  
    cwdec_q_head = n;  
}  
  
// Core 0, из loop(): дренаж очереди в декодер.  
void cwdec_poll(void) {  
    while (cwdec_q_tail != cwdec_q_head) {  
        int16_t x = cwdec_q[cwdec_q_tail];  
        cwdec_q_tail = (uint16_t)((cwdec_q_tail + 1) & (CWDEC_Q_LEN - 1));  
        cw_rx_push(x);  
    }  
}  
  
void cwdec_set_wpm(uint8_t wpm) { cw_rx_set_wpm(wpm); }  

// cw_rx.cpp — рядом с cw_rx_set_wpm  
void cw_rx_set_sql(float lvl) {          // 0 = сквелч off, типично 0.15..0.4  
    if (lvl < 0.0f || lvl > 0.9f) return;  
    sq_level = lvl;  
    if (lvl == 0.0f) sq_open = true;     // off = всегда открыт  
}  
  
void cwdec_stop(void) {  
    cwdec_q_head = cwdec_q_tail = 0;   // хвост очереди выбрасываем  
    cw_rx_stop();  
}  
  
uint32_t cwdec_dropped(void) { return cwdec_dropped_cnt; }    
  
void cwdec_set_sql(float lvl) { cw_rx_set_sql(lvl); }    
  
uint8_t cwdec_wpm(void) { return cw_wpm; }    
float   cwdec_sql(void) { return sq_level; }  