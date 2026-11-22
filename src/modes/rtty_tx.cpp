// src/modes/rtty_tx.cpp — RTTY-передатчик 45.45/170, ITA-2 + МТК-2.  
//  
// FSK с непрерывной фазой: один NCO (LUT-синус), на границе бита  
// переключается только шаг фазы mark<->space — без щелчков и лепестков.  
// Посылка: start(space,1бит) + 5 бит данных LSB-first + stop(mark,1.5бита).  
// Между посылками — mark (idle), в конце — тишина и возврат в RX.  
//  
// Кодировка: ASCII -> ITA-2 (LTRS/FIGS), кириллица UTF-8 -> МТК-2 (RUS).  
// Сдвиги регистров вставляются автоматически при смене алфавита:  
//   LTRS=0x1F, FIGS=0x1B, RUS=0x00 (blank МТК-2).  
//  
// Разделение по ядрам (как cwtx):  
//   Core 0: rttytx_init/send/poll — кодирование и бит-автомат.  
//   Core 1: rttytx_sample() — NCO + выбор шага, только int32/LUT.  
// ============================================================================  
#include "rtty_tx.h"  
#include "../af_config.h"     // AF_FS_OUT_HZ  
#include "af_oled.h"          // эхо передаваемого текста на OLED  
#include <math.h>  
  
// ---------------------------------------------------------------------------  
// Таблицы Бодо — те же, что в rtty_rx.cpp (приёмник). Индекс = 5-битный код.  
// ---------------------------------------------------------------------------  
static const char baudot_ltrs[32] = {  
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0 };  
static const char baudot_figs[32] = {  
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',  
    '5','+',')','2','#','6','0','1','9','?','&',0,'.','/',';',0 };  
// МТК-2: кириллица как пары UTF-8 байт (0xD0/0xD1 + второй байт)  
static const uint16_t baudot_rus[32] = {  
    0,      0x95D0, 0,     0x90D0, 0,     0xA1D0, 0x98D0, 0xA3D0,  // Е А С И У  
    0,      0x94D0, 0xA0D0, 0x99D0, 0x9DD0, 0xA4D0, 0xA6D0, 0x9AD0,  // Д Р Й Н Ф Ц К  
    0xA2D0, 0x97D0, 0x9BD0, 0x92D0, 0xA5D0, 0xABD0, 0x9FD0, 0xAFD0,  // Т З Л В Х Ы П Я  
    0x9ED0, 0x91D0, 0x93D0, 0,      0x9CD0, 0xACD0, 0xA8D0, 0 };     // О Б Г М Ь Ш  
  
#define RTTYTX_LTRS  0x1F  
#define RTTYTX_FIGS  0x1B  
#define RTTYTX_RUS   0x00    // blank -> RUS-регистр МТК-2  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static int16_t  tx_lut[RTTYTX_LUT_LEN];        // Q15 синус  
static uint32_t nco_step_mark, nco_step_space; // шаги фазы  
static volatile uint32_t nco_step;             // текущий шаг (Core 1 читает)  
static uint32_t nco_phase;                     // аккумулятор фазы (непрерывный)  
  
static uint32_t bit_len;                       // отсчётов на бит (Fs/45.45)  
static uint32_t stop_len;                      // отсчётов на стоп (1.5 бита)  
  
// Очередь кодов Бодо (после вставки сдвигов) — заполняется в send()  
#define RTTYTX_CODE_LEN  192u  
static uint8_t  tx_codes[RTTYTX_CODE_LEN];  
static volatile uint8_t  tx_clen;              // кодов в очереди  
static volatile uint8_t  tx_cpos;              // текущий код  
  
// Эхо-строка исходного текста для OLED (печать по мере звучания)  
#define RTTYTX_TEXT_LEN  96u  
static char     tx_echo[RTTYTX_TEXT_LEN];  
static volatile uint8_t  tx_elen;  
static volatile uint8_t  tx_epos;  
  
// Бит-автомат (Core 0 пишет, Core 1 читает)  
static volatile bool     tx_bit;               // текущий бит: 1=mark 0=space  
static volatile uint32_t tx_bit_rem;           // отсчётов до конца бита  
static volatile bool     tx_active;            // идёт передача  
  
static uint8_t  cur_code;                      // код в посылке  
static uint8_t  bit_idx;                       // 0=start 1..5=data 6=stop  
static bool     tail_mark;                     // хвост mark перед концом  
  
// ---------------------------------------------------------------------------  
// Core 1: следующий отсчёт. CPFSK — фаза непрерывна, меняется только шаг.  
// ---------------------------------------------------------------------------  
int16_t rttytx_sample(void) {  
    if (!tx_active) return 0;  
    if (tx_bit_rem == 0) {  
        // Core 0 ещё не подал следующий бит — держим mark (idle)  
        nco_phase += nco_step_mark;  
        return tx_lut[(nco_phase >> 24) & (RTTYTX_LUT_LEN - 1)];  
    }  
    tx_bit_rem--;  
    nco_phase += nco_step;  
    return tx_lut[(nco_phase >> 24) & (RTTYTX_LUT_LEN - 1)];  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: подача одного бита в tx_bit/tx_bit_rem  
// ---------------------------------------------------------------------------  
static void feed_bit(bool mark, uint32_t len) {  
    tx_bit = mark;  
    nco_step = mark ? nco_step_mark : nco_step_space;  
    tx_bit_rem = len;  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: бит-автомат. Вызывать из loop() каждую итерацию.  
// ---------------------------------------------------------------------------  
void rttytx_poll(void) {  
    if (!tx_active) return;  
    if (tx_bit_rem != 0) return;               // Core 1 доигрывает бит  
  
    // биты текущей посылки: 0=start 1..5=data 6=stop  
    if (bit_idx <= 6) {  
        if (bit_idx == 0) {  
            feed_bit(false, bit_len);          // start = space  
        } else if (bit_idx <= 5) {  
            feed_bit((cur_code >> (bit_idx - 1)) & 1, bit_len);  // LSB first  
        } else {  
            feed_bit(true, stop_len);          // stop = mark, 1.5 бита  
        }  
        bit_idx++;  
        return;  
    }  
  
    // следующий код из очереди  
    if (tx_cpos < tx_clen) {  
        cur_code = tx_codes[tx_cpos++];  
        bit_idx = 0;  
        // эхо на OLED: печатаем исходный символ, когда его коды начали звучать.  
        // Сдвиги эхо не дают — печать по исходному тексту, по одному символу  
        // на каждый НЕ-сдвиговый код.  
        if (cur_code != RTTYTX_LTRS && cur_code != RTTYTX_FIGS &&    
            cur_code != RTTYTX_RUS && tx_epos < tx_elen) {    
            char c = tx_echo[tx_epos++];    
            af_oled_putc(c);    
            // UTF-8 кириллица — пара байт: допечатать второй сразу,    
            // иначе эхо отстанет и хвост строки потеряется    
            if ((uint8_t)c == 0xD0 || (uint8_t)c == 0xD1) {    
                if (tx_epos < tx_elen) af_oled_putc(tx_echo[tx_epos++]);    
            }    
        }    
        return;  
    }  
  
    // очередь пуста — хвост mark ~100 мс, потом конец передачи  
    if (!tail_mark) {  
        tail_mark = true;  
        feed_bit(true, (AF_FS_OUT_HZ * 100u) / 1000u);  
    } else {  
        tx_active = false;  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Кодирование: символ -> код Бодо + нужный регистр.  
// reg: 0=LTRS 1=FIGS 2=RUS. Возвращает код или 0xFF (символ не кодируется).  
// ---------------------------------------------------------------------------  
static uint8_t encode_char(uint8_t c, uint8_t c2, int &reg, uint8_t &shift) {  
    shift = 0xFF;   // 0xFF = сдвиг не нужен  
  
    // Кириллица UTF-8: c=0xD0/0xD1, c2=второй байт    
    if (c == 0xD0 || c == 0xD1) {    
        uint16_t u = ((uint16_t)c << 8) | c2;    
        // строчные -> заглавные: а..п (0xB0..0xBF) -> А..П (0x90..0x9F),    
        // р..я (0xD1 0x80..0x8F) -> Р..Я (0xD0 0xA0..0xAF)    
        if (c == 0xD0 && c2 >= 0xB0 && c2 <= 0xBF) u = 0xD000 | (c2 - 0x20);    
        else if (c == 0xD1 && c2 >= 0x80 && c2 <= 0x8F) u = 0xD000 | (c2 + 0x20);    
        if (u == 0x81D0 || u == 0x91D0) u = 0x95D0;   // Ё/ё -> Е    
        for (uint8_t i = 0; i < 32; i++)    
            if (baudot_rus[i] == u) {    
                if (reg != 2) { shift = RTTYTX_RUS; reg = 2; }    
                return i;    
            }    
        return 0xFF;    
    }    
  
    // ASCII -> верхний регистр  
    if (c >= 'a' && c <= 'z') c -= 32;  
  
    // Пробел/переводы — одинаковы во всех регистрах, сдвиг не нужен  
    if (c == ' ')  return 0x04;  
    if (c == '\n') return 0x02;  
    if (c == '\r') return 0x08;  
  
    for (uint8_t i = 0; i < 32; i++)  
        if (baudot_ltrs[i] == (char)c) {  
            if (reg != 0) { shift = RTTYTX_LTRS; reg = 0; }  
            return i;  
        }  
    for (uint8_t i = 0; i < 32; i++)  
        if (baudot_figs[i] == (char)c) {  
            if (reg != 1) { shift = RTTYTX_FIGS; reg = 1; }  
            return i;  
        }  
    return 0xFF;    // не кодируется — пропуск  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: API  
// ---------------------------------------------------------------------------  
void rttytx_init(void) {  
    for (uint32_t i = 0; i < RTTYTX_LUT_LEN; i++)  
        tx_lut[i] = (int16_t)(sinf(2.0f * (float)M_PI * (float)i / RTTYTX_LUT_LEN) * 32767.0f);  
    nco_step_mark  = (uint32_t)(((uint64_t)RTTYTX_MARK_HZ  << 32) / AF_FS_OUT_HZ);  
    nco_step_space = (uint32_t)(((uint64_t)RTTYTX_SPACE_HZ << 32) / AF_FS_OUT_HZ);  
    nco_step  = nco_step_mark;  
    nco_phase = 0;  
    bit_len  = (uint32_t)((float)AF_FS_OUT_HZ / RTTYTX_BAUD + 0.5f);   // ~340  
    stop_len = bit_len + bit_len / 2;                                  // 1.5 бита  
    tx_active = false;  
    tx_clen = tx_cpos = 0;  
    tx_elen = tx_epos = 0;  
    tx_bit_rem = 0;  
    tail_mark = false;  
    Serial.printf("[RTTYTX] init mark=%u space=%u bit=%lu smp\n",  
                  RTTYTX_MARK_HZ, RTTYTX_SPACE_HZ, (unsigned long)bit_len);  
}  
  
bool rttytx_send(const char* text) {  
    if (tx_active) return false;  
  
    // Текст -> коды Бодо со сдвигами; параллельно — эхо-строка для OLED  
    int reg = 0;                        // стартовый регистр — LTRS  
    uint8_t n = 0, e = 0;  
    for (uint8_t i = 0; text[i] && n < RTTYTX_CODE_LEN - 4; i++) {  
        uint8_t c  = (uint8_t)text[i];  
        uint8_t c2 = 0;  
        if (c == 0xD0 || c == 0xD1) {   // UTF-8 кириллица — два байта  
            if (!text[i + 1]) break;  
            c2 = (uint8_t)text[++i];  
        }  
        uint8_t shift;  
        int r = reg;  
        uint8_t code = encode_char(c, c2, r, shift);  
        reg = r;  
        if (code == 0xFF) continue;     // не кодируется  
        if (shift != 0xFF) tx_codes[n++] = shift;  
        tx_codes[n++] = code;  
        if (e < RTTYTX_TEXT_LEN - 1) {  // эхо — исходные байты (UTF-8 парами)  
            tx_echo[e++] = (char)c;  
            if (c2) tx_echo[e++] = (char)c2;  
        }  
    }  
    if (n == 0) return false;  
  
    tx_clen = n; tx_cpos = 0;  
    tx_elen = e; tx_epos = 0;  
    bit_idx = 7;                        // "посылка доиграна" -> сразу новый код  
    tx_bit_rem = 0;  
    tail_mark = false;  
    af_oled_putc('>');                  // префикс TX на экране, как у CW  
    tx_active = true;  
    return true;  
}  
  
bool rttytx_busy(void) { return tx_active; }  
  
void rttytx_abort(void) {  
    tx_active = false;  
    tx_bit_rem = 0;  
    tx_clen = tx_cpos = 0;  
    tx_elen = tx_epos = 0;  
    tail_mark = false;  
}  
