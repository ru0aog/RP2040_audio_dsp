// src/modes/af_oled.cpp — SSD1306 128x32 I2C, bit-bang, Core 0.  
#include "af_oled.h"  
#include "../af_config.h"  
#include "../af_dsp.h"  
  
// ---------------------------------------------------------------------------  
// Шрифт 5x7, ASCII 0x20..0x7F (классический SSD1306 font, column-major)  
// ---------------------------------------------------------------------------  
static const uint8_t FONT5x7[][5] = {  
    {0x00,0x00,0x00,0x00,0x00}, // ' '  
    {0x00,0x00,0x5F,0x00,0x00}, // '!'  
    {0x00,0x07,0x00,0x07,0x00}, // '"'  
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'  
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'  
    {0x23,0x13,0x08,0x64,0x62}, // '%'  
    {0x36,0x49,0x55,0x22,0x50}, // '&'  
    {0x00,0x05,0x03,0x00,0x00}, // '''  
    {0x00,0x1C,0x22,0x41,0x00}, // '('  
    {0x00,0x41,0x22,0x1C,0x00}, // ')'  
    {0x14,0x08,0x3E,0x08,0x14}, // '*'  
    {0x08,0x08,0x3E,0x08,0x08}, // '+'  
    {0x00,0x50,0x30,0x00,0x00}, // ','  
    {0x08,0x08,0x08,0x08,0x08}, // '-'  
    {0x00,0x60,0x60,0x00,0x00}, // '.'  
    {0x20,0x10,0x08,0x04,0x02}, // '/'  
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'  
    {0x00,0x42,0x7F,0x40,0x00}, // '1'  
    {0x42,0x61,0x51,0x49,0x46}, // '2'  
    {0x21,0x41,0x45,0x4B,0x31}, // '3'  
    {0x18,0x14,0x12,0x7F,0x10}, // '4'  
    {0x27,0x45,0x45,0x45,0x39}, // '5'  
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'  
    {0x01,0x71,0x09,0x05,0x03}, // '7'  
    {0x36,0x49,0x49,0x49,0x36}, // '8'  
    {0x06,0x49,0x49,0x29,0x1E}, // '9'  
    {0x00,0x36,0x36,0x00,0x00}, // ':'  
    {0x00,0x56,0x36,0x00,0x00}, // ';'  
    {0x08,0x14,0x22,0x41,0x00}, // '<'  
    {0x14,0x14,0x14,0x14,0x14}, // '='  
    {0x00,0x41,0x22,0x14,0x08}, // '>'  
    {0x02,0x01,0x51,0x09,0x06}, // '?'  
    {0x32,0x49,0x79,0x41,0x3E}, // '@'  
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'  
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'  
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'  
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'  
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'  
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'  
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'  
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'  
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'  
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'  
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'  
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'  
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'  
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'  
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'  
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'  
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'  
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'  
    {0x46,0x49,0x49,0x49,0x31}, // 'S'  
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'  
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'  
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'  
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'  
    {0x63,0x14,0x08,0x14,0x63}, // 'X'  
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'  
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'  
    {0x00,0x7F,0x41,0x41,0x00}, // '['  
    {0x02,0x04,0x08,0x10,0x20}, // '\'  
    {0x00,0x41,0x41,0x7F,0x00}, // ']'  
    {0x04,0x02,0x01,0x02,0x04}, // '^'  
    {0x40,0x40,0x40,0x40,0x40}, // '_'  
    {0x00,0x01,0x02,0x04,0x00}, // '`'  
    {0x20,0x54,0x54,0x54,0x78}, // 'a'  
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'  
    {0x38,0x44,0x44,0x44,0x20}, // 'c'  
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'  
    {0x38,0x54,0x54,0x54,0x18}, // 'e'  
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'  
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'  
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'  
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'  
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'  
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'  
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'  
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'  
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'  
    {0x38,0x44,0x44,0x44,0x38}, // 'o'  
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'  
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'  
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'  
    {0x48,0x54,0x54,0x54,0x20}, // 's'  
    {0x04,0x3F,0x44,0x40,0x20}, // 't'  
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'  
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'  
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'  
    {0x44,0x28,0x10,0x28,0x44}, // 'x'  
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'  
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'  
    {0x00,0x08,0x36,0x41,0x00}, // '{'  
    {0x00,0x00,0x7F,0x00,0x00}, // '|'  
    {0x00,0x41,0x36,0x08,0x00}, // '}'  
    {0x10,0x08,0x08,0x10,0x08}, // '~'  
};  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static bool     oled_ok;                    // модуль ответил на адрес  
static bool     oled_dirty;                 // буфер изменился — нужна перерисовка  
static uint32_t oled_last_ms;               // троттлинг перерисовки  
static char     oled_lines[AF_OLED_ROWS][AF_OLED_COLS + 1];  // текстовый буфер  
static uint8_t  oled_col;                   // позиция в текущей строке скролла  
  
// ---------------------------------------------------------------------------  
// Bit-bang I2C (open-drain: 0 = output low, 1 = input с внешней подтяжкой)  
// ---------------------------------------------------------------------------  
static inline void sda_low(void)  { pinMode(AF_OLED_SDA_PIN, OUTPUT); digitalWrite(AF_OLED_SDA_PIN, LOW); }  
static inline void sda_rel(void)  { pinMode(AF_OLED_SDA_PIN, INPUT_PULLUP); }  
static inline void scl_low(void)  { pinMode(AF_OLED_SCL_PIN, OUTPUT); digitalWrite(AF_OLED_SCL_PIN, LOW); }  
static inline void scl_rel(void)  { pinMode(AF_OLED_SCL_PIN, INPUT_PULLUP); }  
static inline bool sda_rd(void)   { return digitalRead(AF_OLED_SDA_PIN); }  
  
static void i2c_start(void) {  
    sda_rel(); scl_rel(); delayMicroseconds(2);  
    sda_low(); delayMicroseconds(2);  
    scl_low(); delayMicroseconds(2);  
}  
static void i2c_stop(void) {  
    sda_low(); delayMicroseconds(2);  
    scl_rel(); delayMicroseconds(2);  
    sda_rel(); delayMicroseconds(2);  
}  
static bool i2c_write(uint8_t b) {          // true = ACK  
    for (int i = 7; i >= 0; i--) {  
        if (b & (1 << i)) sda_rel(); else sda_low();  
        delayMicroseconds(1);  
        scl_rel(); delayMicroseconds(2);  
        scl_low(); delayMicroseconds(1);  
    }  
    sda_rel(); delayMicroseconds(1);  
    scl_rel(); delayMicroseconds(1);  
    bool ack = !sda_rd();  
    scl_low(); delayMicroseconds(1);  
    return ack;  
}  
  
// ---------------------------------------------------------------------------  
// SSD1306 primitives  
// ---------------------------------------------------------------------------  
static bool oled_cmd(uint8_t c) {  
    i2c_start();  
    if (!i2c_write(AF_OLED_ADDR << 1)) { i2c_stop(); return false; }  
    i2c_write(0x00);                        // Co=0, D/C=0 — команда  
    i2c_write(c);  
    i2c_stop();  
    return true;  
}  
  
static void oled_data(const uint8_t* d, uint16_t n) {  
    i2c_start();  
    i2c_write(AF_OLED_ADDR << 1);  
    i2c_write(0x40);                        // Co=0, D/C=1 — данные  
    for (uint16_t i = 0; i < n; i++) i2c_write(d[i]);  
    i2c_stop();  
}  
  
// ---------------------------------------------------------------------------  
// Рендер текстового буфера -> GDDRAM (page mode, 4 страницы по 8 px)  
// ---------------------------------------------------------------------------  
static void oled_render(void) {  
    for (uint8_t page = 0; page < AF_OLED_ROWS; page++) {  
        oled_cmd(0xB0 + page);              // page address  
        oled_cmd(0x00);                     // col low  
        oled_cmd(0x10);                     // col high  
        uint8_t buf[AF_OLED_W];  
        for (uint8_t col = 0; col < AF_OLED_COLS; col++) {  
            char c = oled_lines[page][col];  
            const uint8_t* g = (c >= 0x20 && c <= 0x7E)  
                             ? FONT5x7[c - 0x20] : FONT5x7[0];  
            for (uint8_t k = 0; k < 5; k++) buf[col * 6 + k] = g[k];  
            buf[col * 6 + 5] = 0x00;        // межсимвольный зазор  
        }  
        buf[126] = 0; buf[127] = 0;  
        oled_data(buf, AF_OLED_W);  
    }  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void af_oled_init(void) {  
    sda_rel(); scl_rel();  
    delay(50);                              // SSD1306 power-up  
    // Проверка присутствия: ACK на адрес  
    i2c_start();  
    oled_ok = i2c_write(AF_OLED_ADDR << 1);  
    i2c_stop();  
    if (!oled_ok) {  
        Serial.println("# OLED: not found at 0x3C — disabled");  
        return;  
    }  
    // Init-последовательность SSD1306 128x32  
    static const uint8_t init_seq[] = {  
        0xAE,       // display off  
        0xD5, 0x80, // clock div  
        0xA8, 0x1F, // multiplex = 31 (32 строки)  
        0xD3, 0x00, // display offset  
        0x40,       // start line 0  
        0x8D, 0x14, // charge pump on  
        0x20, 0x00, // horizontal addressing  
        0xA1,       // segment remap (flip)  
        0xC8,       // COM scan dec  
        0xDA, 0x02, // COM pins config (128x32)  
        0x81, 0x8F, // contrast  
        0xD9, 0xF1, // precharge  
        0xDB, 0x40, // VCOMH  
        0xA4,       // display from RAM  
        0xA6,       // normal (не инверсия)  
        0xAF,       // display on  
    };  
    for (uint8_t i = 0; i < sizeof(init_seq); i++) oled_cmd(init_seq[i]);  
    af_oled_clear();  
    af_oled_status();  
    Serial.println("# OLED: SSD1306 128x32 @0x3C ok");  
}  
  
void af_oled_clear(void) {  
    if (!oled_ok) return;  
    for (uint8_t r = 0; r < AF_OLED_ROWS; r++) {  
        memset(oled_lines[r], ' ', AF_OLED_COLS);  
        oled_lines[r][AF_OLED_COLS] = '\0';  
    }  
    oled_col = 0;  
    oled_dirty = true;  
}  
  
void af_oled_putc(char c) {                 // скролл-вывод в строки 1..3  
    if (!oled_ok) return;  
    if (c == '\n' || oled_col >= AF_OLED_COLS) {  
        // скролл вверх: строка 1 <- 2 <- 3  
        memcpy(oled_lines[1], oled_lines[2], AF_OLED_COLS + 1);  
        memcpy(oled_lines[2], oled_lines[3], AF_OLED_COLS + 1);  
        memset(oled_lines[3], ' ', AF_OLED_COLS);  
        oled_lines[3][AF_OLED_COLS] = '\0';  
        oled_col = 0;  
        if (c == '\n') { oled_dirty = true; return; }  
    }  
    oled_lines[3][oled_col++] = c;  
    oled_dirty = true;  
}  
  
void af_oled_status(void) {                 // строка 0: режим + полоса + f0  
    if (!oled_ok) return;  
    char* s = oled_lines[0];  
    if (af_dsp_cwtx_busy()) {  
        snprintf(s, AF_OLED_COLS + 1, "TX CW %luHz",  
                 (unsigned long)af_dsp_cw_freq());  
    } else if (af_dsp_cw()) {  
        snprintf(s, AF_OLED_COLS + 1, "RX CW %luHz",  
                 (unsigned long)af_dsp_cw_freq());  
    } else {  
        snprintf(s, AF_OLED_COLS + 1, "RX %lu-%lu",  
                 (unsigned long)af_dsp_band_lo_hz(),  
                 (unsigned long)af_dsp_band_hi_hz());  
    }  
    // добить пробелами до ширины  
    size_t l = strlen(s);  
    while (l < AF_OLED_COLS) s[l++] = ' ';  
    s[AF_OLED_COLS] = '\0';  
    oled_dirty = true;  
}  
  
void af_oled_poll(void) {  
    if (!oled_ok || !oled_dirty) return;  
    uint32_t now = millis();  
    if ((uint32_t)(now - oled_last_ms) < 250u) return;  // ~4 обновления/сек  
    oled_last_ms = now;  
    oled_dirty = false;  
    oled_render();  
}  
  
bool af_oled_present(void) { return oled_ok; }  
