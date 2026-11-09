// src/af_cli.cpp  
// ============================================================================  
// af_cli.cpp — Serial CLI (Core 0)  
//  
// Команды (по одной на строку, завершение \n):  
//   band <lo> <hi>     — полоса ПФ в Гц, напр. "band 300 3000"  
//   nt on|off          — автонотч (подавление тональников)  
//   nr on|off          — шумоподавление  
//   cw on|off          — телеграфный фильтр (АРУ -> медленный профиль)  
//   cw freq <hz>       — центральная частота CW (300..3000)  
//   cw bw <hz>         — полоса CW (80..500)  
//   cw noise <0..2000> — примесь шума эфира в CW
//   stats              — диагностика: загрузка ядер, потери, клипы  
//   banks clr          — перезаписать все банки заводскими пресетами (ребут)
//   help               — справка  
// ============================================================================  
#include <Arduino.h>  
#include <string.h>  
#include <stdlib.h>  
#include "af_cli.h"  
#include "af_config.h"  
#include "af_dsp.h"  
#include "af_capture.h"  
#include "af_pwm.h"  
#include "af_usb.h"
#include "af_store.h"
#include "af_led.h"
  
// ---------------------------------------------------------------------------  
// Справка  
// ---------------------------------------------------------------------------  
static void cli_help(void) {  
    Serial.println("# Команды:");  
    Serial.println("#   band <lo> <hi>    — полоса ПФ, Гц (напр. band 300 3000)");  
    Serial.println("#   nt on|off         — автонотч NT");  
    Serial.println("#   nr on|off         — шумоподавление NR");  
    Serial.println("#   cw on|off         — телеграфный фильтр");  
    Serial.println("#   cw freq <hz>      — центральная частота CW (300..3000)");  
    Serial.println("#   cw bw <hz>        — полоса CW (80..500)");  
    Serial.println("#   cw noise <0..100> — примесь шума эфира в CW, %");  
    Serial.println("#   stats             — диагностика");  
    Serial.println("#   help / h / ?      — эта справка");  
    Serial.println("#   save <1..7>       — сохранить настройки в ячейку");  
    Serial.println("#   load <0..7>       — применить ячейку (0 = ручной режим)");  
    Serial.println("#   banks             — показать все ячейки");  
    Serial.println("#   banks clr         — перезаписать все банки заводскими пресетами (ребут)"); 
    Serial.println("# GPIO: GP2,3,4=банк (двоичный код, 000=ручной), GP5=NR, GP6=NT (LOW=on)");
}  
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
static void cli_stats(void) {  
    Serial.printf("# band=%lu..%lu Hz  nt=%s  nr=%s  cw=%s f0=%lu bw=%lu noise=%lu%%\n",  
                  (unsigned long)af_dsp_band_lo_hz(),  
                  (unsigned long)af_dsp_band_hi_hz(),
                  af_dsp_notch_on() ? "on" : "off",  
                  af_dsp_nr_on()    ? "on" : "off",  
                  af_dsp_cw_on()    ? "on" : "off",  
                  (unsigned long)af_dsp_cw_freq(),  
                  (unsigned long)af_dsp_cw_bw(),  
                  (unsigned long)af_dsp_cw_noise());  
    Serial.printf("# core1_load=%lu.%lu%%  cap_dropped=%lu  out_dropped=%lu  clip=%lu\n",  
                  (unsigned long)(af_dsp_load_permille() / 10u),  
                  (unsigned long)(af_dsp_load_permille() % 10u),  
                  (unsigned long)af_capture_dropped(),  
                  (unsigned long)af_dsp_out_dropped(),  
                  (unsigned long)af_dsp_clip_count());  
    Serial.printf("# cap: isr=%lu fcs_over=%lu q_full=%lu scratch=%lu busy=%lu\n",  
                  (unsigned long)af_cap_isr_count,  
                  (unsigned long)af_adc_fcs_over,  
                  (unsigned long)af_drop_queue_full,  
                  (unsigned long)af_drop_scratch,  
                  (unsigned long)af_drop_busy);  
    Serial.printf("# pwm: underruns=%lu fill=%u\n",  
                  (unsigned long)af_pwm_underruns(),  
                  (unsigned)af_dsp_out_fill_pwm());
    Serial.printf("# usb: mounted=%d dropped=%lu\n",  
                  (int)af_usb_mounted(),  
                  (unsigned long)af_usb_dropped());
    Serial.printf("# core1: max_block_us=%lu\n", (unsigned long)af_dsp_max_block_us());
}  
  
// ---------------------------------------------------------------------------  
// Разбор одной строки  
// ---------------------------------------------------------------------------  
static void cli_exec(char* line) {    
    // Пропуск ведущих пробелов; пустые строки и комментарии — мимо.    
    while (*line == ' ' || *line == '\t') ++line;    
    if (*line == '\0' || *line == '#') return;    
    
    Serial.printf("# > %s\n", line);      // эхо введённой команды    
    
    char* cmd = strtok(line, " \t");    
    if (cmd == NULL) return;
  
    if (strcasecmp(cmd, "help") == 0 ||    
        strcasecmp(cmd, "h") == 0 ||    
        strcmp(cmd, "?") == 0) {    
        cli_help();    
    }
    else if (strcmp(cmd, "stats") == 0) {  
        cli_stats();  
    }  
    else if (strcmp(cmd, "band") == 0) {  
        char* s_lo = strtok(NULL, " \t");  
        char* s_hi = strtok(NULL, " \t");  
        if (s_lo == NULL || s_hi == NULL) {  
            Serial.println("# ERR: band <lo> <hi>");  
            return;  
        }  
        uint32_t lo = (uint32_t)strtoul(s_lo, NULL, 10);  
        uint32_t hi = (uint32_t)strtoul(s_hi, NULL, 10);  
        if (af_dsp_set_band(lo, hi)) {    
            Serial.printf("# OK band=%lu..%lu Hz\n",    
                          (unsigned long)af_dsp_band_lo_hz(),    
                          (unsigned long)af_dsp_band_hi_hz());    
        } else {    
            Serial.println("# band: rejected (CW on — полоса следует за f0)");    
        } 
    }

    else if (strcmp(cmd, "nt") == 0) {  
        char* s = strtok(NULL, " \t");  
        if (s == NULL) { Serial.println("# ERR: nt on|off"); return; }  
        bool on = (strcmp(s, "on") == 0);  
        int rc = af_dsp_set_notch(on);    
        Serial.printf("# OK nt=%s\n", on ? "on" : "off");    
        if (rc == 2)    
            Serial.println("# note: NR off (приоритет NT, совместная работа не поддерживается)");
    }  
    else if (strcmp(cmd, "nr") == 0) {  
        char* s = strtok(NULL, " \t");  
        if (s == NULL) { Serial.println("# ERR: nr on|off"); return; }  
        bool on = (strcmp(s, "on") == 0);  
        if (af_dsp_set_nr(on)) {    
            Serial.printf("# OK nr=%s\n", on ? "on" : "off");    
        } else {    
            Serial.println("# nr: rejected (NT on — приоритет NT)");    
        }
    }  
    else if (strcmp(cmd, "cw") == 0) {  
        char* sub = strtok(NULL, " \t");  
        if (sub == NULL) { Serial.println("# ERR: cw on|off|freq <hz>|bw <hz>|noise <0..100>"); return; }  
  
        if (strcmp(sub, "on") == 0) {  
            af_dsp_set_cw(true);  
            Serial.println("# OK cw=on");  
        }  
        else if (strcmp(sub, "off") == 0) {  
            af_dsp_set_cw(false);  
            Serial.println("# OK cw=off");  
        }  
        else if (strcmp(sub, "freq") == 0) {  
            char* v = strtok(NULL, " \t");  
            if (v == NULL) { Serial.println("# ERR: cw freq <hz>"); return; }  
            af_dsp_set_cw_freq((uint32_t)strtoul(v, NULL, 10));  
            Serial.printf("# OK cw_freq=%lu Hz\n", (unsigned long)af_dsp_cw_freq());  
        }  
        else if (strcmp(sub, "bw") == 0) {  
            char* v = strtok(NULL, " \t");  
            if (v == NULL) { Serial.println("# ERR: cw bw <80..500>"); return; }  
            af_dsp_set_cw_bw((uint32_t)strtoul(v, NULL, 10));  
            Serial.printf("# OK cw_bw=%lu Hz\n", (unsigned long)af_dsp_cw_bw());  
        }  
        else if (strcmp(sub, "noise") == 0) {  
            char* v = strtok(NULL, " \t");  
            if (v == NULL) { Serial.println("# ERR: cw noise <0..100>"); return; }  
            af_dsp_set_cw_noise((uint32_t)strtoul(v, NULL, 10));  
            Serial.printf("# OK cw_noise=%lu%%\n", (unsigned long)af_dsp_cw_noise());  
        }  
        else {  
            Serial.println("# ERR: cw on|off|freq <hz>|bw <hz>|noise <0..100>");  
        }  
    }

    else if (strcmp(cmd, "save") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: save <1..7>"); return; }  
        int idx = (int)strtoul(v, NULL, 10);  
        if (idx < 1 || idx > AF_STORE_BANKS) {  
            Serial.println("# ERR: save <1..7>");  
            return;  
        }  
        af_store_save((uint8_t)idx);  
        Serial.printf("# OK saved bank %d\n", idx);  
    }  
    else if (strcmp(cmd, "load") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: load <0..7>"); return; }  
        int idx = (int)strtoul(v, NULL, 10);  
        if (idx == 0) {                          // выход в ручной режим  
            af_led_set_bank(0);  
            Serial.println("# OK manual mode (bank 0)");  
            return;  
        }  
        if (idx < 1 || idx > AF_STORE_BANKS) {  
            Serial.println("# ERR: load <0..7>");  
            return;  
        } 
        if (af_store_load(idx)) {  
            af_led_set_bank((uint8_t)idx);   // цвет вспышки = загруженный банк  
            Serial.printf("# OK loaded bank %d\n", idx);
        } else {  
            Serial.printf("# bank %d: empty — ignored\n", idx);  
        }  
    }  
    else if (strcmp(cmd, "banks") == 0) {  
        char* sub = strtok(NULL, " \t");  
        if (sub && strcmp(sub, "clr") == 0) {  
            // Перезаписать все 7 ячеек заводскими пресетами (ребут).  
            static const af_bank_t preset[AF_STORE_BANKS] = {  
                {  100, 4000, 0, 700, 150, 400 },  // 1: широкая  
                {  300, 2400, 0, 700, 150, 400 },  // 2: нормальная  
                {  300, 1800, 0, 700, 150, 400 },  // 3: узкая  
                {  500, 1100, 1, 800, 150, 400 },  // 4: CW 800 Гц  
                {  300,  900, 0, 700, 150, 400 },  // 5: узкая  
                {  300, 3000, 1, 800, 200, 400 },  // 6: CW 800 Гц  
                {  200, 3500, 0, 700, 150, 400 },  // 7: резерв/широкая  
            };  
            Serial.println("# banks clr: writing 7 banks, rebooting...");  
            Serial.flush();  
            af_store_write_all(preset);   // не возвращается — ребут  
        }  
        // Без аргументов — список ячеек, как раньше.  
        for (uint8_t i = 1; i <= AF_STORE_BANKS; i++) {  
            af_bank_t b;  
            if (af_store_read(i, &b)) {  
                Serial.printf("# bank %d: band=%lu..%lu cw=%s f0=%lu bw=%lu noise=%lu\n",  
                              i, (unsigned long)b.lo_hz, (unsigned long)b.hi_hz,  
                              b.cw_on ? "on" : "off",  
                              (unsigned long)b.cw_f0, (unsigned long)b.cw_bw,  
                              (unsigned long)b.cw_noise);  
            } else {  
                Serial.printf("# bank %d: empty\n", i);  
            }  
        }  
    }

    // вставлять сюда

    else {  
        Serial.printf("# ERR: unknown '%s' (help — справка)\n", cmd);  
    }  
}

// ---------------------------------------------------------------------------  
// CAT: подмножество Kenwood TS-2000 (порт fldigi_cat.cpp из приёмника).  
// Команды оканчиваются ';'. Маппинг на AF-тракт:  
//   NT1/NT0 -> автонотч, NR1/NR0 -> шумодав, SH/SL -> полоса ПФ,  
//   MD3 -> CW on, MD1/MD2 -> CW off. FA/IF/ID — заглушки для fldigi.  
// ---------------------------------------------------------------------------  
static char cat_mode = '2';   // 1=LSB 2=USB 3=CW — для ответа MD;  
  
static void cat_send(const char* s) {  
    Serial.write((const uint8_t*)s, strlen(s));  
    Serial.flush();  
}  
  
static const uint16_t SL_TABLE[12] = {  
      0,   50,  100,  200,  300,  400,  500,  600,  700,  800,  900, 1000  
};  
static const uint16_t SH_TABLE[12] = {  
   1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3400, 4000, 5000  
};  
  
static void cat_execute(const char* cmd) {  
    size_t n = strlen(cmd);  
    if (n < 2) { cat_send("?;"); return; }  
  
    // FA — частоты нет (AF-процессор), отвечаем нулём для совместимости  
    if (cmd[0] == 'F' && cmd[1] == 'A') {  
        if (n == 2) { cat_send("FA00000000000;"); }  
        return;                                    // set-FA игнорируем  
    }  
  
    // IF — статус (шаблон TS-2000)  
    if (cmd[0] == 'I' && cmd[1] == 'F' && n == 2) {  
        cat_send("IF00000000000       -00300   20000000;");  
        return;  
    }  
  
    // ID — идентификация (019 = TS-2000)  
    if (cmd[0] == 'I' && cmd[1] == 'D' && n == 2) { cat_send("ID019;"); return; }  
  
    // MD — режим: 3=CW -> af_dsp_set_cw(true), остальные -> cw off  
    if (cmd[0] == 'M' && cmd[1] == 'D') {  
        if (n == 2) {  
            char out[8];  
            snprintf(out, sizeof(out), "MD%c;", cat_mode);  
            cat_send(out);  
            return;  
        }  
        if (n == 3) {  
            switch (cmd[2]) {  
                case '1': case '2': case '4': case '5':  
                    af_dsp_set_cw(false);  
                    cat_mode = cmd[2];  
                    return;  
                case '3':  
                    af_dsp_set_cw(true);  
                    cat_mode = '3';  
                    return;  
                default:  
                    cat_send("?;");  
                    return;  
            }  
        }  
        cat_send("?;");  
        return;  
    }  
  
    // Заглушки PTT (fldigi шлёт TX1;/RX;)  
    if (cmd[0] == 'T' && cmd[1] == 'X' && n == 3) { return; }  
    if (cmd[0] == 'R' && cmd[1] == 'X' && n == 2) { return; }  
  
    // NT — автонотч: NT1 вкл, NT0 выкл  
    if (cmd[0] == 'N' && cmd[1] == 'T') {    
        if (n == 2 || n == 3) {  
            if (n == 3) af_dsp_set_notch(cmd[2] != '0');  
            char buf[12];  
            snprintf(buf, sizeof(buf), "NT%d;NR%d;",  
                     af_dsp_notch_on() ? 1 : 0, af_dsp_nr_on() ? 1 : 0);  
            cat_send(buf);  
            return;  
        }  
        cat_send("?;");    
        return;    
    }
  
    // NR — шумоподавление: NR; запрос, NR1/NR0  
    if (cmd[0] == 'N' && cmd[1] == 'R') {  
        if (n == 2) {  
            char buf[8];  
            snprintf(buf, sizeof(buf), "NR%d;", af_dsp_nr_on() ? 1 : 0);  
            cat_send(buf);  
            return;  
        }  
        if (n == 3) {  
            af_dsp_set_nr(cmd[2] == '1');   // false при NT on — состояние не меняется  
            char buf[12];  
            snprintf(buf, sizeof(buf), "NR%d;NT%d;",  
                     af_dsp_nr_on() ? 1 : 0, af_dsp_notch_on() ? 1 : 0);  
            cat_send(buf);  
            return;  
        }
    }
  
    // SH — верхний срез: SH; запрос, SHnn индекс, SHnnnn Гц  
    if (cmd[0] == 'S' && cmd[1] == 'H') {  
        if (n == 2) {  
            char out[12];  
            snprintf(out, sizeof(out), "SH%04lu;", (unsigned long)af_dsp_band_hi_hz());  
            cat_send(out);  
            return;  
        }  
        uint32_t f_hi;  
        if (n == 4) {  
            int idx = (int)strtoul(cmd + 2, NULL, 10);  
            if (idx < 0 || idx > 11) { cat_send("?;"); return; }  
            f_hi = SH_TABLE[idx];  
        } else if (n == 6) {  
            f_hi = (uint32_t)strtoul(cmd + 2, NULL, 10);  
        } else { cat_send("?;"); return; }  
        if (!af_dsp_set_band(af_dsp_band_lo_hz(), f_hi)) {    
            cat_send("?;");   // CW on — полоса следует за f0    
        }    
        return;
    }  
  
    // SL — нижний срез: SL; запрос, SLnn индекс, SLnnnn Гц  
    if (cmd[0] == 'S' && cmd[1] == 'L') {  
        if (n == 2) {  
            char out[12];  
            snprintf(out, sizeof(out), "SL%04lu;", (unsigned long)af_dsp_band_lo_hz());  
            cat_send(out);  
            return;  
        }  
        uint32_t f_lo;  
        if (n == 4) {  
            int idx = (int)strtoul(cmd + 2, NULL, 10);  
            if (idx < 0 || idx > 11) { cat_send("?;"); return; }  
            f_lo = SL_TABLE[idx];  
        } else if (n == 6) {  
            f_lo = (uint32_t)strtoul(cmd + 2, NULL, 10);  
        } else { cat_send("?;"); return; }  
        if (!af_dsp_set_band(f_lo, af_dsp_band_hi_hz())) {    
            cat_send("?;");   // CW on — полоса следует за f0    
        }    
        return;
    }  
  
    cat_send("?;");  
}
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void af_cli_init(void) {  
    Serial.println("# CLI ready. 'help' — список команд.");  
}  
  
void af_cli_poll(void) {  
    static char line[64];  
    static uint8_t pos = 0;  
  
    while (Serial.available() > 0) {  
        char c = (char)Serial.read();  
        if (c == '\r') continue;  
        if (c == '\n') {  
            line[pos] = '\0';  
            cli_exec(line);  
            pos = 0;  
        } else if (c == ';') {              // CAT-команда Kenwood  
            line[pos] = '\0';  
            cat_execute(line);  
            pos = 0;  
        } else if (pos < sizeof(line) - 1) {  
            line[pos++] = c;  
        } else {  
            pos = 0;  
        }
    }  
}