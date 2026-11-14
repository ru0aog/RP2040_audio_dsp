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
//   gain <0..1000>     — ручное усиление при agc off, множитель
//   agc on|off         — АРУ (off -> ручное усиление, лимитер остаётся)  
//   bypass on|off      — обход: выход = вход  
//   voice [N|vol <0..400>] — голосовые сообщения
//   stats              — диагностика: загрузка ядер, потери, клипы  
//   banks clr          — перезаписать все банки заводскими пресетами (ребут)
//   pwm on|off         — PWM-выход (off = несущая заглушена, пин в 0)
//   ramp on|off        — пила-счётчик вместо DSP (тест целостности потока)  
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
#include "af_voice.h"     // подмес голосовых клипов в выход
#include "af_tx.h"        // TX-тракт: полоса/компрессия/клиппер  

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
    Serial.println("#   cw noise <0..2000> — примесь шума эфира в CW");
    Serial.println("#   stats             — диагностика");  
    Serial.println("#   help / h / ?      — эта справка");  
    Serial.println("#   save <1..7>       — сохранить настройки в ячейку");  
    Serial.println("#   load <0..7>       — применить ячейку (0 = ручной режим)");  
    Serial.println("#   banks             — показать все ячейки");
    Serial.println("#   banks clr         — перезаписать все банки заводскими пресетами (ребут)");
    Serial.println("#   agc on|off        — АРУ (off -> ручное усиление, лимитер остаётся)");
    Serial.println("#   gain <0..8>       — ручное усиление при agc off, множитель (1.0 = единичное)");
    Serial.println("#   bypass on|off     — обход: выход = вход");  
    Serial.println("#   ramp on|off       — пила-счётчик вместо DSP (тест целостности потока)");
    Serial.println("#   tx on|off         — передающий тракт (PTT: GP8 LOW=TX)");    
    Serial.println("#   txband <lo> <hi>  — речевая полоса TX, Гц (напр. 300 2700)");  
    Serial.println("#   txcomp <x10>      — компрессия x10: 10=1:1, 20=2:1, 40=4:1");  
    Serial.println("#   txclip <0..32767> — порог клиппера TX (меньше = плотнее)");    
    Serial.println("#   txagc <0..32767>  — целевой уровень TX-АРУ (0 = АРУ off)");  
    Serial.println("#   txgate <0..32767> — порог шумового гейта TX (0 = гейт off)");
    Serial.println("#   txap on|off       — all-pass симметризатор пиков TX");  
    Serial.println("#   txmode soft|hard  — тип клиппера TX");
    Serial.println("#   tx2band on|off    — 2-полосная компрессия TX (split ~900 Гц)");  
    Serial.println("#   txgfloor <0..65536> — нижняя ступень гейта Q16 (4120=-24дБ, 0=тишина)");  
    Serial.println("#   txnotch on|off    — узкие нотчи 1/2 кГц против тонов-наводок");  
    Serial.println("#   txpreset <aggro|max|flat|def> — пресет речевого процессора одной командой");    
    Serial.println("# GPIO: GP2,3,4=банк (двоичный код, 000=ручной), GP5=NR, GP6=NT, GP7=bypass, GP8=PTT (LOW=on)");    
}  
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
static void cli_stats(void) {  
    uint32_t g100 = (uint32_t)(af_dsp_gain() * 100.0f + 0.5f);  
        Serial.printf("# band=%lu..%lu Hz  nt=%s  nr=%s  cw=%s f0=%lu bw=%lu noise=%lu  agc=%s gain=%lu.%02lu  bypass=%s  tx=%s\n",
                  (unsigned long)af_dsp_band_lo_hz(),  
                  (unsigned long)af_dsp_band_hi_hz(),
                  af_dsp_notch_on() ? "on" : "off",  
                  af_dsp_nr_on()    ? "on" : "off",  
                  af_dsp_cw_on()    ? "on" : "off",  
                  (unsigned long)af_dsp_cw_freq(),  
                  (unsigned long)af_dsp_cw_bw(),  
                  (unsigned long)af_dsp_cw_noise(),
                  af_dsp_agc_on() ? "on" : "off",  
                  (unsigned long)(g100 / 100), (unsigned long)(g100 % 100),
                  af_dsp_bypass_on() ? "on" : "off",
                  af_dsp_tx_on() ? "on" : "off");
    Serial.printf("# ramp=%s\n", af_dsp_ramp_on() ? "on" : "off");    
    Serial.printf("# tx: band=%lu..%lu Hz  comp=%lu:1(x10)  clip=%lu  agc=%lu  gate=%lu  gfloor=%lu  ap=%s  mode=%s  2band=%s  notch=%s\n",      
                  (unsigned long)af_tx_lo_hz(),          
                  (unsigned long)af_tx_hi_hz(),          
                  (unsigned long)af_tx_ratio(),          
                  (unsigned long)af_tx_clip(),        
                  (unsigned long)af_tx_agc(),        
                  (unsigned long)af_tx_gate(),      
                  (unsigned long)af_tx_gfloor(),    
                  af_tx_ap() ? "on" : "off",      
                  af_tx_softclip() ? "soft" : "hard",    
                  af_tx_2band() ? "on" : "off",  
                  af_tx_notch() ? "on" : "off");    
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
    Serial.printf("# pwm: %s underruns=%lu fill=%u\n",  
              af_pwm_is_enabled() ? "on" : "OFF",  
              (unsigned long)af_pwm_underruns(), af_dsp_out_fill_pwm());
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
        if (on) af_voice_play(VOICE_NT_ON);
        Serial.printf("# OK nt=%s\n", on ? "on" : "off");    
        if (rc == 2)    
            Serial.println("# note: NR off (приоритет NT, совместная работа не поддерживается)");
    }  
    else if (strcmp(cmd, "nr") == 0) {  
        char* s = strtok(NULL, " \t");  
        if (s == NULL) { Serial.println("# ERR: nr on|off"); return; }  
        bool on = (strcmp(s, "on") == 0);    
        if (af_dsp_set_nr(on)) {      
            if (on) af_voice_play(VOICE_NR_ON);   // "адаптивное шумоподавление включено"  
            Serial.printf("# OK nr=%s\n", on ? "on" : "off");      
        } else {      
            Serial.println("# nr: rejected (NT on — приоритет NT)");      
        }
    }
    else if (strcmp(cmd, "agc") == 0) {  
        char* a = strtok(NULL, " \t");  
        if (a && strcmp(a, "on") == 0)       { af_dsp_set_agc(true);  Serial.println("# OK agc=on"); }  
        else if (a && strcmp(a, "off") == 0) {
            af_dsp_set_agc(false);
            af_voice_play(VOICE_AGC_OFF);   // "АРУ выключена"
            Serial.println("# OK agc=off"); }  
        else Serial.println("# ERR: agc on|off");  
    }  
    else if (strcmp(cmd, "gain") == 0) {    
        char* a = strtok(NULL, " \t");    
        if (a) {    
            float v = strtof(a, NULL);              // множитель: 0..AGC_MAX_GAIN    
            af_dsp_set_gain(v);    
            uint32_t g100 = (uint32_t)(af_dsp_gain() * 100.0f + 0.5f);  
            Serial.printf("# OK gain=%lu.%02lu\n",  
                          (unsigned long)(g100 / 100), (unsigned long)(g100 % 100));
        } else Serial.println("# ERR: gain <0..1000> — множитель при agc off (1.0 = единичное)");    
    }
    else if (strcmp(cmd, "bypass") == 0) {  
        char* a = strtok(NULL, " \t");  
        if (a && strcmp(a, "on") == 0)       {
            af_dsp_set_bypass(true);
            af_voice_play(VOICE_BYPASS_ON);   // "режим обхода включен"
            Serial.println("# OK bypass=on"); }
        else if (a && strcmp(a, "off") == 0) { af_dsp_set_bypass(false); Serial.println("# OK bypass=off"); }  
        else Serial.println("# ERR: bypass on|off");  
    }
    else if (strcmp(cmd, "cw") == 0) {  
        char* sub = strtok(NULL, " \t");  
        if (sub == NULL) { Serial.println("# ERR: cw on|off|freq <hz>|bw <hz>|noise <0..100>"); return; }  
  
        if (strcmp(sub, "on") == 0) {  
            af_dsp_set_cw(true);
            af_voice_play(VOICE_CW_ON);
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
            // Поля: {lo_hz, hi_hz, cw_on, cw_f0, cw_bw, cw_noise, agc_on, gain_pct, voice_vol}  
            //   lo_hz/hi_hz — полоса ПФ, Гц  
            //   cw_on       — телеграфный фильтр (0/1)  
            //   cw_f0       — центральная частота CW, Гц  
            //   cw_bw       — полоса CW, Гц  
            //   cw_noise    — примесь шума в CW, уровень 0..2000  
            //   agc_on      — АРУ (0/1); при 0 — ручное усиление  
            //   gain_q8     — ручное усиление при agc off, Q8 (256 = ×1.0) 
            //   voice_vol   — громкость голосовых сообщений, % (0..400) 
            static const af_bank_t preset[AF_STORE_BANKS] = {  
                {  100, 3100, 0, 700, 150, 400, 1, 256, 300 },  // 1 (GP2): SSB нормальная  
                {  300, 2200, 0, 700, 150, 400, 1, 256, 300 },  // 2 (GP3): SSB узкая  
                {  100, 4000, 0, 700, 150, 400, 1, 256, 300 },  // 3: резерв — широкая  
                {  450,  950, 1, 700, 150, 400, 1, 256, 300 },  // 4 (GP4): CW 700 Гц  
                {  300,  900, 0, 700, 150, 400, 1, 256, 300 },  // 5: резерв — очень узкая  
                {  300, 3000, 0, 700, 150, 400, 0, 384, 300 },  // 6: резерв — AM, АРУ off (×1.5)  
                {  200, 3500, 0, 700, 150, 400, 1, 256, 300 },  // 7: резерв  
            };
            Serial.println("# banks clr: writing 7 banks, rebooting...");  
            Serial.flush();  
            af_store_write_all(preset);   // не возвращается — ребут  
        }  
        // Без аргументов — список ячеек, как раньше.  
        for (uint8_t i = 1; i <= AF_STORE_BANKS; i++) {  
            af_bank_t b;  
            if (af_store_read(i, &b)) {  
                uint32_t g100 = (uint32_t)b.gain_pct * 100u / 256u;  
                Serial.printf("# bank %d: band=%lu..%lu cw=%s f0=%lu bw=%lu noise=%lu agc=%s gain=%lu.%02lu vvol=%lu%%\n",
                              i, (unsigned long)b.lo_hz, (unsigned long)b.hi_hz,      
                              b.cw_on ? "on" : "off",      
                              (unsigned long)b.cw_f0, (unsigned long)b.cw_bw,      
                              (unsigned long)b.cw_noise, b.agc_on ? "on" : "off",    
                              (unsigned long)(g100 / 100), (unsigned long)(g100 % 100),  
                              (unsigned long)b.voice_vol);
            } else {  
                Serial.printf("# bank %d: empty\n", i);  
            }  
        }  
    }

    else if (strcmp(cmd, "pwm") == 0) {  
        char* s = strtok(NULL, " \t");  
        if (s && strcmp(s, "on") == 0) {  
            af_pwm_set_enabled(true);  
            Serial.println("# OK pwm=on");  
        } else if (s && strcmp(s, "off") == 0) {  
            af_pwm_set_enabled(false);  
            Serial.println("# OK pwm=off (carrier muted)");  
        } else {  
            Serial.println("# ERR: pwm on|off");  
        }  
    }

    else if (strcmp(cmd, "voice") == 0) {  
        char* a1 = strtok(NULL, " \t");  
        if (a1 && strcmp(a1, "vol") == 0) {    
            char* a2 = strtok(NULL, " \t");    
            if (a2) {    
                af_voice_set_volume((uint16_t)strtoul(a2, NULL, 10));    
                Serial.printf("# OK voice vol=%u%%\n", (unsigned)af_voice_volume());    
            } else {    
                Serial.printf("# voice vol=%u%%\n", (unsigned)af_voice_volume());    
            }  
        } else if (a1) {  
            long n = atol(a1);  
            if (n >= 1 && n < VOICE_COUNT) {  
                af_voice_play((af_voice_id_t)n);  
                Serial.printf("# OK voice %ld\n", n);  
            } else {  
                Serial.println("# ERR: voice <1..N|vol <0..400>>");  
            }  
        } else {  
            af_voice_play(VOICE_READY);  
            Serial.println("# OK voice");  
        }  
    }

    else if (strcmp(cmd, "ramp") == 0) {  
        char* a = strtok(NULL, " \t");  
        if (a && strcmp(a, "on") == 0) {  
            af_dsp_set_ramp(true);  
            Serial.println("# OK ramp=on (пила-счётчик вместо DSP)");  
        }  
        else if (a && strcmp(a, "off") == 0) {  
            af_dsp_set_ramp(false);  
            Serial.println("# OK ramp=off");  
        }  
        else Serial.println("# ERR: ramp on|off");  
    }  

    else if (strcmp(cmd, "tx") == 0) {  
        char* a = strtok(NULL, " \t");  
        if (a && strcmp(a, "on") == 0)       { af_dsp_set_tx(true);  Serial.println("# OK tx=on"); }  
        else if (a && strcmp(a, "off") == 0) { af_dsp_set_tx(false); Serial.println("# OK tx=off"); }  
        else Serial.println("# ERR: tx on|off");  
    }  

    else if (strcmp(cmd, "txband") == 0) {  
        char* s_lo = strtok(NULL, " \t");  
        char* s_hi = strtok(NULL, " \t");  
        if (s_lo == NULL || s_hi == NULL) {  
            Serial.println("# ERR: txband <lo> <hi>");  
            return;  
        }  
        af_tx_set_band((uint32_t)strtoul(s_lo, NULL, 10),  
                       (uint32_t)strtoul(s_hi, NULL, 10));  
        Serial.printf("# OK txband=%lu..%lu Hz\n",  
                      (unsigned long)af_tx_lo_hz(),  
                      (unsigned long)af_tx_hi_hz());  
    }  
    else if (strcmp(cmd, "txcomp") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txcomp <x10>"); return; }  
        af_tx_set_ratio((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txcomp=%lu\n", (unsigned long)af_tx_ratio());  
    }  
    else if (strcmp(cmd, "txclip") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txclip <0..32767>"); return; }  
        af_tx_set_clip((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txclip=%lu\n", (unsigned long)af_tx_clip());  
    }  

    else if (strcmp(cmd, "txagc") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txagc <0..32767>"); return; }  
        af_tx_set_agc((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txagc=%lu\n", (unsigned long)af_tx_agc());  
    }  
    else if (strcmp(cmd, "txgate") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txgate <0..32767>"); return; }  
        af_tx_set_gate((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txgate=%lu\n", (unsigned long)af_tx_gate());  
    }  

    else if (strncmp(cmd, "txap", 4) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_tx_set_ap(true);  Serial.println("# OK txap=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_tx_set_ap(false); Serial.println("# OK txap=off"); }  
        else Serial.println("# ERR: txap on|off");  
    }  
    else if (strncmp(cmd, "txmode", 6) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "soft") == 0)     { af_tx_set_softclip(true);  Serial.println("# OK txmode=soft"); }  
        else if (v && strcmp(v, "hard") == 0){ af_tx_set_softclip(false); Serial.println("# OK txmode=hard"); }  
        else Serial.println("# ERR: txmode soft|hard");  
    }  

    else if (strncmp(cmd, "tx2band", 7) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_tx_set_2band(true);  Serial.println("# OK tx2band=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_tx_set_2band(false); Serial.println("# OK tx2band=off"); }  
        else Serial.println("# ERR: tx2band on|off");  
    }  

    else if (strncmp(cmd, "txgfloor", 8) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txgfloor <0..65536>"); return; }  
        af_tx_set_gfloor((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txgfloor=%lu\n", (unsigned long)af_tx_gfloor());  
    }  

    else if (strncmp(cmd, "txpreset", 8) == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txpreset <aggro|max|flat|def>"); return; }    
    
        if (strcmp(v, "aggro") == 0) {          // агрессивный «кирпич»    
            af_tx_set_band(300, 2700);    
            af_tx_set_ratio(80);                // 8:1    
            af_tx_set_clip(8192);               // 0.25 FS    
            af_tx_set_agc(18000);    
            af_tx_set_gate(800);    
            af_tx_set_gfloor(4120);             // -24 дБ    
            af_tx_set_ap(true);    
            af_tx_set_softclip(true);    
            af_tx_set_2band(true);    
        }  
        else if (strcmp(v, "max") == 0) {       // максимальная плотность    
            af_tx_set_band(300, 2200);    
            af_tx_set_ratio(100);               // 10:1 — лимитер    
            af_tx_set_clip(4096);               // мин. порог    
            af_tx_set_agc(18000);    
            af_tx_set_gate(800);    
            af_tx_set_gfloor(4120);    
            af_tx_set_ap(true);    
            af_tx_set_softclip(true);    
            af_tx_set_2band(true);    
        }    
        else if (strcmp(v, "flat") == 0) {      // линейный тракт (только FIR+АРУ)    
            af_tx_set_band(300, 2700);    
            af_tx_set_ratio(10);                // 1:1 — компрессия off    
            af_tx_set_clip(32767);              // клиппер off    
            af_tx_set_agc(18000);    
            af_tx_set_gate(0);                  // гейт off    
            af_tx_set_gfloor(4120);    
            af_tx_set_ap(false);    
            af_tx_set_softclip(true);    
            af_tx_set_2band(false);    
        }    
        else if (strcmp(v, "def") == 0) {       // заводские дефолты из af_config.h    
            af_tx_set_band(AF_TX_LO_HZ_DEFAULT, AF_TX_HI_HZ_DEFAULT);    
            af_tx_set_ratio(AF_TX_RATIO_X10_DEFAULT);    
            af_tx_set_clip(AF_TX_CLIP_DEFAULT);    
            af_tx_set_agc(AF_TX_AGC_TARGET);    
            af_tx_set_gate(AF_TX_GATE_THR);    
            af_tx_set_gfloor(AF_TX_GATE_FLOOR_Q16);    
            af_tx_set_ap(AF_TX_AP_ON_DEFAULT != 0);    
            af_tx_set_softclip(AF_TX_SOFTCLIP_ON_DEFAULT != 0);    
            af_tx_set_2band(AF_TX_2BAND_ON_DEFAULT != 0);    
        }    
        else { Serial.println("# ERR: txpreset <aggro|max|flat|def>"); return; }    
    
        Serial.printf("# OK txpreset=%s  band=%lu..%lu comp=%lu clip=%lu agc=%lu gate=%lu gfloor=%lu ap=%s mode=%s 2band=%s\n",    
                      v,    
                      (unsigned long)af_tx_lo_hz(),    
                      (unsigned long)af_tx_hi_hz(),    
                      (unsigned long)af_tx_ratio(),    
                      (unsigned long)af_tx_clip(),    
                      (unsigned long)af_tx_agc(),    
                      (unsigned long)af_tx_gate(),    
                      (unsigned long)af_tx_gfloor(),    
                      af_tx_ap() ? "on" : "off",    
                      af_tx_softclip() ? "soft" : "hard",    
                      af_tx_2band() ? "on" : "off");    
    }    

    else if (strncmp(cmd, "txnotch", 7) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_tx_set_notch(true);  Serial.println("# OK txnotch=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_tx_set_notch(false); Serial.println("# OK txnotch=off"); }  
        else Serial.println("# ERR: txnotch on|off");  
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
                    af_voice_play(VOICE_CW_ON);
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
            if (n == 3) {  
                af_dsp_set_notch(cmd[2] != '0');  
                if (cmd[2] != '0') af_voice_play(VOICE_NT_ON);  
            }
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