// src/af_cli.cpp  
// ============================================================================  
// af_cli.cpp — Serial CLI (Core 0)  
//  
// Команды (по одной на строку, завершение \n). Полный список — help.  
//   RX:  band nt nr rxnotch cw agc gain bypass  
//        agcref agcmax agcmin limceil nrdepth nrmakeup ntmu nrmu  
//   TX:  tx txband txap txagc txagcmax txgate txgatehold txgatehyst txgfloor  
//        tx2band txsplit txlr2 txcomp txcompthr txcompthrlo txcompthrhi  
//        txduck txduckmax txintel txinteltgt txintelcut txintelboost  
//        txpres txpresmax txpresfreq txpresq txshelf txshelffreq txshelfq  
//        txexc txexcmix txmode txclip txnotch txnotchf txsetdef txprof    
//   SYS: stats save load banks pwm voice ramp help  
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
#include "af_gpio_ctl.h"  // af_gpio_bank_code — текущий банк с пинов  
  
// Загрузка Core 0 из RP2040_audio_dsp.cpp (busy-время loop, ‰).  
extern uint32_t af_core0_load_permille(void);  

// ---------------------------------------------------------------------------  
// Справка  
// ---------------------------------------------------------------------------  
static void cli_help(void) {    
    Serial.println("# === ПРИЁМ (RX) ===");    
    Serial.println("#   band <lo> <hi>    — полоса ПФ, Гц (напр. band 300 3000)");    
    Serial.println("#   nt on|off         — автонотч NT");    
    Serial.println("#   ntmu <0..32768>   — шаг NLMS нотча (эксперт)");    
    Serial.println("#   nr on|off         — шумоподавление NR");    
    Serial.println("#   nrmu <0..32768>   — шаг LMS NR (эксперт)");    
    Serial.println("#   nrdepth <q15>     — глубина подавления NR (32768 = полная)");    
    Serial.println("#   nrmakeup <q15>    — makeup-gain NR (65536 = x2)");    
    Serial.println("#   rxnotch on|off    — узкие нотчи 1/2/3/4 кГц в RX (дефолт on)");    
    Serial.println("#   cw on|off         — телеграфный фильтр");    
    Serial.println("#   cw freq <hz>      — центральная частота CW (300..3000)");    
    Serial.println("#   cw bw <hz>        — полоса CW (80..500)");    
    Serial.println("#   cw noise <0..2000> — примесь шума эфира в CW");  
    Serial.println("#   agc on|off        — АРУ (off -> ручное усиление, лимитер остаётся)");  
    Serial.println("#   agcref <1000..32767> — целевой уровень АРУ (SSB)");    
    Serial.println("#   agcmax <1..1000>  — потолок усиления АРУ, множитель");    
    Serial.println("#   agcmin <0.01..1>  — нижний предел АРУ, множитель");    
    Serial.println("#   limceil <1000..32767> — потолок лимитера");    
    Serial.println("#   gain <0..8>       — ручное усиление при agc off, множитель (1.0 = единичное)");  
    Serial.println("#   bypass on|off     — обход: выход = вход");    
    Serial.println("# === ПЕРЕДАЧА (TX) ===");    
    Serial.println("#   tx on|off         — передающий тракт (PTT: GP8 LOW=TX)");      
    Serial.println("#   txband <lo> <hi>  — речевая полоса TX, Гц (напр. 300 2700)");    
    Serial.println("#   txap on|off       — all-pass симметризатор пиков TX");    
    Serial.println("#   txagc <0..32767>  — целевой уровень TX-АРУ (0 = АРУ off)");    
    Serial.println("#   txagcmax <x10>    — потолок TX-АРУ (200 = 20x)");    
    Serial.println("#   txgate <0..32767> — порог шумового гейта TX (0 = гейт off)");  
    Serial.println("#   txgatehold <мс>   — hangover гейта (0..2000)");    
    Serial.println("#   txgatehyst <x10>  — гистерезис гейта (7 = закрытие по thr*0.7)");    
    Serial.println("#   txgfloor <0..65536> — нижняя ступень гейта Q16 (4120=-24дБ, 0=тишина)");    
    Serial.println("#   tx2band on|off    — 2-полосная компрессия TX");    
    Serial.println("#   txsplit <300..3000> — частота кроссовера, Гц");    
    Serial.println("#   txlr2 on|off      — кроссовер LR2 + раздельный клиппинг полос");        
    Serial.println("#   txcomp <x10>      — компрессия x10: 10=1:1, 20=2:1, 40=4:1");    
    Serial.println("#   txcompthr <256..32767>   — порог компрессора 1-полосного");    
    Serial.println("#   txcompthrlo <256..32767> — порог НЧ-полосы");    
    Serial.println("#   txcompthrhi <256..32767> — порог ВЧ-полосы");    
    Serial.println("#   txduck on|off     — HF ducking: НЧ приглушается при доминировании ВЧ");        
    Serial.println("#   txduckmax <дБx10> — потолок ослабления НЧ (30 = -3 дБ)");        
    Serial.println("#   txintel on|off    — k_intelligibility: автобаланс НЧ/ВЧ");      
    Serial.println("#   txinteltgt <q15>  — цель ratio НЧ/ВЧ (32768 = 1.0)");    
    Serial.println("#   txintelcut <q15>  — потолок ослабления ВЧ (16384 = -6дБ)");    
    Serial.println("#   txintelboost <q15> — потолок подъёма ВЧ (32768 = +6дБ)");    
    Serial.println("#   txpres on|off     — dynamic presence EQ (подъём по SNR)");    
    Serial.println("#   txpresmax <дБx10> — потолок подъёма presence (30=+3дБ, 60=+6дБ)");      
    Serial.println("#   txpresfreq <500..6000> — центр presence-пика, Гц");    
    Serial.println("#   txpresq <10..500> — добротность presence x100 (100 = 1.00)");    
    Serial.println("#   txshelf <дБx10>   — статический подъём ВЧ (0=off, 40=+4дБ)");    
    Serial.println("#   txshelffreq <500..6000> — частота полки, Гц");    
    Serial.println("#   txshelfq <10..500> — добротность полки x100");    
    Serial.println("#   txexc on|off      — ВЧ-эксайтер: гармоники 2-3 кГц");        
    Serial.println("#   txexcmix <q15>    — подмес эксайтера (3277 = -20 дБ)");  
    Serial.println("#   txmode soft|hard  — тип клиппера TX");  
    Serial.println("#   txclip <0..32767> — порог клиппера TX (меньше = плотнее)");      
    Serial.println("#   txnotch on|off    — узкие нотчи против тонов-наводок");  
    Serial.println("#   txnotchf <f1> <f2> — частоты TX-нотчей, Гц (200..7000)");    
    Serial.println("#   txprof <1..7|имя> — профиль передачи: 1 flat, 2 natural, 3 dx, 4 bass, 5 whisper, 6 loud, 7 max");  
    Serial.println("#   txsetdef          — сброс профиля передачи к дефолтам"); 
    Serial.println("# === СИСТЕМА ===");    
    Serial.println("#   stats             — диагностика");    
    Serial.println("#   save <1..7>       — сохранить настройки в ячейку");    
    Serial.println("#   load <0..7>       — применить ячейку (0 = ручной режим)");    
    Serial.println("#   banks             — показать все ячейки");  
    Serial.println("#   banks clr         — перезаписать все банки заводскими пресетами (ребут)");  
    Serial.println("#   pwm on|off        — PWM-выход (off = несущая заглушена)");  
    Serial.println("#   voice [N|vol <0..400>] — голосовые сообщения");  
    Serial.println("#   ramp on|off       — пила-счётчик вместо DSP (тест потока)");  
    Serial.println("#   help / h / ?      — эта справка");    
    Serial.println("# === GPIO ===");    
    Serial.println("# GP2,3,4=банк (двоичный код, 000=ручной), GP5=NR, GP6=NT, GP7=bypass, GP8=PTT (LOW=on)");      
}
  
// ---------------------------------------------------------------------------  
// Диагностика  
// ---------------------------------------------------------------------------  
static void cli_stats(void) {      
    uint32_t g100 = (uint32_t)(af_dsp_gain() * 100.0f + 0.5f);      
    bool tx = af_dsp_tx_on();      
    
    // ---- RX: параметры приёмного тракта (при tx=on неактивен) ----      
    Serial.printf("# === RX (%s) ===\n", tx ? "inactive" : "active");      
    Serial.printf("#   band    = %lu..%lu Hz\n",      
                  (unsigned long)af_dsp_band_lo_hz(),      
                  (unsigned long)af_dsp_band_hi_hz());      
    Serial.printf("#   nt      = %s   nr      = %s   bypass  = %s   ramp    = %s\n",      
                  af_dsp_notch_on()  ? "on" : "off",      
                  af_dsp_nr_on()     ? "on" : "off",      
                  af_dsp_bypass_on() ? "on" : "off",      
                  af_dsp_ramp_on()   ? "on" : "off");      
    Serial.printf("#   cw      = %s   f0=%lu bw=%lu noise=%lu\n",      
                  af_dsp_cw_on() ? "on" : "off",      
                  (unsigned long)af_dsp_cw_freq(),      
                  (unsigned long)af_dsp_cw_bw(),      
                  (unsigned long)af_dsp_cw_noise());      
    Serial.printf("#   agc     = %s   gain=%lu.%02lu\n",      
                  af_dsp_agc_on() ? "on" : "off",      
                  (unsigned long)(g100 / 100), (unsigned long)(g100 % 100));      
    Serial.printf("#   agcref  = %ld   agcmax=%ld agcmin=%ld   limceil=%ld\n",      
                  (long)af_dsp_agcref(),      
                  (long)(af_dsp_agcmax() * 100.0f + 0.5f),      
                  (long)(af_dsp_agcmin() * 100.0f + 0.5f),      
                  (long)af_dsp_limceil());      
    Serial.printf("#   nrdepth = %ld   nrmakeup=%ld   ntmu=%ld nrmu=%ld\n",      
                  (long)af_dsp_nrdepth(),      
                  (long)af_dsp_nrmakeup(),      
                  (long)af_dsp_ntmu(),      
                  (long)af_dsp_nrmu());      
    
    // ---- TX: параметры передающего тракта (при tx=off неактивен) ----      
    Serial.printf("# === TX (%s) ===\n", tx ? "active" : "inactive");      
    Serial.printf("#   band    = %lu..%lu Hz   mode=%s\n",      
                  (unsigned long)af_tx_lo_hz(),      
                  (unsigned long)af_tx_hi_hz(),      
                  af_tx_softclip() ? "soft" : "hard");      
    Serial.printf("#   comp    = %lu:1(x10)   clip=%lu   agc=%lu agcmax=%lu\n",      
                  (unsigned long)af_tx_ratio(),      
                  (unsigned long)af_tx_clip(),      
                  (unsigned long)af_tx_agc(),      
                  (unsigned long)af_tx_agcmax());      
    Serial.printf("#   gate    = %lu   hold=%lu hyst=%lu   gfloor=%lu\n",      
                  (unsigned long)af_tx_gate(),      
                  (unsigned long)af_tx_gatehold(),      
                  (unsigned long)af_tx_gatehyst(),      
                  (unsigned long)af_tx_gfloor());      
    Serial.printf("#   ap      = %s   2band=%s split=%lu lr2=%s   compthr=%ld/%ld/%ld\n",      
                  af_tx_ap()    ? "on" : "off",      
                  af_tx_2band() ? "on" : "off",      
                  (unsigned long)af_tx_split(),      
                  af_tx_lr2()   ? "on" : "off",      
                  (long)af_tx_compthr(), (long)af_tx_compthrlo(), (long)af_tx_compthrhi());      
    Serial.printf("#   pres    = %s pmax=%lu f=%lu q=%lu   shelf=%lu f=%lu q=%lu\n",      
                  af_tx_pres() ? "on" : "off",      
                  (unsigned long)af_tx_presmax(),      
                  (unsigned long)af_tx_presfreq(),      
                  (unsigned long)af_tx_presq(),      
                  (unsigned long)af_tx_shelf(),      
                  (unsigned long)af_tx_shelffreq(),      
                  (unsigned long)af_tx_shelfq());      
    Serial.printf("#   duck    = %s dmax=%lu   exc=%s emix=%lu   intel=%s %ld/%ld/%ld\n",      
                  af_tx_duck() ? "on" : "off",      
                  (unsigned long)af_tx_duckmax(),      
                  af_tx_exc()  ? "on" : "off",      
                  (unsigned long)af_tx_excmix(),      
                  af_tx_intel() ? "on" : "off",      
                  (long)af_tx_inteltgt(), (long)af_tx_intelcut(), (long)af_tx_intelboost());      
    Serial.printf("#   notch   = %s   f=%lu/%lu Hz\n",      
                  af_tx_notch() ? "on" : "off",      
                  (unsigned long)af_tx_notch1(),      
                  (unsigned long)af_tx_notch2());      
    Serial.printf("#   txenv   : agc_env=%lu agc_gain=%lu env_lo=%lu env_hi=%lu\n",      
                  (unsigned long)af_tx_agc_env(),      
                  (unsigned long)af_tx_agc_gain(),      
                  (unsigned long)af_tx_env_lo(),      
                  (unsigned long)af_tx_env_hi());      
    Serial.printf("#             hi_pre=%lu nf=%lu k=%lu kd=%lu gi=%lu\n",      
                  (unsigned long)af_tx_env_hi_pre(),      
                  (unsigned long)af_tx_pres_nf(),      
                  (unsigned long)af_tx_k_dyn(),      
                  (unsigned long)af_tx_k_duck(),      
                  (unsigned long)af_tx_g_intel());      
    
    // ---- SYS: общее состояние, работает в обоих режимах ----      
    Serial.println("# === SYS ===");      
    Serial.printf("#   mode    = %s   bank=%u   voice=%u%%   pwm=%s   usb=%s\n",      
                  tx ? "tx" : "rx",      
                  (unsigned)af_gpio_bank_code(),      
                  (unsigned)af_voice_volume(),      
                  af_pwm_is_enabled() ? "on" : "OFF",      
                  af_usb_mounted() ? "mounted" : "no");      
    Serial.printf("#   core0   : load=%lu.%lu%%\n",      
                  (unsigned long)(af_core0_load_permille() / 10u),      
                  (unsigned long)(af_core0_load_permille() % 10u));      
    Serial.printf("#   core1   : load=%lu.%lu%% max_block_us=%lu\n",      
                  (unsigned long)(af_dsp_load_permille() / 10u),      
                  (unsigned long)(af_dsp_load_permille() % 10u),      
                  (unsigned long)af_dsp_max_block_us());      
    Serial.printf("#   cap     : isr=%lu fcs_over=%lu q_full=%lu scratch=%lu busy=%lu dropped=%lu\n",      
                  (unsigned long)af_cap_isr_count,      
                  (unsigned long)af_adc_fcs_over,      
                  (unsigned long)af_drop_queue_full,      
                  (unsigned long)af_drop_scratch,      
                  (unsigned long)af_drop_busy,      
                  (unsigned long)af_capture_dropped());      
    Serial.printf("#   out     : dropped=%lu clip=%lu   usb: ur=%lu fill_max=%u   pwm: ur=%lu fill_max=%u\n",      
                  (unsigned long)af_dsp_out_dropped(),      
                  (unsigned long)af_dsp_clip_count(),      
                  (unsigned long)af_dsp_out_underruns_usb(),      
                  (unsigned)af_dsp_out_fill_max_usb(),      
                  (unsigned long)af_dsp_out_underruns_pwm(),      
                  (unsigned)af_dsp_out_fill_max_pwm());      
    Serial.printf("#   fs      : out=%lu adc=%lu div=%lu\n",      
                  (unsigned long)AF_FS_OUT_HZ,      
                  (unsigned long)AF_FS_ADC_HZ,      
                  (unsigned long)(AF_FS_ADC_HZ / AF_FS_OUT_HZ));  
}  

// TX-профили по таблице пресетов: 1=FLAT .. 7=MAX (от прозрачного к плотному).  
// Каждый профиль — полный снимок тракта: общий блок + per-профиль отличия.  
// Порядок сеттеров важен: freq/Q до presmax/shelf (пересчёт биквадов),  
// split при включённом 2band.  
static void tx_apply_profile(uint8_t idx) {  
    // --- Общее для всех профилей ---  
    af_tx_set_softclip(true);                       // txmode soft  
    af_tx_set_split(900);  
    af_tx_set_compthr(8192);                        // при 2band не активен  
    af_tx_set_gatehyst(7);  
    af_tx_set_presfreq(1900);  af_tx_set_presq(90);  
    af_tx_set_shelffreq(1800); af_tx_set_shelfq(70);  
    af_tx_set_notchf(1000, 2000);  
    af_tx_set_inteltgt(15400); af_tx_set_intelcut(16384); af_tx_set_intelboost(32768);  
  
    switch (idx) {  
    case 1: // FLAT — линейный тракт (только FIR + АРУ-лимитер)  
        af_tx_set_band(300, 2700);  
        af_tx_set_ratio(10);   af_tx_set_clip(32767);  
        af_tx_set_agc(18000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(0);     af_tx_set_gatehold(0);   af_tx_set_gfloor(65536);  
        af_tx_set_ap(false);   af_tx_set_2band(false);  af_tx_set_lr2(false);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(false); af_tx_set_presmax(0);  
        af_tx_set_exc(false);  af_tx_set_excmix(3277);  
        af_tx_set_duck(false); af_tx_set_duckmax(30);  
        af_tx_set_intel(false);  
        af_tx_set_shelf(0);  
        af_tx_set_notch(false);  
        break;  
    case 2: // NATURAL — мягкий повседневный  
        af_tx_set_band(300, 2700);  
        af_tx_set_ratio(30);   af_tx_set_clip(20000);  
        af_tx_set_agc(16000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(250);   af_tx_set_gatehold(180); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(15);  
        af_tx_set_exc(false);  af_tx_set_excmix(3277);  
        af_tx_set_duck(false); af_tx_set_duckmax(30);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(0);  
        af_tx_set_notch(true);  
        break;  
    case 3: // DX — пробив на слабом сигнале  
        af_tx_set_band(300, 2700);  
        af_tx_set_ratio(60);   af_tx_set_clip(12000);  
        af_tx_set_agc(18000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(250);   af_tx_set_gatehold(200); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(30);  
        af_tx_set_exc(true);   af_tx_set_excmix(3277);  
        af_tx_set_duck(true);  af_tx_set_duckmax(30);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(20);  
        af_tx_set_notch(true);  
        break;  
    case 4: // BASS — низкий голос  
        af_tx_set_band(350, 2700);  
        af_tx_set_ratio(40);   af_tx_set_clip(26000);  
        af_tx_set_agc(16000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(250);   af_tx_set_gatehold(180); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(20);  
        af_tx_set_exc(false);  af_tx_set_excmix(3277);  
        af_tx_set_duck(true);  af_tx_set_duckmax(40);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(10);  
        af_tx_set_notch(true);  
        break;  
    case 5: // WHISPER — тихая речь  
        af_tx_set_band(300, 2700);  
        af_tx_set_ratio(50);   af_tx_set_clip(16000);  
        af_tx_set_agc(20000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(150);   af_tx_set_gatehold(250); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(30);  
        af_tx_set_exc(true);   af_tx_set_excmix(1638);  
        af_tx_set_duck(false); af_tx_set_duckmax(30);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(0);  
        af_tx_set_notch(true);  
        break;  
    case 6: // LOUD — громкий голос  
        af_tx_set_band(300, 2700);  
        af_tx_set_ratio(40);   af_tx_set_clip(24000);  
        af_tx_set_agc(14000);  af_tx_set_agcmax(200);  
        af_tx_set_gate(350);   af_tx_set_gatehold(150); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(8192); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(15);  
        af_tx_set_exc(false);  af_tx_set_excmix(3277);  
        af_tx_set_duck(true);  af_tx_set_duckmax(30);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(0);  
        af_tx_set_notch(true);  
        break;  
    case 7: // MAX — максимум плотности  
    default:  
        af_tx_set_band(300, 2400);  
        af_tx_set_ratio(80);   af_tx_set_clip(6144);  
        af_tx_set_agc(18000);  af_tx_set_agcmax(300);  
        af_tx_set_gate(250);   af_tx_set_gatehold(180); af_tx_set_gfloor(4120);  
        af_tx_set_ap(true);    af_tx_set_2band(true);   af_tx_set_lr2(true);  
        af_tx_set_compthrlo(6144); af_tx_set_compthrhi(6144);  
        af_tx_set_pres(true);  af_tx_set_presmax(30);  
        af_tx_set_exc(true);   af_tx_set_excmix(3277);  
        af_tx_set_duck(true);  af_tx_set_duckmax(30);  
        af_tx_set_intel(true);  
        af_tx_set_shelf(40);  
        af_tx_set_notch(true);  
        break;  
    }  
}  

// Заводские дефолты банка: RX — из af_config.h, TX — рабочая точка    
// tx_apply_profile. Per-банк отличия задаются в banks clr.    
static void bank_fill_defaults(af_bank_t* b) {    
    // RX    
    b->lo_hz = 300;  b->hi_hz = 3000;    
    b->cw_on = 0;  b->cw_f0 = 700;  b->cw_bw = 150;  b->cw_noise = 400;    
    b->agc_on = 1;  b->gain_pct = 256;  b->voice_vol = 300;    
    b->agc_ref = AGC_REF;    
    b->agc_max_x100 = (uint16_t)(AGC_MAX_GAIN * 100.0f + 0.5f);    
    b->agc_min_x100 = (uint16_t)(AGC_MIN_GAIN * 100.0f + 0.5f);    
    b->lim_ceil = LIMIT_CEIL;    
    b->nr_depth = NR_DEPTH_Q15;  b->nr_makeup = NR_MAKEUP_Q15;    
    b->nt_mu = NOTCH_MU_Q;  b->nr_mu = NR_MU_Q;    
    // TX — рабочая точка (как базис tx_apply_profile)    
    b->tx_lo_hz = 300;  b->tx_hi_hz = 2700;    
    b->tx_ratio_x10 = 40;  b->tx_clip = 26000;    
    b->tx_agc_target = 18000;  b->tx_agc_max_x10 = AF_TX_AGC_MAX_X10;    
    b->tx_gate_thr = AF_TX_GATE_THR;    
    b->tx_gate_hold_ms = AF_TX_GATE_HOLD_MS;    
    b->tx_gate_hyst_x10 = AF_TX_GATE_HYST_X10;    
    b->tx_gfloor = AF_TX_GATE_FLOOR_Q16;    
    b->tx_ap_on = 1;  b->tx_softclip = 1;  b->tx_2band_on = 1;    
    b->tx_split_hz = AF_TX_SPLIT_HZ;  b->tx_lr2_on = 1;    
    b->tx_comp_thr = 8192;    
    b->tx_comp_thr_lo = AF_TX_COMP_THR_LO;    
    b->tx_comp_thr_hi = AF_TX_COMP_THR_HI;    
    b->tx_duck_on = 1;  b->tx_duckmax = 30;    
    b->tx_intel_on = 1;    
    b->tx_intel_tgt = AF_TX_INTEL_TGT_Q15;    
    b->tx_intel_cut = AF_TX_INTEL_CUT_Q15;    
    b->tx_intel_boost = AF_TX_INTEL_BOOST_Q15;    
    b->tx_pres_on = 1;  b->tx_presmax = 15;    
    b->tx_pres_freq = AF_TX_PRES_FREQ_HZ;  b->tx_pres_q = AF_TX_PRES_Q_X100;    
    b->tx_shelf_db = 0;    
    b->tx_shelf_freq = AF_TX_SHELF_FREQ_HZ;  b->tx_shelf_q = AF_TX_SHELF_Q_X100;    
    b->tx_exc_on = 1;  b->tx_excmix = 3277;    
    b->tx_notch_on = 1;    
    b->tx_notch1_hz = AF_TX_NOTCH1_HZ;  b->tx_notch2_hz = AF_TX_NOTCH2_HZ;    
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
        if (rc == 3) {    
            Serial.println("# nt: rejected (CW on — выключите CW: cw off)");    
            return;    
        }    
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
            Serial.println(af_dsp_cw_on()    
                ? "# nr: rejected (CW on — выключите CW: cw off)"    
                : "# nr: rejected (NT on — приоритет NT)");        
        }  
    }
    else if (strncmp(cmd, "rxnotch", 7) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_dsp_set_rxnotch(true);  Serial.println("# OK rxnotch=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_dsp_set_rxnotch(false); Serial.println("# OK rxnotch=off"); }  
        else Serial.println("# ERR: rxnotch on|off");  
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
            // Банк = полный снимок RX+TX: сначала дефолты из af_config.h  
            // и рабочая точка tx_apply_profile, затем per-банк отличия  
            // (полоса ПФ, CW, АРУ/gain — как раньше).  
            af_bank_t preset[AF_STORE_BANKS];  
            for (uint8_t i = 0; i < AF_STORE_BANKS; i++) {  
                bank_fill_defaults(&preset[i]);  
            }  
            // Per-банк отличия — только RX-поля, TX у всех одинаковый.  
            // 1 (GP2): SSB нормальная  
            preset[0].lo_hz = 100;  preset[0].hi_hz = 3100;  
            // 2 (GP3): SSB узкая  
            preset[1].lo_hz = 300;  preset[1].hi_hz = 2200;  
            // 3: резерв — широкая  
            preset[2].lo_hz = 100;  preset[2].hi_hz = 4000;  
            // 4 (GP4): CW 700 Гц  
            preset[3].lo_hz = 450;  preset[3].hi_hz = 950;  
            preset[3].cw_on = 1;  
            // 5: резерв — очень узкая  
            preset[4].lo_hz = 300;  preset[4].hi_hz = 900;  
            // 6: резерв — AM, АРУ off (×1.5)  
            preset[5].lo_hz = 300;  preset[5].hi_hz = 3000;  
            preset[5].agc_on = 0;   preset[5].gain_pct = 384;  
            // 7: резерв  
            preset[6].lo_hz = 200;  preset[6].hi_hz = 3500;  
  
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
                Serial.printf("#   rx: agcref=%u agcmax=%u.%02u agcmin=%u.%02u lim=%u nrd=%u nrm=%lu ntmu=%u nrmu=%u\n",  
                              (unsigned)b.agc_ref,  
                              (unsigned)(b.agc_max_x100 / 100), (unsigned)(b.agc_max_x100 % 100),  
                              (unsigned)(b.agc_min_x100 / 100), (unsigned)(b.agc_min_x100 % 100),  
                              (unsigned)b.lim_ceil, (unsigned)b.nr_depth,  
                              (unsigned long)b.nr_makeup,  
                              (unsigned)b.nt_mu, (unsigned)b.nr_mu);  
                Serial.printf("#   tx: band=%u..%u comp=%u clip=%u agc=%u/%u gate=%u h=%u y=%u gf=%u\n",  
                              (unsigned)b.tx_lo_hz, (unsigned)b.tx_hi_hz,  
                              (unsigned)b.tx_ratio_x10, (unsigned)b.tx_clip,  
                              (unsigned)b.tx_agc_target, (unsigned)b.tx_agc_max_x10,  
                              (unsigned)b.tx_gate_thr, (unsigned)b.tx_gate_hold_ms,  
                              (unsigned)b.tx_gate_hyst_x10, (unsigned)b.tx_gfloor);  
                Serial.printf("#   tx: ap=%u soft=%u 2b=%u split=%u lr2=%u cthr=%u/%u/%u duck=%u/%u\n",  
                              (unsigned)b.tx_ap_on, (unsigned)b.tx_softclip,  
                              (unsigned)b.tx_2band_on, (unsigned)b.tx_split_hz,  
                              (unsigned)b.tx_lr2_on,  
                              (unsigned)b.tx_comp_thr, (unsigned)b.tx_comp_thr_lo,  
                              (unsigned)b.tx_comp_thr_hi,  
                              (unsigned)b.tx_duck_on, (unsigned)b.tx_duckmax);  
                Serial.printf("#   tx: intel=%u %u/%u/%u pres=%u/%u f%u q%u shelf=%u f%u q%u exc=%u/%u notch=%u %u/%u\n",  
                              (unsigned)b.tx_intel_on,  
                              (unsigned)b.tx_intel_tgt, (unsigned)b.tx_intel_cut,  
                              (unsigned)b.tx_intel_boost,  
                              (unsigned)b.tx_pres_on, (unsigned)b.tx_presmax,  
                              (unsigned)b.tx_pres_freq, (unsigned)b.tx_pres_q,  
                              (unsigned)b.tx_shelf_db, (unsigned)b.tx_shelf_freq,  
                              (unsigned)b.tx_shelf_q,  
                              (unsigned)b.tx_exc_on, (unsigned)b.tx_excmix,  
                              (unsigned)b.tx_notch_on,  
                              (unsigned)b.tx_notch1_hz, (unsigned)b.tx_notch2_hz);  
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

    else if (strcmp(cmd, "txsetdef") == 0) {      // сброс TX к дефолтам af_config.h    
        af_tx_set_band(AF_TX_LO_HZ_DEFAULT, AF_TX_HI_HZ_DEFAULT);      
        af_tx_set_ratio(AF_TX_RATIO_X10_DEFAULT);      
        af_tx_set_clip(AF_TX_CLIP_DEFAULT);      
        af_tx_set_agc(AF_TX_AGC_TARGET);      
        af_tx_set_gate(AF_TX_GATE_THR);      
        af_tx_set_gfloor(AF_TX_GATE_FLOOR_Q16);      
        af_tx_set_ap(AF_TX_AP_ON_DEFAULT != 0);        
        af_tx_set_softclip(AF_TX_SOFTCLIP_ON_DEFAULT != 0);        
        af_tx_set_2band(AF_TX_2BAND_ON_DEFAULT != 0);        
        af_tx_set_pres(AF_TX_PRES_ON_DEFAULT != 0);      
        af_tx_set_presmax(AF_TX_PRES_MAX_DB_X10);      
        af_tx_set_lr2(AF_TX_LR2_ON_DEFAULT != 0);      
        af_tx_set_duck(AF_TX_DUCK_ON_DEFAULT != 0);      
        af_tx_set_duckmax(AF_TX_DUCK_MAX_DB_X10);      
        af_tx_set_exc(AF_TX_EXC_ON_DEFAULT != 0);      
        af_tx_set_excmix(AF_TX_EXC_MIX_Q15);      
        af_tx_set_intel(AF_TX_INTEL_ON_DEFAULT != 0);    
        af_tx_set_shelf(AF_TX_SHELF_DB_X10_DEFAULT);    
        af_tx_set_compthr(8192);      
        af_tx_set_compthrlo(AF_TX_COMP_THR_LO);  af_tx_set_compthrhi(AF_TX_COMP_THR_HI);      
        af_tx_set_split(AF_TX_SPLIT_HZ);      
        af_tx_set_gatehold(AF_TX_GATE_HOLD_MS);  af_tx_set_gatehyst(AF_TX_GATE_HYST_X10);      
        af_tx_set_agcmax(AF_TX_AGC_MAX_X10);      
        af_tx_set_inteltgt(AF_TX_INTEL_TGT_Q15);      
        af_tx_set_intelcut(AF_TX_INTEL_CUT_Q15);      
        af_tx_set_intelboost(AF_TX_INTEL_BOOST_Q15);      
        af_tx_set_presfreq(AF_TX_PRES_FREQ_HZ);  af_tx_set_presq(AF_TX_PRES_Q_X100);      
        af_tx_set_shelffreq(AF_TX_SHELF_FREQ_HZ); af_tx_set_shelfq(AF_TX_SHELF_Q_X100);      
        af_tx_set_notchf(AF_TX_NOTCH1_HZ, AF_TX_NOTCH2_HZ);      
        Serial.printf("# OK txsetdef  band=%lu..%lu comp=%lu clip=%lu agc=%lu gate=%lu gfloor=%lu ap=%s mode=%s 2band=%s\n",      
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

    else if (strncmp(cmd, "txprof", 6) == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txprof <1..7|flat|natural|dx|bass|whisper|loud|max>"); return; }    
        uint8_t idx = 0;    
        if      (strcmp(v, "flat")    == 0) idx = 1;    
        else if (strcmp(v, "natural") == 0) idx = 2;    
        else if (strcmp(v, "dx")      == 0) idx = 3;    
        else if (strcmp(v, "bass")    == 0) idx = 4;    
        else if (strcmp(v, "whisper") == 0) idx = 5;    
        else if (strcmp(v, "loud")    == 0) idx = 6;    
        else if (strcmp(v, "max")     == 0) idx = 7;    
        else if (strcmp(v, "work")    == 0) { Serial.println("# work -> txsetdef"); return; }      
        else {    
            idx = (uint8_t)strtoul(v, NULL, 10);    
            if (idx < 1 || idx > 7) { Serial.println("# ERR: txprof <1..7>"); return; }    
        }    
        tx_apply_profile(idx);  
        Serial.printf("# OK txprof=%u\n", idx);  
    }  
    
    else if (strcmp(cmd, "txnotchf") == 0) {    
        char* f1 = strtok(NULL, " \t");    
        char* f2 = strtok(NULL, " \t");    
        if (f1 == NULL || f2 == NULL) { Serial.println("# ERR: txnotchf <f1> <f2>"); return; }    
        af_tx_set_notchf((uint32_t)strtoul(f1, NULL, 10),    
                         (uint32_t)strtoul(f2, NULL, 10));    
        Serial.printf("# OK txnotchf=%lu/%lu Hz\n",    
                      (unsigned long)af_tx_notch1(), (unsigned long)af_tx_notch2());    
    }

    else if (strncmp(cmd, "txnotch", 7) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_tx_set_notch(true);  Serial.println("# OK txnotch=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_tx_set_notch(false); Serial.println("# OK txnotch=off"); }  
        else Serial.println("# ERR: txnotch on|off");  
    }  
    else if (strcmp(cmd, "txpresfreq") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txpresfreq <500..6000>"); return; }    
        af_tx_set_presfreq((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txpresfreq=%lu Hz\n", (unsigned long)af_tx_presfreq());    
    }    
    else if (strcmp(cmd, "txpresq") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txpresq <10..500>"); return; }    
        af_tx_set_presq((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txpresq=%lu\n", (unsigned long)af_tx_presq());    
    }    
    else if (strncmp(cmd, "txpresmax", 9) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: txpresmax <дБx10>"); return; }  
        af_tx_set_presmax((uint32_t)strtoul(v, NULL, 10));  
        Serial.printf("# OK txpresmax=%lu (дБx10)\n", (unsigned long)af_tx_presmax());  
    }  
    else if (strncmp(cmd, "txpres", 6) == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v && strcmp(v, "on") == 0)       { af_tx_set_pres(true);  Serial.println("# OK txpres=on"); }  
        else if (v && strcmp(v, "off") == 0) { af_tx_set_pres(false); Serial.println("# OK txpres=off"); }  
        else Serial.println("# ERR: txpres on|off");  
    }  

    else if (strncmp(cmd, "txlr2", 5) == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txlr2 on|off"); return; }    
        af_tx_set_lr2(strcmp(v, "on") == 0);    
        Serial.printf("# OK txlr2=%s\n", af_tx_lr2() ? "on" : "off");    
    }    

    else if (strncmp(cmd, "txduckmax", 9) == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txduckmax <дБx10>"); return; }    
        af_tx_set_duckmax((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txduckmax=%lu\n", (unsigned long)af_tx_duckmax());    
    }    
    else if (strncmp(cmd, "txduck", 6) == 0) {      
        char* v = strtok(NULL, " \t");      
        if (v == NULL) { Serial.println("# ERR: txduck on|off"); return; }      
        bool on = (strncmp(v, "on", 2) == 0);      
        af_tx_set_duck(on);      
        Serial.printf("# OK txduck=%s\n", on ? "on" : "off");      
    }      
  
    else if (strncmp(cmd, "txexcmix", 8) == 0) {      
        char* v = strtok(NULL, " \t");      
        if (v == NULL) { Serial.println("# ERR: txexcmix <q15>"); return; }      
        af_tx_set_excmix((uint32_t)strtoul(v, NULL, 10));      
        Serial.printf("# OK txexcmix=%lu\n", (unsigned long)af_tx_excmix());      
    }      
    else if (strncmp(cmd, "txexc", 5) == 0) {      
        char* v = strtok(NULL, " \t");      
        if (v == NULL) { Serial.println("# ERR: txexc on|off"); return; }      
        af_tx_set_exc(strncmp(v, "on", 2) == 0);      
        Serial.printf("# OK txexc=%s\n", af_tx_exc() ? "on" : "off");      
    }      
    else if (strcmp(cmd, "txinteltgt") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txinteltgt <q15>"); return; }    
        af_tx_set_inteltgt((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txinteltgt=%ld\n", (long)af_tx_inteltgt());    
    }    
    else if (strcmp(cmd, "txintelcut") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txintelcut <q15>"); return; }    
        af_tx_set_intelcut((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txintelcut=%ld\n", (long)af_tx_intelcut());    
    }    
    else if (strcmp(cmd, "txintelboost") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txintelboost <q15>"); return; }    
        af_tx_set_intelboost((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txintelboost=%ld\n", (long)af_tx_intelboost());    
    }    
    else if (strncmp(cmd, "txintel", 7) == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txintel on|off"); return; }    
        af_tx_set_intel(strncmp(v, "on", 2) == 0);    
        Serial.printf("# OK txintel=%s\n", af_tx_intel() ? "on" : "off");    
    }    
    else if (strcmp(cmd, "txshelffreq") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txshelffreq <500..6000>"); return; }    
        af_tx_set_shelffreq((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txshelffreq=%lu Hz\n", (unsigned long)af_tx_shelffreq());    
    }    
    else if (strcmp(cmd, "txshelfq") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txshelfq <10..500>"); return; }    
        af_tx_set_shelfq((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txshelfq=%lu\n", (unsigned long)af_tx_shelfq());    
    }    
    else if (strncmp(cmd, "txshelf", 7) == 0) {  
        const char* v = strtok(NULL, " ");  
        if (!v) { Serial.println("# ERR: txshelf <дБx10>"); return; }  
        af_tx_set_shelf((uint32_t)atoi(v));  
        Serial.printf("# OK txshelf=%lu\n", (unsigned long)af_tx_shelf());  
    }  
    // --- RX: АРУ/лимитер/NR (этап 1) ---    
    else if (strcmp(cmd, "agcref") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: agcref <1000..32767>"); return; }    
        af_dsp_set_agcref((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK agcref=%ld\n", (long)af_dsp_agcref());    
    }    
    else if (strcmp(cmd, "agcmax") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: agcmax <1..1000>"); return; }    
        af_dsp_set_agcmax(strtof(v, NULL));    
        Serial.printf("# OK agcmax=%ld\n", (long)(af_dsp_agcmax() * 100.0f + 0.5f));    
    }    
    else if (strcmp(cmd, "agcmin") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: agcmin <0.01..1>"); return; }    
        af_dsp_set_agcmin(strtof(v, NULL));    
        Serial.printf("# OK agcmin=%ld\n", (long)(af_dsp_agcmin() * 100.0f + 0.5f));    
    }    
    else if (strcmp(cmd, "limceil") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: limceil <1000..32767>"); return; }    
        af_dsp_set_limceil((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK limceil=%ld\n", (long)af_dsp_limceil());    
    }    
    else if (strcmp(cmd, "nrdepth") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: nrdepth <0..65536>"); return; }    
        af_dsp_set_nrdepth((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK nrdepth=%ld\n", (long)af_dsp_nrdepth());    
    }    
    else if (strcmp(cmd, "nrmakeup") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: nrmakeup <0..131072>"); return; }    
        af_dsp_set_nrmakeup((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK nrmakeup=%ld\n", (long)af_dsp_nrmakeup());    
    }    
    else if (strcmp(cmd, "ntmu") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: ntmu <0..32768>"); return; }    
        af_dsp_set_ntmu((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK ntmu=%ld\n", (long)af_dsp_ntmu());    
    }    
    else if (strcmp(cmd, "nrmu") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: nrmu <0..32768>"); return; }    
        af_dsp_set_nrmu((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK nrmu=%ld\n", (long)af_dsp_nrmu());    
    }    
  
    // --- TX: пороги/частоты/тайминги (этап 2) ---    
    else if (strcmp(cmd, "txcompthr") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txcompthr <256..32767>"); return; }    
        af_tx_set_compthr((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txcompthr=%ld\n", (long)af_tx_compthr());    
    }    
    else if (strcmp(cmd, "txcompthrlo") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txcompthrlo <256..32767>"); return; }    
        af_tx_set_compthrlo((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txcompthrlo=%ld\n", (long)af_tx_compthrlo());    
    }    
    else if (strcmp(cmd, "txcompthrhi") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txcompthrhi <256..32767>"); return; }    
        af_tx_set_compthrhi((int32_t)strtol(v, NULL, 10));    
        Serial.printf("# OK txcompthrhi=%ld\n", (long)af_tx_compthrhi());    
    }    
    else if (strcmp(cmd, "txsplit") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txsplit <300..3000>"); return; }    
        af_tx_set_split((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txsplit=%lu Hz\n", (unsigned long)af_tx_split());    
    }    
    else if (strcmp(cmd, "txgatehold") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txgatehold <0..2000>"); return; }    
        af_tx_set_gatehold((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txgatehold=%lu ms\n", (unsigned long)af_tx_gatehold());    
    }    
    else if (strcmp(cmd, "txgatehyst") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txgatehyst <1..10>"); return; }    
        af_tx_set_gatehyst((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txgatehyst=%lu\n", (unsigned long)af_tx_gatehyst());    
    }    
    else if (strcmp(cmd, "txagcmax") == 0) {    
        char* v = strtok(NULL, " \t");    
        if (v == NULL) { Serial.println("# ERR: txagcmax <10..1000>"); return; }    
        af_tx_set_agcmax((uint32_t)strtoul(v, NULL, 10));    
        Serial.printf("# OK txagcmax=%lu\n", (unsigned long)af_tx_agcmax());    
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