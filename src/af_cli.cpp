// src/af_cli.cpp  
// ============================================================================  
// af_cli.cpp — Serial CLI (Core 0)  
//  
// Команды (по одной на строку, завершение \n). Полный список — help.  
//   RX:  band nt nr rxnotch cw agc gain bypass  
//        agcref agcmax agcmin limceil nrdepth nrmakeup ntmu nrmu  
//        "rx pres" "rx presmax" "rx pres f" "rx pres q" (метки AF_PARAMS)  
//   TX:  tx txband txap txagc txagcmax txgate txgatehold txgatehyst txgfloor  
//        tx2band txsplit txlr2 txcomp txcompthr txcompthrlo txcompthrhi  
//        txduck txduckmax txintel txinteltgt txintelcut txintelboost  
//        txpres txpresmax txpresfreq txpresq txshelf txshelffreq txshelfq  
//        txexc txexcmix txmode txclip txnotch txnotchf txsetdef txprof
//   SYS: stats dump fmap ftest save load banks pwm led voice ramp rec play stop recs help  
// ============================================================================  
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>                     // log10 — inlvl в dBFS 
#include <stddef.h>                   // offsetof — таблица параметров AF_PARAMS
#include "hardware/clocks.h"          // clock_get_hz — фактическая clk_sys  
#include "hardware/structs/ssi.h"     // ssi_hw->baudr — делитель SCK флеша  
#include "hardware/flash.h"           // FLASH_SECTOR_SIZE, PICO_FLASH_SIZE_BYTES — fmap
#include "hardware/sync.h"            // save_and_disable_interrupts — ftest

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
#include "af_rec.h"       // цифровой магнитофон: RAM-буфер ~7.8 с (децимация ×2)
#include "af_fm.h"        // af_fm_request_sync — перерендер SET.TXT 
  
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
    Serial.println("#   rx pres on|off    — dynamic presence EQ RX (подъём ВЧ по SNR)");  
    Serial.println("#   rx presmax <дБx10> — потолок подъёма (15=+1.5дБ, 60=+6дБ)");  
    Serial.println("#   rx pres f <1000..4000> — центр presence-пика, Гц");  
    Serial.println("#   rx pres q <30..300> — добротность x100 (100 = 1.00)");  
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
    Serial.println("#   stats             — таблица параметров RX/TX/SYS");  
    Serial.println("#   dump [N|off]      — инженерный дамп счётчиков: однократно / каждые N сек / стоп");    
    Serial.println("#   fmap              — RLE-карта flash: регионы, граница прошивки, банки");
    Serial.println("#   ftest [m] [N]     — FLASH-TEST @0x100000: m=0 окно, 1 +park, 2 +clk, 3 всё; N итераций");
    Serial.println("#   save <1..7>       — сохранить настройки в ячейку");    
    Serial.println("#   load <0..7>       — применить ячейку (0 = ручной режим)");    
    Serial.println("#   banks             — показать все ячейки");  
    Serial.println("#   banks clr         — стандартные пресеты во все банки + очистка голосовых слотов");      
    Serial.println("#   pwm on|off        — PWM-выход (off = несущая заглушена)");  
    Serial.println("#   led on|off        — heartbeat-индикация (WS2812B + GP25)");  
    Serial.println("#   voice [N|vol <0..400>] — голосовые сообщения");  
    Serial.println("#   ramp on|off       — пила-счётчик вместо DSP (тест потока)");    
    Serial.println("#   rec               — запись в RAM (~7.8 с, источник = активный тракт)");      
    Serial.println("#   play              — воспроизведение (запись — в активный тракт; пусто — voice_14 в TX+PTT)");        
    Serial.println("#   stop              — стоп записи/воспроизведения");    
    Serial.println("#   recs              — состояние слота");    
    Serial.println("#   help / h / ?      — эта справка");      
    Serial.println("# === GPIO ===");    
    Serial.println("# GP2,3,4=банк (двоичный код, 000=ручной), GP5=NR, GP6=NT, GP7=bypass, GP8=PTT, GP9=rec, GP10=play (LOW=on)");
}
  
// ---------------------------------------------------------------------------  
// Диагностика  
// --------------------------------------------------------------------------- 

// Периодический дамп: dump <N> — каждые N сек, dump 0/off — стоп.  
static uint32_t dump_period_ms = 0;  
static uint32_t dump_next_ms   = 0;
static bool cli_rec_tx = false;        // CLI rec включил TX — stop вернёт RX      
static bool cli_play_pending = false;  // CLI play запущен — OK после окончания      
static bool cli_play_ptt = false;      // CLI play voice_14 держит TX+PTT — отпустить по концу      

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
    Serial.printf("#   rxpres  = %s pmax=%ld f=%lu q=%lu   env=%ld nf=%ld k=%ld\n",  
                  af_dsp_rxpres_on() ? "on" : "off",  
                  (long)af_dsp_rxpresmax(),  
                  (unsigned long)af_dsp_rxpresf(),  
                  (unsigned long)af_dsp_rxpresq(),  
                  (long)af_dsp_rxpres_env(),  
                  (long)af_dsp_rxpres_nf(),  
                  (long)af_dsp_rxpres_k());  
    {   // S-meter: уровень и пик после FIR в линейных единицах и dBFS  
        int32_t lvl = af_dsp_rx_in_lvl();  
        int32_t pk  = af_dsp_rx_in_peak();   // чтение сбрасывает пик  
        Serial.printf("#   inlvl   = %ld (%.1f dBFS)   inpk=%ld (%.1f dBFS)\n",  
                      (long)lvl, 20.0 * log10((lvl + 1) / 32768.0),  
                      (long)pk,  20.0 * log10((pk + 1) / 32768.0));  
    }  

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
    {   // Фактические параметры ADC: то, что раньше было только в boot-логе [CAP].  
        const uint32_t cad = af_capture_clk_adc_hz();  
        const float    dvr = af_capture_adc_div();  
        Serial.printf("#   adc     : clk_adc=%lu MHz  div=%lu.%02u  fs=%lu Hz  ch=%u\n",  
                      (unsigned long)(cad / 1000000u),  
                      (unsigned long)dvr,  
                      (unsigned)((dvr - (float)(unsigned long)dvr) * 100.0f),  
                      (unsigned long)(dvr > 0.0f  
                          ? (uint32_t)((float)cad / (dvr + 1.0f))  
                          : 0u),  
                      (unsigned)af_capture_channel());  
    }  
    Serial.printf("#   clk_sys = %lu MHz  flash_sck=%lu.%lu MHz (BAUDR=%u)\n",    
                  (unsigned long)(clock_get_hz(clk_sys) / 1000000u),  
                  (unsigned long)(clock_get_hz(clk_sys) / ssi_hw->baudr / 1000000u),  
                  (unsigned long)((clock_get_hz(clk_sys) / ssi_hw->baudr % 1000000u) / 100000u),  
                  (unsigned)ssi_hw->baudr);  
}  

// Инженерный дамп: дельты счётчиков с прошлого вызова + ur-логи.  
// Бывшая периодическая сводка из loop() — теперь по запросу.  
static void cli_dump(void) {  
    static uint32_t p_pub = 0, p_dU = 0, p_dP = 0, p_urU = 0, p_urP = 0;  
    static uint32_t p_usb_ur = 0, p_pwm_ur = 0, p_cap = 0, p_isr = 0;  
  
    const uint32_t pub   = af_dsp_out_published_usb();  
    const uint32_t dU    = af_dsp_out_dropped_usb();  
    const uint32_t dP    = af_dsp_out_dropped_pwm();  
    const uint32_t urU   = af_dsp_out_underruns_usb();  
    const uint32_t urP   = af_dsp_out_underruns_pwm();  
    const uint32_t uur   = af_usb_underruns();  
    const uint32_t pur   = af_pwm_underruns();  
    const uint32_t cap   = af_capture_dropped();  
    const uint32_t isr   = af_cap_isr_count;  
  
    Serial.printf(  
        "# L0=%lu.%lu%% L1=%lu.%lu%% blk=%lu max=%luus inlvl=%ld inpk=%ld |"  
        " cap: isr=%lu d=%lu scr=%lu qf=%lu bs=%lu fcs=%lu both=%lu qmax=%lu |"    
        " out: dU=%lu dP=%lu urU=%lu urP=%lu fU=%u fP=%u |"    
        " usb: ur=%lu short=%lu rs=%ldmHz rpr=%lu gap=%luus ff=%lu ws=%lu av=%lu |"    
        " pwm: ur=%lu rst=%lu dmax=%u | clip=%lu\n",    
        (unsigned long)(af_core0_load_permille() / 10u),    
        (unsigned long)(af_core0_load_permille() % 10u),    
        (unsigned long)(af_dsp_load_permille() / 10u),    
        (unsigned long)(af_dsp_load_permille() % 10u),    
        (unsigned long)(pub - p_pub),    
        (unsigned long)af_dsp_max_block_us(),    
        (long)af_dsp_rx_in_lvl(),  
        (long)af_dsp_rx_in_peak(),  
        (unsigned long)(isr - p_isr), (unsigned long)(cap - p_cap),  
        (unsigned long)af_drop_scratch, (unsigned long)af_drop_queue_full,  
        (unsigned long)af_drop_busy, (unsigned long)af_adc_fcs_over,  
        (unsigned long)af_irq_both_pending, (unsigned long)af_q_fill_max,  
        (unsigned long)(dU - p_dU), (unsigned long)(dP - p_dP),  
        (unsigned long)(urU - p_urU), (unsigned long)(urP - p_urP),  
        (unsigned)af_dsp_out_fill_max_usb(), (unsigned)af_dsp_out_fill_max_pwm(),  
        (unsigned long)(uur - p_usb_ur), (unsigned long)af_usb_frames_short(),  
        (long)af_usb_rs_adj_mhz(), (unsigned long)af_usb_reprimes(),  
        (unsigned long)af_usb_poll_gap_max(), (unsigned long)af_usb_fifo_full(),  
        (unsigned long)af_usb_write_short(), (unsigned long)af_usb_avail_max(),  
        (unsigned long)(pur - p_pwm_ur), (unsigned long)af_pwm_restarts(),  
        (unsigned)af_pwm_duty_fill_max(),  
        (unsigned long)af_dsp_clip_count());  
  
    uint32_t ut[8]; uint16_t un[8];  
    af_usb_ur_log(ut, un);  
    Serial.printf("# usb_ur_t:");  
    for (int i = 0; i < 8; i++) Serial.printf(" %lu", (unsigned long)ut[i]);  
    Serial.printf(" | n:");  
    for (int i = 0; i < 8; i++) Serial.printf(" %u", (unsigned)un[i]);  
    Serial.println();  
  
    uint32_t pt[8];    
    af_pwm_ur_log(pt);    
    Serial.printf("# pwm_ur_t:");    
    for (int i = 0; i < 8; i++) Serial.printf(" %lu", (unsigned long)pt[i]);    
    Serial.println();    
    
    // ---- Мини-проверка: заключение о здоровье тракта ----    
    // Проверки на дельтах за период (не накопленное с boot) + пики.    
    // Порядок — от источника к потребителю: cap -> core1 -> out -> клиппинг.    
    {    
        const uint32_t d_cap  = cap - p_cap;      // потери на захвате за период    
        const uint32_t d_dU   = dU - p_dU;        // дропы USB-очереди    
        const uint32_t d_dP   = dP - p_dP;        // дропы PWM-очереди    
        const uint32_t d_urU  = urU - p_urU;      // underruns USB (публикация)    
        const uint32_t d_urP  = urP - p_urP;      // underruns PWM (публикация)    
        const uint32_t max_us = af_dsp_max_block_us();    
        const uint32_t l1     = af_dsp_load_permille();    
        bool ok = true;    
    
        if (d_cap != 0 || af_drop_queue_full != 0 || af_adc_fcs_over != 0) {    
            ok = false;    
            Serial.printf("# WARN cap: потери захвата (dropped+%lu q_full=%lu fcs_over=%lu)\n",    
                          (unsigned long)d_cap,    
                          (unsigned long)af_drop_queue_full,    
                          (unsigned long)af_adc_fcs_over);    
        }    
        if (max_us > 4096) {    
            ok = false;    
            Serial.printf("# WARN core1: max_block_us=%lu > окна блока (~4096) — перегрузка DSP\n",    
                          (unsigned long)max_us);    
        } else if (l1 > 950) {    
            ok = false;    
            Serial.printf("# WARN core1: load=%lu.%lu%% — на пределе\n",    
                          (unsigned long)(l1 / 10u), (unsigned long)(l1 % 10u));    
        }    
        if (af_q_fill_max >= 3) {    
            ok = false;    
            Serial.printf("# WARN cap: очередь заполнялась до %lu — Core 1 не успевает забирать\n",    
                          (unsigned long)af_q_fill_max);    
        }    
        if (d_dU != 0 || d_dP != 0) {    
            ok = false;    
            Serial.printf("# WARN out: дропы блоков usb=%lu pwm=%lu\n",    
                          (unsigned long)d_dU, (unsigned long)d_dP);    
        }    
        // urU — паддинг кадров при пустой out_usb: хост (ОС) читает эндпоинт    
        // всегда, дефицит источника ~1% — штатно и неслышно (af_usb.cpp).    
        // WARN только при реальных потерях: дропы публикации (d_dU выше)    
        // или перезаправки очереди (reprimes — дно + уход на заправку).    
        // USB-здоровье оцениваем только когда хост реально читает поток.  
        // Признак чтения — не blocks_rx (он растёт при записи в буфер  
        // AudioTools всегда), а rs_adj: при нечитающем хосте ресемплер  
        // стоит на -48000 mHz (буфер полон, av=2048). При чтении rs  
        // колеблется около нуля.  
        if (af_usb_mounted() && af_usb_rs_adj_mhz() > -40000) {  
            if (d_urU != 0) {  
                Serial.printf("# INFO usb: паддинг +%lu фреймов (штатно при читающем хосте)\n",  
                              (unsigned long)d_urU);  
            }  
            const uint32_t rpr_now = af_usb_reprimes();  
            static uint32_t p_rpr = 0;  
            if (rpr_now - p_rpr != 0) {  
                ok = false;  
                Serial.printf("# WARN usb: reprimes +%lu — очередь уходила на дно\n",  
                              (unsigned long)(rpr_now - p_rpr));  
            }  
            p_rpr = rpr_now;  
        } else {  
            Serial.println("# INFO usb: хост не читает поток (av=2048, rs=-48000mHz)");  
        }  
        // urP — «запрос при пустой очереди»: PWM-ISR опрашивает очередь      
        // чаще, чем Core 1 публикует блоки, счётчик растёт всегда.      
        // Реальный критерий голодания — af_pwm_underruns (пропуск выхода).      
        const uint32_t pwm_ur_now = af_pwm_underruns();      
        static uint32_t p_pwm_ur2 = 0;      
        if (af_pwm_is_enabled() && (pwm_ur_now - p_pwm_ur2) != 0) {      
            ok = false;      
            Serial.printf("# WARN pwm: underruns +%lu — выход голодает\n",      
                          (unsigned long)(pwm_ur_now - p_pwm_ur2));      
        }      
        p_pwm_ur2 = pwm_ur_now;      
        if (af_dsp_clip_count() != 0) {    
            ok = false;    
            Serial.printf("# WARN clip: клиппинг выхода, count=%lu\n",    
                          (unsigned long)af_dsp_clip_count());    
        }    
        if (ok) Serial.println("# OK: потерь нет, тракт здоров");    
    }    
    
    p_pub = pub; p_dU = dU; p_dP = dP; p_urU = urU; p_urP = urP;    
    p_usb_ur = uur; p_pwm_ur = pur; p_cap = cap; p_isr = isr;    
}    

// ---------------------------------------------------------------------------  
// fmap — RLE-сканер flash: диапазоны секторов с одинаковым статусом.  
// Только чтение через XIP — erase/program не используется, зависнуть не может.  
// ---------------------------------------------------------------------------  
typedef enum {  
    FM_FW,        // прошивка (данные до AF_REC_FLASH_OFF)  
    FM_FREE,      // пусто (всё 0xFF)  
    FM_REC_EMPTY, // слот магнитофона, пустой  
    FM_REC_DATA,  // слот магнитофона, есть данные (остатки дампа)  
    FM_BANKS,     // последний сектор — банки настроек  
    FM_DATA,      // данные вне известных регионов  
} fm_class_t;  
  
static fm_class_t fmap_classify(uint32_t off, bool empty) {  
    if (off == PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) return FM_BANKS;  
    if (off >= AF_REC_FLASH_OFF &&  
        off <  AF_REC_FLASH_OFF + AF_REC_FLASH_LEN)  
        return empty ? FM_REC_EMPTY : FM_REC_DATA;  
    if (empty) return FM_FREE;  
    return (off < AF_REC_FLASH_OFF) ? FM_FW : FM_DATA;  
}  
  
static const char* fmap_name(fm_class_t c) {  
    switch (c) {  
        case FM_FW:        return "firmware";  
        case FM_FREE:      return "free";  
        case FM_REC_EMPTY: return "rec-slot(empty)";  
        case FM_REC_DATA:  return "rec-slot(DATA)";  
        case FM_BANKS:     return "banks";  
        default:           return "data?";  
    }  
}  
  
static void cli_fmap(void) {  
    const uint32_t FSZ  = PICO_FLASH_SIZE_BYTES;  
    const uint32_t SEC  = FLASH_SECTOR_SIZE;  
    const uint32_t NSEC = FSZ / SEC;  
    const uint8_t* base = (const uint8_t*)XIP_BASE;  
  
    Serial.printf("# fmap: flash=%lu KB sectors=%lu rec_slot=%06lx..%06lx banks=%06lx\n",  
                  (unsigned long)(FSZ / 1024), (unsigned long)NSEC,  
                  (unsigned long)AF_REC_FLASH_OFF,  
                  (unsigned long)(AF_REC_FLASH_OFF + AF_REC_FLASH_LEN - 1),  
                  (unsigned long)(FSZ - SEC));  
  
    fm_class_t run_cls = (fm_class_t)-1;  
    uint32_t   run_start = 0;  
    uint32_t   fw_end = 0;          // последний непустой сектор до rec-слота  
  
    for (uint32_t s = 0; s <= NSEC; s++) {          // <= — флаш хвоста  
        fm_class_t cls = FM_FREE;  
        if (s < NSEC) {  
            uint32_t off = s * SEC;  
            // быстрый тест на пустоту: словами по 4 байта  
            const uint32_t* w = (const uint32_t*)(base + off);  
            bool empty = true;  
            for (uint32_t i = 0; i < SEC / 4; i++) {  
                if (w[i] != 0xFFFFFFFFu) { empty = false; break; }  
            }  
            cls = fmap_classify(off, empty);  
            if (!empty && off < AF_REC_FLASH_OFF) fw_end = off + SEC;  
        }  
        if (s == NSEC || cls != run_cls) {  
            if (run_cls != (fm_class_t)-1) {  
                Serial.printf("# %06lx..%06lx  %4lu KB  %s\n",  
                              (unsigned long)run_start,  
                              (unsigned long)(s * SEC - 1),  
                              (unsigned long)((s * SEC - run_start) / 1024),  
                              fmap_name(run_cls));  
            }  
            run_cls = cls; run_start = s * SEC;  
        }  
        if ((s & 0x3F) == 0) Serial.flush();  
    }  
  
    Serial.printf("# fmap: firmware end ~0x%06lx, free before rec-slot ~%lu KB\n",  
                  (unsigned long)fw_end,  
                  (unsigned long)((AF_REC_FLASH_OFF - fw_end) / 1024));  
  
    // Превью сектора банков: magic + начало первой записи.  
    const uint8_t* b = base + (FSZ - SEC);  
    Serial.printf("# banks @%06lx: %02x %02x %02x %02x | rec1: %02x %02x %02x %02x\n",  
                  (unsigned long)(FSZ - SEC),  
                  b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);  
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

// ---------------------------------------------------------------------------  
// Единая таблица параметров: метка = команда и строка таблицы banks.  
// off — смещение поля в af_bank_t (для печати колонок cur/1..7).  
// set — сеттер DSP (для команд вида "<метка> <значение>").  
// ---------------------------------------------------------------------------  
typedef enum { PT_U16, PT_U8, PT_I16, PT_ONOFF, PT_X100, PT_GAIN, PT_SEP } ptype_t;    
  
typedef struct {  
    const char* label;                 // "band lo", "tx pres f" — и команда  
    ptype_t     type;  
    uint16_t    off;                   // offsetof(af_bank_t, field)  
    void      (*set)(int32_t v);       // NULL у PT_SEP  
} af_param_t;  
  
// Обёртки для парных/плавающих сеттеров.  
static void p_set_band_lo(int32_t v)  { af_dsp_set_band((uint32_t)v, af_dsp_band_hi_hz()); }  
static void p_set_band_hi(int32_t v)  { af_dsp_set_band(af_dsp_band_lo_hz(), (uint32_t)v); }  
static void p_set_txband_lo(int32_t v){ af_tx_set_band((uint32_t)v, af_tx_hi_hz()); }  
static void p_set_txband_hi(int32_t v){ af_tx_set_band(af_tx_lo_hz(), (uint32_t)v); }  
static void p_set_notch1(int32_t v)   { af_tx_set_notchf((uint32_t)v, af_tx_notch2()); }  
static void p_set_notch2(int32_t v)   { af_tx_set_notchf(af_tx_notch1(), (uint32_t)v); }  
static void p_set_gain(int32_t v)     { af_dsp_set_gain((float)v / 256.0f); }        // Q8  
static void p_set_agcmax(int32_t v)   { af_dsp_set_agcmax((float)v / 100.0f); }      // x100  
static void p_set_agcmin(int32_t v)   { af_dsp_set_agcmin((float)v / 100.0f); }      // x100  
  
#define P_SET(fn)  ((void(*)(int32_t))(fn))   // сеттеры u32/bool — совместимы по ABI  
  
static const af_param_t AF_PARAMS[] = {  
    // --- RX ---  
    { "band lo",     PT_U16,   offsetof(af_bank_t, lo_hz),          p_set_band_lo },  
    { "band hi",     PT_U16,   offsetof(af_bank_t, hi_hz),          p_set_band_hi },  
    { "cw",          PT_ONOFF, offsetof(af_bank_t, cw_on),          P_SET(af_dsp_set_cw) },  
    { "cw f0",       PT_U16,   offsetof(af_bank_t, cw_f0),          P_SET(af_dsp_set_cw_freq) },  
    { "cw bw",       PT_U16,   offsetof(af_bank_t, cw_bw),          P_SET(af_dsp_set_cw_bw) },  
    { "cw noise",    PT_U16,   offsetof(af_bank_t, cw_noise),       P_SET(af_dsp_set_cw_noise) },  
    { "agc",         PT_ONOFF, offsetof(af_bank_t, agc_on),         P_SET(af_dsp_set_agc) },  
    { "gain",        PT_GAIN,  offsetof(af_bank_t, gain_pct),       p_set_gain },  
    { "voice vol",   PT_U16,   offsetof(af_bank_t, voice_vol),      P_SET(af_voice_set_volume) },  
    { "agc ref",     PT_U16,   offsetof(af_bank_t, agc_ref),        P_SET(af_dsp_set_agcref) },  
    { "agc max",     PT_X100,  offsetof(af_bank_t, agc_max_x100),   p_set_agcmax },  
    { "agc min",     PT_X100,  offsetof(af_bank_t, agc_min_x100),   p_set_agcmin },  
    { "lim ceil",    PT_U16,   offsetof(af_bank_t, lim_ceil),       P_SET(af_dsp_set_limceil) },  
    { "nr depth",    PT_U16,   offsetof(af_bank_t, nr_depth),       P_SET(af_dsp_set_nrdepth) },  
    { "nr makeup",   PT_U16,   offsetof(af_bank_t, nr_makeup),      P_SET(af_dsp_set_nrmakeup) },  
    { "nt mu",       PT_U16,   offsetof(af_bank_t, nt_mu),          P_SET(af_dsp_set_ntmu) },  
    { "nr mu",       PT_U16,   offsetof(af_bank_t, nr_mu),          P_SET(af_dsp_set_nrmu) },  
    { "rx pres",     PT_ONOFF, offsetof(af_bank_t, rx_pres_on),     P_SET(af_dsp_set_rxpres) },  
    { "rx presmax",  PT_U16,   offsetof(af_bank_t, rx_presmax),     P_SET(af_dsp_set_rxpresmax) },  
    { "rx pres f",   PT_U16,   offsetof(af_bank_t, rx_pres_freq),   P_SET(af_dsp_set_rxpresf) },  
    { "rx pres q",   PT_U16,   offsetof(af_bank_t, rx_pres_q),      P_SET(af_dsp_set_rxpresq) },  
    { "--- TX ---",  PT_SEP,   0, NULL },  
    // --- TX ---  
    { "tx band lo",  PT_U16,   offsetof(af_bank_t, tx_lo_hz),       p_set_txband_lo },  
    { "tx band hi",  PT_U16,   offsetof(af_bank_t, tx_hi_hz),       p_set_txband_hi },  
    { "tx comp",     PT_U16,   offsetof(af_bank_t, tx_ratio_x10),   P_SET(af_tx_set_ratio) },  
    { "tx clip",     PT_U16,   offsetof(af_bank_t, tx_clip),        P_SET(af_tx_set_clip) },  
    { "tx agc tgt",  PT_U16,   offsetof(af_bank_t, tx_agc_target),  P_SET(af_tx_set_agc) },  
    { "tx agc max",  PT_U16,   offsetof(af_bank_t, tx_agc_max_x10), P_SET(af_tx_set_agcmax) },  
    { "tx gate",     PT_U16,   offsetof(af_bank_t, tx_gate_thr),    P_SET(af_tx_set_gate) },  
    { "tx gate hld", PT_U16,   offsetof(af_bank_t, tx_gate_hold_ms),P_SET(af_tx_set_gatehold) },  
    { "tx gate hys", PT_U8,    offsetof(af_bank_t, tx_gate_hyst_x10),P_SET(af_tx_set_gatehyst) },    
    { "tx gfloor",   PT_U16,   offsetof(af_bank_t, tx_gfloor),      P_SET(af_tx_set_gfloor) },  
    { "tx ap",       PT_ONOFF, offsetof(af_bank_t, tx_ap_on),       P_SET(af_tx_set_ap) },  
    { "tx soft",     PT_ONOFF, offsetof(af_bank_t, tx_softclip),    P_SET(af_tx_set_softclip) },  
    { "tx 2band",    PT_ONOFF, offsetof(af_bank_t, tx_2band_on),    P_SET(af_tx_set_2band) },  
    { "tx split",    PT_U16,   offsetof(af_bank_t, tx_split_hz),    P_SET(af_tx_set_split) },  
    { "tx lr2",      PT_ONOFF, offsetof(af_bank_t, tx_lr2_on),      P_SET(af_tx_set_lr2) },  
    { "tx cthr",     PT_U16,   offsetof(af_bank_t, tx_comp_thr),    P_SET(af_tx_set_compthr) },  
    { "tx cthr lo",  PT_U16,   offsetof(af_bank_t, tx_comp_thr_lo), P_SET(af_tx_set_compthrlo) },  
    { "tx cthr hi",  PT_U16,   offsetof(af_bank_t, tx_comp_thr_hi), P_SET(af_tx_set_compthrhi) },  
    { "tx duck",     PT_ONOFF, offsetof(af_bank_t, tx_duck_on),     P_SET(af_tx_set_duck) },  
    { "tx duckmax",  PT_U16,   offsetof(af_bank_t, tx_duckmax),     P_SET(af_tx_set_duckmax) },  
    { "tx intel",    PT_ONOFF, offsetof(af_bank_t, tx_intel_on),    P_SET(af_tx_set_intel) },  
    { "tx intel tgt",PT_U16,   offsetof(af_bank_t, tx_intel_tgt),   P_SET(af_tx_set_inteltgt) },  
    { "tx intel cut",PT_U16,   offsetof(af_bank_t, tx_intel_cut),   P_SET(af_tx_set_intelcut) },  
    { "tx intel bst",PT_U16,   offsetof(af_bank_t, tx_intel_boost), P_SET(af_tx_set_intelboost) },  
    { "tx pres",     PT_ONOFF, offsetof(af_bank_t, tx_pres_on),     P_SET(af_tx_set_pres) },  
    { "tx presmax",  PT_U16,   offsetof(af_bank_t, tx_presmax),     P_SET(af_tx_set_presmax) },  
    { "tx pres f",   PT_U16,   offsetof(af_bank_t, tx_pres_freq),   P_SET(af_tx_set_presfreq) },  
    { "tx pres q",   PT_U16,   offsetof(af_bank_t, tx_pres_q),      P_SET(af_tx_set_presq) },  
    { "tx shelf",    PT_U16,   offsetof(af_bank_t, tx_shelf_db),   P_SET(af_tx_set_shelf) },    
    { "tx shelf f",  PT_U16,   offsetof(af_bank_t, tx_shelf_freq),  P_SET(af_tx_set_shelffreq) },  
    { "tx shelf q",  PT_U16,   offsetof(af_bank_t, tx_shelf_q),     P_SET(af_tx_set_shelfq) },  
    { "tx exc",      PT_ONOFF, offsetof(af_bank_t, tx_exc_on),      P_SET(af_tx_set_exc) },  
    { "tx excmix",   PT_U16,   offsetof(af_bank_t, tx_excmix),      P_SET(af_tx_set_excmix) },  
    { "tx notch",    PT_ONOFF, offsetof(af_bank_t, tx_notch_on),    P_SET(af_tx_set_notch) },  
    { "tx notch f1", PT_U16,   offsetof(af_bank_t, tx_notch1_hz),   p_set_notch1 },  
    { "tx notch f2", PT_U16,   offsetof(af_bank_t, tx_notch2_hz),   p_set_notch2 },  
};  
#define AF_PARAMS_COUNT  (sizeof(AF_PARAMS) / sizeof(AF_PARAMS[0]))  
  
// Чтение поля банка по смещению (тип задаёт размер: u8/i16/u16).  
static int32_t bank_field(const af_bank_t* b, const af_param_t* p) {  
    const uint8_t* base = (const uint8_t*)b;  
    switch (p->type) {    
        case PT_ONOFF:    
        case PT_U8:    return (int32_t)*(const uint8_t*)(base + p->off);    
        case PT_I16:   return (int32_t)*(const int16_t*)(base + p->off);    
        default:       return (int32_t)*(const uint16_t*)(base + p->off);    
    }    
}  

// Запись поля банка по смещению (зеркало bank_field).        
static void bank_field_set(af_bank_t* b, const af_param_t* p, int32_t v) {      
    uint8_t* base = (uint8_t*)b;      
    switch (p->type) {      
        case PT_ONOFF:      
        case PT_U8:    *(uint8_t*)(base + p->off)  = (uint8_t)v;  break;      
        case PT_I16:   *(int16_t*)(base + p->off)  = (int16_t)v;  break;      
        default:       *(uint16_t*)(base + p->off) = (uint16_t)v; break;      
    }      
}      

// Диспетчер команд-меток: "<label> <value>" — longest match по таблице.  
// Вызывается до strtok (строка ещё целая). true = команда обработана.  
static bool cli_param_try(char* line) {    
    const af_param_t* best = NULL;    
    size_t best_len = 0;    
    for (size_t i = 0; i < AF_PARAMS_COUNT; i++) {    
        const af_param_t* p = &AF_PARAMS[i];    
        if (p->type == PT_SEP) continue;    
        size_t l = strlen(p->label);    
        // Метка + пробел (значение) или метка + конец строки (чтение).    
        if (l > best_len && strncmp(line, p->label, l) == 0 &&    
            (line[l] == ' ' || line[l] == '\0')) {    
            best = p; best_len = l;    
        }    
    }    
    if (best == NULL) return false;    
    
    char* v = line + best_len;    
    while (*v == ' ') v++;    
    if (*v == '\0') {    
        // Голая метка — чтение текущего значения (снимок DSP = столбец cur).    
        af_bank_t cur;    
        af_store_snapshot(&cur);    
        int32_t f = bank_field(&cur, best);    
        switch (best->type) {    
            case PT_ONOFF: Serial.printf("# %s = %s\n", best->label, f ? "on" : "off"); break;    
            case PT_X100:  Serial.printf("# %s = %ld.%02ld\n", best->label,    
                                         (long)(f / 100), (long)(f % 100)); break;    
            case PT_GAIN: {    
                unsigned g100 = (unsigned)f * 100u / 256u;    
                Serial.printf("# %s = %u.%02u\n", best->label, g100 / 100u, g100 % 100u); break;    
            }    
            default:       Serial.printf("# %s = %ld\n", best->label, (long)f); break;    
        }    
        return true;    
    }    
  
    int32_t val;  
    if (best->type == PT_ONOFF) {  
        if      (strncmp(v, "on", 2) == 0)  val = 1;  
        else if (strncmp(v, "off", 3) == 0) val = 0;  
        else { Serial.printf("# ERR: %s on|off\n", best->label); return true; }  
    } else if (best->type == PT_GAIN) {    
        val = (int32_t)(strtof(v, NULL) * 256.0f + 0.5f);   // ввод множителем: 1.5 -> 384    
    } else if (best->type == PT_X100) {    
        val = (int32_t)(strtof(v, NULL) * 100.0f + 0.5f);   // ввод множителем: 0.25 -> 25    
    } else {    
        val = (int32_t)strtol(v, NULL, 10);    
    }    
    best->set(val);    
    af_fm_request_sync();   // cur изменился — SET.TXT перерендерить  
    Serial.printf("# OK %s=%ld\n", best->label, (long)val);    
    return true;    
}    

// ---------------------------------------------------------------------------  
// af_cli_format_banks — таблица banks в текстовый буфер (для SET.TXT / af_fm).  
// Формат идентичен выводу команды 'banks': строки = AF_PARAMS,  
// столбцы = cur + ячейки 1..7. Возвращает длину текста (без '\0').  
// ---------------------------------------------------------------------------  
size_t af_cli_format_banks(char* buf, size_t cap) {  
    size_t n = 0;  
    #define FMTP(...) do { \
        if (n < cap) n += snprintf(buf + n, cap - n, __VA_ARGS__); \
    } while (0)  
  
    af_bank_t bk[AF_STORE_BANKS + 1];  
    bool      bv[AF_STORE_BANKS + 1];  
    af_store_snapshot(&bk[0]);            // bk[0] = текущее состояние  
    bv[0] = true;  
    for (uint8_t i = 0; i < AF_STORE_BANKS; i++)  
        bv[i + 1] = af_store_read(i + 1, &bk[i + 1]);  
  
    FMTP("# === BANKS (столбцы = cur + ячейки 1..7) ===\r\n");  
    FMTP("# %-14s %6s", "param", "cur");  
    for (uint8_t i = 0; i < AF_STORE_BANKS; i++) FMTP(" %6u", i + 1);  
    FMTP("\r\n");  
  
    for (size_t p = 0; p < AF_PARAMS_COUNT; p++) {  
        const af_param_t* par = &AF_PARAMS[p];  
        if (par->type == PT_SEP) { FMTP("# %s\r\n", par->label); continue; }  
        FMTP("# %-14s", par->label);  
        for (uint8_t i = 0; i <= AF_STORE_BANKS; i++) {  
            if (!bv[i]) { FMTP(" %6s", "-----"); continue; }  
            int32_t f = bank_field(&bk[i], par);  
            switch (par->type) {  
                case PT_ONOFF:  
                    FMTP(" %6s", f ? "on" : "off"); break;  
                case PT_X100:  
                    FMTP(" %3ld.%02ld", (long)(f / 100), (long)(f % 100)); break;  
                case PT_GAIN: {  
                    unsigned g100 = (unsigned)f * 100u / 256u;  
                    FMTP(" %3u.%02u", g100 / 100u, g100 % 100u); break;  
                }  
                default:  
                    FMTP(" %6ld", (long)f); break;  
            }  
        }  
        FMTP("\r\n");  
    }  
    #undef FMTP  
    if (n < cap) buf[n] = '\0'; else buf[cap - 1] = '\0';  
    return n;  
}  

// ---------------------------------------------------------------------------    
// af_cli_apply_set_txt — парсер SET.TXT с диска (запись хоста -> банки).    
// Формат = вывод af_cli_format_banks: строки "# <label> <cur> <1>..<7>".    
// Колонка cur игнорируется (только чтение); "-----" и пустые банки — пропуск.    
// Применённые поля пишутся во flash одним окном af_store_write_all.    
// Возвращает true, если хоть одно поле применено.    
// ---------------------------------------------------------------------------    
bool af_cli_apply_set_txt(const char* txt, size_t len) {      
    af_bank_t bk[AF_STORE_BANKS];      
    bool      bv[AF_STORE_BANKS];      
    for (uint8_t i = 0; i < AF_STORE_BANKS; i++)      
        bv[i] = af_store_read(i + 1, &bk[i]);      
      
    uint32_t applied = 0;      
    const char* p   = txt;      
    const char* end = txt + len;      
      
    while (p < end) {      
        const char* eol = p;      
        while (eol < end && *eol != '\n') eol++;      
      
        const char* s = p;      
        while (s < eol && (*s == '#' || *s == ' ' || *s == '\t' || *s == '\r')) s++;      
      
        // Метка из AF_PARAMS — longest match (как в cli_param_try).      
        const af_param_t* best = NULL;      
        size_t best_len = 0;      
        for (size_t i = 0; i < AF_PARAMS_COUNT; i++) {      
            const af_param_t* par = &AF_PARAMS[i];      
            if (par->type == PT_SEP) continue;      
            size_t l = strlen(par->label);      
            if (l > best_len && (size_t)(eol - s) > l &&      
                strncmp(s, par->label, l) == 0 && s[l] == ' ') {      
                best = par; best_len = l;      
            }      
        }      
      
        if (best) {        
            const char* v = s + best_len;        
            for (uint8_t col = 0; col <= AF_STORE_BANKS; col++) {        
                while (v < eol && (*v == ' ' || *v == '\t')) v++;        
                if (v >= eol || *v == '\r') break;        
                char tok[16]; uint8_t tl = 0;        
                while (v < eol && *v != ' ' && *v != '\t' && *v != '\r') {        
                    if (tl < sizeof(tok) - 1) tok[tl++] = *v;        
                    v++;        
                }        
                tok[tl] = '\0';        
        
                if (strcmp(tok, "-----") == 0) continue;        
                if (col > 0 && !bv[col - 1]) continue;   // банк пуст — не трогаем        
        
                int32_t val;        
                if (best->type == PT_ONOFF) {        
                    if      (strncmp(tok, "on", 2) == 0)  val = 1;        
                    else if (strncmp(tok, "off", 3) == 0) val = 0;        
                    else continue;        
                } else if (best->type == PT_GAIN) {        
                    val = (int32_t)(strtof(tok, NULL) * 256.0f + 0.5f);        
                } else if (best->type == PT_X100) {        
                    val = (int32_t)(strtof(tok, NULL) * 100.0f + 0.5f);        
                } else {        
                    val = (int32_t)strtol(tok, NULL, 10);        
                }        
        
                if (col == 0) {        
                    best->set(val);          // cur — в живой DSP, как CLI set        
                } else {        
                    bank_field_set(&bk[col - 1], best, val);        
                }        
                applied++;        
            }        
        }        
        p = (eol < end) ? eol + 1 : end;      
    }      
      
    if (applied == 0) return false;      
    Serial.printf("# fm: SET.TXT parsed, %lu fields -> writing banks...\n",      
                  (unsigned long)applied);      
    Serial.flush();      
    af_store_write_all(bk);          // park + окно + erase/program (~1 с)      
    return true;      
}      

// ---------------------------------------------------------------------------  
// Разбор одной строки  
// ---------------------------------------------------------------------------  
static void cli_exec(char* line) {    
    // Пропуск ведущих пробелов; пустые строки и комментарии — мимо.    
    while (*line == ' ' || *line == '\t') ++line;    
    if (*line == '\0' || *line == '#') return;    
    
    Serial.printf("# > %s\n", line);      // эхо введённой команды      
      
    if (cli_param_try(line)) return;      // команды-метки из таблицы banks      
      
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
    else if (strcmp(cmd, "fmap") == 0) {  
        cli_fmap();  
    }  
    else if (strcmp(cmd, "dump") == 0) {      
        char* v = strtok(NULL, " \t");    
        if (v == NULL) {    
            dump_period_ms = 0;                    // однократно    
            cli_dump();    
        } else if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0) {    
            dump_period_ms = 0;    
            Serial.println("# OK dump=off");    
        } else {    
            uint32_t sec = (uint32_t)strtoul(v, NULL, 10);    
            if (sec < 1 || sec > 3600) { Serial.println("# ERR: dump [1..3600|off]"); return; }    
            dump_period_ms = sec * 1000u;    
            dump_next_ms   = millis() + dump_period_ms;    
            Serial.printf("# OK dump every %lu s\n", (unsigned long)sec);    
            cli_dump();                            // первый сразу    
        }    
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
        if (af_store_save((uint8_t)idx)) {    
            af_fm_request_sync();   // ячейка перезаписана — SET.TXT перерендерить  
            Serial.printf("# OK saved bank %d\n", idx);    
        } else {    
            Serial.printf("# ERR save bank %d\n", idx);    
        }    
    }  
    else if (strcmp(cmd, "load") == 0) {  
        char* v = strtok(NULL, " \t");  
        if (v == NULL) { Serial.println("# ERR: load <0..7>"); return; }  
        int idx = (int)strtoul(v, NULL, 10);  
        if (idx == 0) {                          // выход в ручной режим    
            af_led_set_bank(0);    
            af_gpio_set_bank_code(0);            // stats 'bank=' = manual  
            Serial.println("# OK manual mode (bank 0)");    
            return;    
        }    
        if (idx < 1 || idx > AF_STORE_BANKS) {  
            Serial.println("# ERR: load <0..7>");  
            return;  
        } 
        if (af_store_load(idx)) {        
            af_led_set_bank((uint8_t)idx);   // цвет вспышки = загруженный банк        
            af_gpio_set_bank_code((uint8_t)idx);  // stats 'bank=' = активный банк    
            af_voice_play((af_voice_id_t)(VOICE_PROF_1 + idx - 1));  // "профиль N"      
            af_fm_request_sync();   // cur = загруженный банк — SET.TXT перерендерить  
            Serial.printf("# OK loaded bank %d\n", idx);      
        } else {        
            Serial.printf("# bank %d: empty — ignored\n", idx);        
        }        
    }  
    else if (strcmp(cmd, "banks") == 0) {  
        char* sub = strtok(NULL, " \t");  
        if (sub && strcmp(sub, "clr") == 0) {          
            // Перезаписать все 7 ячеек заводскими пресетами.        
            // Единый источник дефолтов — af_store_fill_defaults      
            // (af_store.cpp): полный снимок RX+TX + per-банк отличия.      
            af_bank_t preset[AF_STORE_BANKS];      
            af_store_fill_defaults(preset);      
      
            Serial.println("# banks clr: writing 7 banks...");        
            Serial.flush();        
            af_store_write_all(preset);   // park + nominal + DMA-окно + erase/program        
  
            // + очистка голосовых слотов всех банков (erase 0x140000..0x1EFFFF,    
            // окно ~3-4 с). Отказ при активном rec/play — слоты остаются.    
            if (af_rec_erase_all()) {      
                Serial.println("# banks clr: rec slots erased");      
            } else {      
                Serial.println("# banks clr: rec slots kept (rec/play active)");      
            }      
            af_fm_request_sync();   // пресеты + слоты изменились — перерендерить  
        }      
        // Без аргументов — та же таблица, что пишется в SET.TXT (af_fm).  
        // Единый форматтер — af_cli_format_banks.  
        static char tbl[4096];  
        af_cli_format_banks(tbl, sizeof(tbl));  
        Serial.print(tbl);  
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
  
    else if (strcmp(cmd, "led") == 0) {  
        char* s = strtok(NULL, " \t");  
        if (s && strcmp(s, "on") == 0) {  
            af_led_set_enabled(true);  
            Serial.println("# OK led=on");  
        } else if (s && strcmp(s, "off") == 0) {  
            af_led_set_enabled(false);  
            Serial.println("# OK led=off");  
        } else {  
            Serial.printf("# led = %s\n", af_led_enabled() ? "on" : "off");  
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
        else if (a && strcmp(a, "off") == 0) {    
            if (af_rec_is_recording() || af_rec_is_playing()) {    
                Serial.println("# ERR: tx off — rec/play active (stop сначала)");    
                return;    
            }    
            af_dsp_set_tx(false); Serial.println("# OK tx=off");    
        }    
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

    else if (strcmp(cmd, "rec") == 0) {      
        // Как GP9: тракт -> TX (микрофон), без PTT. Флаг — чтобы stop вернул RX.    
        if (af_rec_is_playing()) { Serial.println("# ERR rec: play активен"); return; }    
        cli_rec_tx = !af_dsp_tx_on();          // TX включили мы — мы и вернём    
        if (cli_rec_tx) af_dsp_set_tx(1);    
        if (af_rec_start()) {      
            Serial.printf("# OK rec: tx тракт (микрофон), буфер %lu с\n",        
                          (unsigned long)(AF_REC_RAM_SAMP * AF_REC_DECIM / AF_FS_OUT_HZ));        
        } else {      
            if (cli_rec_tx) { af_dsp_set_tx(0); cli_rec_tx = false; }    
            Serial.println("# ERR rec: занято или tx/rx переключается");      
        }      
    }      
    else if (strcmp(cmd, "play") == 0) {        
        // Пустой буфер -> voice_14: как GP10 — TX + PTT на GP8, звук в TX.    
        // RAM-запись -> как раньше: подмес в активный тракт, без PTT.    
        const bool stub = (af_rec_len_samples() == 0);    
        if (stub) {    
            if (!af_gpio_ptt_tx_on()) {    
                Serial.println("# ERR play: PTT занят (GP8/GP10)");    
                return;    
            }    
            if (!af_rec_play()) {    
                af_gpio_ptt_tx_off();    
                Serial.println("# ERR play: занято (rec/play активен)");    
                return;    
            }    
            cli_play_ptt = true;    
            cli_play_pending = true;    
        } else if (af_rec_play()) {          
            cli_play_pending = true;   // OK — после окончания (af_cli_poll)      
        } else {        
            Serial.println("# ERR play: занято (rec/play активен)");        
        }        
    }        
    else if (strcmp(cmd, "stop") == 0) {          
        af_rec_stop();          
        if (cli_rec_tx) { af_dsp_set_tx(0); cli_rec_tx = false; }        
        if (cli_play_ptt) { af_gpio_ptt_tx_off(); cli_play_ptt = false; }    
        cli_play_pending = false;   // прерывание — OK play не печатаем      
        Serial.println("# OK stop");          
    }          
    else if (strcmp(cmd, "recs") == 0) {      
        const uint32_t len = af_rec_len_samples();        
        if (len == 0) {        
            Serial.println("# rec: slot empty");        
        } else {        
            Serial.printf("# rec: slot used %lu samp (%lu.%lu s)  state=%s\n",          
                          (unsigned long)len,          
                          (unsigned long)(len * AF_REC_DECIM / AF_FS_OUT_HZ),          
                          (unsigned long)((len * AF_REC_DECIM % AF_FS_OUT_HZ) * 10u / AF_FS_OUT_HZ),             
                          af_rec_is_recording() ? "REC" :        
                          af_rec_is_playing()   ? "PLAY" : "idle");        
        }        
    }      

    else if (strcmp(cmd, "ftest") == 0) {  
        // ftest [mode] [iters] — тест записи в свободный сектор 0x100000.  
        // mode: 0=чистое окно, 1=+park, 2=+даунклок, 3=park+даунклок.  
        char* s_m = strtok(NULL, " \t");  
        char* s_n = strtok(NULL, " \t");  
        uint8_t  mode  = s_m ? (uint8_t)strtoul(s_m, NULL, 10) : 0;  
        uint32_t iters = s_n ? (uint32_t)strtoul(s_n, NULL, 10) : 10;  
        if (mode > 3) { Serial.println("# ERR: ftest [0..3] [N]"); return; }  
        af_store_test(mode, iters);  
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

    // CLI rec включил TX, а запись кончилась без команды stop
    // (авто-стоп по переполнению буфера в af_rec_poll) — вернуть RX.
    if (cli_rec_tx && !af_rec_is_recording()) {      
        af_dsp_set_tx(0);      
        cli_rec_tx = false;      
        Serial.println("# rec end -> tx released (rx)");      
    }      
  
    // CLI play завершился по буферу — OK после проигрывания, затем отпуск PTT/TX          
    if (cli_play_pending && !af_rec_is_playing()) {          
        cli_play_pending = false;          
        Serial.printf("# OK play: %s тракт, %lu с\n",          
                      af_dsp_tx_on() ? "tx" : "rx",          
                      (unsigned long)(af_rec_play_len() * AF_REC_DECIM / AF_FS_OUT_HZ));          
        if (cli_play_ptt) {        
            af_gpio_ptt_tx_off();        
            cli_play_ptt = false;        
        }        
    }          
      
    if (dump_period_ms != 0 && (int32_t)(millis() - dump_next_ms) >= 0) {      
        dump_next_ms = millis() + dump_period_ms;      
        cli_dump();      
    }      
    
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