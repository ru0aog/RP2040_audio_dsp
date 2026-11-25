// src/af_fm.cpp  
//  
// FAT16-диск 10 МБ, RAM-образ ~10 КБ (boot+dir+SET.TXT+CW.TXT). Раскладка:          
//   LBA 0      boot sector          
//   LBA 1..80  FAT (80 секторов — СИНТЕЗ в af_fm_msc_read, в RAM не хранится)          
//   LBA 81     root dir (SET.TXT + CW.TXT + REC1..REC4.WAV)              
//   LBA 82..97 data: SET.TXT (RAM)              
//   LBA 98     data: CW.TXT (RAM, 4 строки сообщений магнитофона)            
//   LBA 99..   REC1..REC4.WAV — виртуальные, синтез в af_fm_msc_read              
//   LBA 3603.. свободный пул ~16877 секторов (~8.4 МБ): сюда хост пишет              
//              данные новых файлов; Explorer пропускает копирование              
//              любого размера по проверке свободного места                
//    
// Маппинг RAM-образа (fm_disk): [0]=boot, [1]=dir, [2..17]=SET.TXT, [18]=CW.TXT.    
// FAT не хранится: цепочки регулярные, сектор генерируется формулой fat_sector().    
//  
// SET.TXT генерируется af_cli_format_banks() — тот же текст, что 'banks'.      
// Запись с ПК: дебаунс 800 мс -> af_cli_apply_set_txt -> банки во flash.      
//
// CW.TXT — 4 строки вызовов (af_cw_msgs, af_store.cpp). Пустая строка =      
// голосовой слот. Запись с ПК: дебаунс 800 мс -> парсер -> af_store_cw_save.    
//      
// Аплоад: перезапись RECn.WAV -> стриминг-конвертер -> инкрементальный      
// ADPCM-encode -> ram_buf (нибблы, ~29 с) -> слот n. Любой WAV (PCM 8/16/24/32 бит,      
// mono/stereo, любая частота): ресемплинг в 7732 Гц mono int16. Коммит —    
// по таймауту 1.5 с после последнего сектора (flash-окно в af_rec_slot_write).    
// Экспорт: codec=0 — PCM WAV 8000 Гц (ресемплинг), codec=1 — декод ADPCM    
// в PCM WAV 7732 Гц (родная частота, без ресемплинга).    
  
#include <Arduino.h>  
#include <string.h>  
#include <stdio.h>
#include <stdarg.h>
#include "hardware/flash.h"

#include "af_config.h"  
#include "af_fm.h"
#include "af_cli.h"
#include "af_usb.h"          // af_usb_reenumerate — перемонтирование диска
#include "af_store.h"        // af_cw_msgs, af_store_cw_save — CW.TXT 

// Рендер таблицы banks в буфер (af_cli.cpp — там AF_PARAMS/bank_field).    
// Возвращает длину текста без NUL.    
extern size_t af_cli_format_banks(char* buf, size_t cap);    
  
#include "af_rec.h"          // af_rec_slot_len/data — WAV-экспорт    
    
static uint8_t  fm_disk[AF_FM_RAM_SECTORS][AF_FM_SECTOR_SIZE];  // ~10 КБ (19 секторов) 
static uint32_t fm_set_len = 0;        // байт в SET.TXT  
static bool     fm_dirty   = false;    // ПК писал — ждёт парсера    
static bool     fm_stale   = true;     // SET.TXT устарел — регенерировать      
static bool     fm_sizes_changed = false;  // размеры файлов изменились -> re-enumerate  
static uint32_t fm_dirty_ms = 0;       // millis() последней записи (дебаунс)        
static bool     fm_cw_dirty = false;   // ПК писал CW.TXT — ждёт парсера    
static uint32_t fm_cw_ms    = 0;       // millis() последней записи CW.TXT    
  
// --- Аплоад WAV: состояние сессии -------------------------------------------    
static bool     ul_active   = false;   // идёт приём RECn.WAV    
static uint8_t  ul_bank     = 0;       // целевой банк (1..4)      
static uint32_t ul_lba_next = 0;       // ожидаемый LBA (последовательная запись)    
static uint32_t ul_last_ms  = 0;       // millis() последнего сектора    
static uint32_t ul_file_off = 0;       // байт, прочитанный из файла    
static uint32_t ul_data_end = 0;       // конец data-чанка (байт в файле)    
static uint32_t ul_rate     = 0;       // частота входного WAV    
static uint16_t ul_channels = 0;    
static uint16_t ul_bits     = 0;      
static uint16_t ul_codec    = 1;       // WAV format tag: 1=PCM, 6=A-law, 7=μ-law  
static uint16_t ul_balign   = 0;       // block align (байт на фрейм)    
static uint32_t ul_in_pos   = 0;       // позиция во входном потоке (фреймы)    
static uint32_t ul_out_pos  = 0;       // записано выходных отсчётов в ram_buf    
static int16_t  ul_prev     = 0;       // предыдущий входной отсчёт (интерполяция)    
static bool     ul_trunc    = false;   // ram_buf заполнен — хвост отброшен    
static uint8_t  ul_tail[8];            // недошедший кусок фрейма между секторами      
static uint8_t  ul_tail_len = 0;      
// Инкрементальный IMA-ADPCM энкодер аплоада: ul_push пишет нибблы      
// в ram_buf по мере ресемплинга — буфер вмещает ~29 с (224000 отсчётов),      
// а не 7.2 с PCM-стейджинга. Состояние локальное — не делить с      
// энкодером записи в af_rec.cpp (тот на Core 1).      
static int16_t  ul_enc_pred = 0;        
static int8_t   ul_enc_idx  = 0;        
  
// Отложенная печать: Serial.printf внутри MSC-колбэков (TinyUSB task)    
// рекурсивно дёргает USB-стек -> intermittent reboot. Колбэки только    
// выставляют флаги, печать — из af_fm_poll (обычный контекст Core 0).    
static char     fm_log[160];    
static volatile bool fm_log_pending = false;    
static void fm_logf(const char* fmt, ...) {    
    if (fm_log_pending) return;              // одно сообщение за раз    
    va_list ap; va_start(ap, fmt);    
    vsnprintf(fm_log, sizeof(fm_log), fmt, ap);    
    va_end(ap);    
    fm_log_pending = true;    
}       

// --- Перехват dir-записей: первый кластер файла -> банк ----------------------  
// Windows при замене делает delete+create: dir-запись с именем RECn.WAV и  
// первым кластером появляется ДО записи данных. Данные идут в свободный пул  
// подряд от первого кластера — мапим их в слот по имени, а не по LBA.  
static uint16_t dir_first_cl[AF_FM_WAV_FILES + 1];   // [bank 1..4] -> cluster, 0=нет    

static uint16_t rd16(const uint8_t* p);

// Разбор записанного сектора root dir: обновить dir_first_cl[].  
static void dir_track(const uint8_t* d) {    
    static const char wav_names[AF_FM_WAV_FILES][12] = {      
        "REC1    WAV", "REC2    WAV", "REC3    WAV", "REC4    WAV" };      
    for (uint8_t e = 0; e < 16; e++) {               // 16 записей по 32 Б      
        const uint8_t* p = d + e * 32;      
        if (p[0] == 0x00) continue;                  // пустая запись      
        for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {      
            if (memcmp(p + 1, wav_names[i] + 1, 10) != 0) continue;    
            // Имя совпало (по 10 символам после первого байта —    
            // у удалённой записи p[0]=0xE5, имя остальное).    
            if (p[0] == 0xE5) {    
                // Хост удалил RECn.WAV (delete+create при замене):    
                // stale-кластер сбросить — иначе он перехватит данные    
                // чужого аплоада в "best_cl <= cl" матчинге.    
                if (dir_first_cl[i + 1]) {    
                    dir_first_cl[i + 1] = 0;    
                    fm_logf("# fm: dir REC%u.WAV deleted", (unsigned)(i + 1));    
                }    
                break;    
            }    
            uint16_t cl = rd16(p + 26);    
            if (cl != dir_first_cl[i + 1]) {      
                dir_first_cl[i + 1] = cl;      
                fm_logf("# fm: dir REC%u.WAV -> cluster %u",      
                        (unsigned)(i + 1), (unsigned)cl);      
            }      
            break;    
        }    
    }    
}    

// Чужие записи в root dir (System Volume Information, корзина и т.п.):    
// хост выделяет им кластеры из нашей разметки — они указывают внутрь    
// цепочек RECn.WAV, и chkdsk видит "первая единица распределения    
// повреждена" / "неустранимая ошибка в каталоге". Помечаем такие записи    
// удалёнными (0xE5): chkdsk их не видит, Windows при необходимости    
// создаст каталог заново — цикл безвредный.    
// Вызывается после memcpy dir-сектора в af_fm_msc_write, до dir_track. 
static void dir_fix_foreign(uint8_t* d) {  
    static const char known[6][12] = {  
        "SET     TXT", "CW      TXT",  
        "REC1    WAV", "REC2    WAV", "REC3    WAV", "REC4    WAV" };  
    for (uint8_t e = 0; e < 16; e++) {  
        uint8_t* p = d + e * 32;  
        if (p[0] == 0xE5 || p[0] == 0x00 || p[0] == 0x05) continue;  // удалённая/пустая  
        if (p[11] == 0x0F) continue;                                 // LFN-запись  
        bool ours = false;    
        for (uint8_t i = 0; i < 6; i++)    
            if (memcmp(p, known[i], 11) == 0) { ours = true; break; }    
        if (ours) continue;      
        // Удалённая запись — chkdsk пропускает. Вместе с ней помечаем  
        // подряд идущие перед ней LFN-записи (attr=0x0F): они принадлежат  
        // этому же файлу, без родителя — сироты ("повреждённый длинный  
        // элемент папки" в chkdsk).  
        p[0] = 0xE5;  
        for (int k = (int)e - 1; k >= 0; k--) {  
            uint8_t* l = d + k * 32;  
            if (l[0] == 0x00 || l[0] == 0xE5 || l[11] != 0x0F) break;  
            l[0] = 0xE5;  
        }  
    }      
}      
  
// ---------------------------------------------------------------------------    
// Построение образа    
// ---------------------------------------------------------------------------    
  
static void fm_build_boot(void) {  
    uint8_t* b = fm_disk[0];  
    memset(b, 0, AF_FM_SECTOR_SIZE);  
    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;  
    memcpy(&b[3], "MSDOS5.0", 8);  
    b[11] = 0x00; b[12] = 0x02;          // 512 Б/сектор  
    b[13] = 1;                           // 1 сектор/кластер  
    b[14] = 1;  b[15] = 0;               // reserved    
    b[16] = 1;                           // FAT count    
    b[17] = 16; b[18] = 0;               // root entries = 16 (1 сектор)    
    const uint16_t ts16 = (uint16_t)AF_FM_SECTOR_COUNT;   // 20480 < 65536  
    memcpy(&b[19], &ts16, 2);        
    b[21] = 0xF8;                        // media          
    const uint16_t fat_sz = (uint16_t)AF_FM_FAT_SECTORS;          
    memcpy(&b[22], &fat_sz, 2);          // sectors/FAT = 80    
    b[24] = 1;  b[25] = 0;               // spt      
    b[26] = 1;  b[27] = 0;               // heads      
    b[36] = 0x80; b[38] = 0x29;      
    const uint32_t vid = 0xA0D5F004;   // bump: новый volume serial ->        
    memcpy(&b[39], &vid, 4);           // Windows не подставит кэш старого диска      
    memcpy(&b[43], "AUDIO_DSP3 ", 11); // + метка для надёжности      
    memcpy(&b[54], "FAT16   ", 8);    
    b[510] = 0x55; b[511] = 0xAA;  
}  
  
// FAT16: значение записи n по формуле — FAT в RAM не хранится,    
// сектор синтезируется в fat_sector() при чтении хостом.    
// Раскладка цепочек (кластер = 1 сектор, нумерация с 2):    
//   2..17        SET.TXT (16 кластеров, вся территория занята)    
//   18           CW.TXT    
//   19..         REC1..REC4.WAV по AF_FM_WAV_CLUSTERS кластеров    
//   далее        свободный пул (0)    
static uint16_t fat_entry(uint16_t n) {    
    if (n == 0) return 0xFFF8;              // media + EOC    
    if (n == 1) return 0xFFFF;    
    uint16_t cl = 2;    
    // SET.TXT: цепочка AF_FM_SET_CLUSTERS    
    if (n >= cl && n < cl + AF_FM_SET_CLUSTERS)    
        return (n == cl + AF_FM_SET_CLUSTERS - 1) ? 0xFFFF : (uint16_t)(n + 1);    
    cl += AF_FM_SET_CLUSTERS;    
    // CW.TXT: 1 кластер    
    if (n == cl) return 0xFFFF;    
    cl += AF_FM_CW_CLUSTERS;    
    // RECn.WAV: фиксированная сетка AF_FM_WAV_CLUSTERS на файл    
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        if (n >= cl && n < cl + AF_FM_WAV_CLUSTERS)    
            return (n == cl + AF_FM_WAV_CLUSTERS - 1) ? 0xFFFF : (uint16_t)(n + 1);    
        cl += AF_FM_WAV_CLUSTERS;    
    }    
    return 0;                               // свободный пул    
}    
    
// Синтез одного сектора FAT (lba 1..AF_FM_FAT_SECTORS) в buffer.    
// Сектор = 256 записей по 2 Б, little-endian.    
static void fat_sector(uint32_t lba, uint8_t* buffer) {      
    uint32_t base = (lba - 1) * 256;        // первая запись сектора      
    for (uint32_t i = 0; i < 256; i++) {    
        uint16_t v = fat_entry((uint16_t)(base + i));    
        buffer[i * 2]     = (uint8_t)v;     // byte-wise: буфер TinyUSB    
        buffer[i * 2 + 1] = (uint8_t)(v >> 8);  // может быть невыровнен —    
    }                                       // (uint16_t*)buffer -> hardfault    
}      
  
static void dir_entry(uint8_t* e, const char* name83, uint16_t cl, uint32_t size) {    
    memset(e, 0, 32);    
    memcpy(e, name83, 11);    
    e[11] = 0x20;                                // archive    
    memcpy(e + 26, &cl, 2);    
    memcpy(e + 28, &size, 4);    
}    
    
// Каталог: SET.TXT + CW.TXT + REC1..REC4.WAV. wav_sizes[i] = байт (44 = пусто).      
static void fm_build_dir(uint32_t set_size, uint16_t set_clusters,    
                         const uint32_t* wav_sizes) {    
    uint8_t* d = fm_disk[AF_FM_DIR_RAM];    // dir — fm_disk[1], не LBA 81    
    memset(d, 0, AF_FM_SECTOR_SIZE);    
    dir_entry(&d[0], "SET     TXT", set_clusters ? 2 : 0, set_size);      
    dir_entry(&d[32], "CW      TXT", (uint16_t)(2 + AF_FM_SET_CLUSTERS),    
              AF_FM_CW_CLUSTERS * AF_FM_SECTOR_SIZE);   // размер = кластер,    
                                                        // хвост — пробелы    
    static const char wav_names[AF_FM_WAV_FILES][12] = {    
        "REC1    WAV", "REC2    WAV", "REC3    WAV", "REC4    WAV" }; 
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {      
        uint16_t first = (uint16_t)(2 + AF_FM_SET_CLUSTERS + AF_FM_CW_CLUSTERS    
                                    + i * AF_FM_WAV_CLUSTERS);      
        dir_entry(&d[32 * (i + 2)], wav_names[i], wav_sizes[i] ? first : 0, wav_sizes[i]);    
    }      
}    
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
  
void af_fm_init(void) {  
    memset(dir_first_cl, 0, sizeof(dir_first_cl));  
    memset(fm_disk, 0, sizeof(fm_disk));  
    fm_build_boot();      
    static const uint32_t zsz4[AF_FM_WAV_FILES]  = {0};        
    fm_build_dir(0, 0, zsz4);      // FAT не строится — синтез на чтении    
    fm_set_len = 0;  
    fm_dirty   = false;  
    fm_stale   = true;                 // первый sync — после af_store_init  
}  
  
void af_fm_request_sync(void) { fm_stale = true; }  
  
void af_fm_sync(void) {    
    // SET.TXT рендерится в RAM-область (fm_disk[2..17])  
    char* txt = (char*)&fm_disk[AF_FM_SET_RAM][0];      
    size_t cap = AF_FM_SET_CLUSTERS * AF_FM_SECTOR_SIZE;    
    size_t n = af_cli_format_banks(txt, cap);    
    if (n >= cap) n = cap - 1;    
    fm_set_len = (uint32_t)n;    
    uint16_t set_clusters = (uint16_t)((fm_set_len + 511) / 512);    
    
    // WAV: размер = 44 + out_samp*2; пустой слот -> 44 Б (только заголовок).    
    uint16_t wav_clusters[AF_FM_WAV_FILES];        
    uint32_t wav_sizes[AF_FM_WAV_FILES];        
    static uint32_t prev_sizes[AF_FM_WAV_FILES] = {0};   // для детекта изменений    
    static bool     first_sync = true;                 // первый sync — базовая линия    
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {            
        uint32_t len = af_rec_slot_len(i + 1);            
        // codec=1 (ADPCM): экспорт в PCM на родных 7732 Гц — out_samp = len.    
        // codec=0 (PCM): ресемплинг 7732 -> 8000.    
        uint32_t out_samp = (af_rec_slot_codec(i + 1) == AF_REC_CODEC_ADPCM)    
                            ? len    
                            : (len * AF_FM_WAV_RATE) / AF_FM_REC_RATE;            
        if (out_samp > AF_FM_WAV_MAX_SAMP)          // cap: не выйти за кластеры    
            out_samp = AF_FM_WAV_MAX_SAMP;          // RECn.WAV на виртуальном диске    
        wav_sizes[i] = 44 + out_samp * 2;            
        wav_clusters[i] = (uint16_t)((wav_sizes[i] + 511) / 512);        
        // Re-enumerate при ЛЮБОЙ смене размера: у хоста закэшированы  
        // старые dir+FAT — без перемонтирования он читает/пишет по  
        // устаревшей раскладке (cross-link при записи 28-с записи).  
        if (!first_sync && prev_sizes[i] != wav_sizes[i])        
            fm_sizes_changed = true;        
        prev_sizes[i] = wav_sizes[i];      
    }        
    first_sync = false;    
    // CW.TXT: 4 строки из af_cw_msgs, CRLF, хвост — пробелы (чистый вид    
    // в редакторе; парсер пробелы/CRLF пропускает).    
    {    
        uint8_t* cw = &fm_disk[AF_FM_CW_RAM][0];      
        memset(cw, ' ', AF_FM_CW_CLUSTERS * AF_FM_SECTOR_SIZE);    
        uint32_t off = 0;    
        for (uint8_t i = 0; i < AF_CW_MSGS; i++) {    
            size_t l = strlen(af_cw_msgs[i]);    
            if (off + l + 2 > AF_FM_CW_CLUSTERS * AF_FM_SECTOR_SIZE) break;    
            memcpy(cw + off, af_cw_msgs[i], l);    
            off += l;    
            cw[off++] = '\r'; cw[off++] = '\n';    
        }    
    }    
    fm_build_dir(fm_set_len, set_clusters, wav_sizes);   // FAT — синтез    
    fm_stale = false;        
}      

static void ul_commit(void);   // определена ниже — коммит аплоада по таймауту  

// Парсер CW.TXT: до 4 непустых строк -> af_cw_msgs -> flash.    
// Пустые строки пропускаются; строка > AF_CW_MSG_LEN-1 обрезается.    
static void cw_txt_parse(const char* txt, uint32_t cap) {    
    uint8_t idx = 0;    
    uint32_t pos = 0;    
    while (pos < cap && idx < AF_CW_MSGS) {    
        // пропустить пробелы/CR/LF    
        while (pos < cap && (txt[pos] == ' ' || txt[pos] == '\r' ||    
                             txt[pos] == '\n' || txt[pos] == '\t')) pos++;    
        if (pos >= cap || txt[pos] == '\0') break;    
        char line[AF_CW_MSG_LEN];    
        uint8_t l = 0;    
        while (pos < cap && txt[pos] != '\r' && txt[pos] != '\n' &&    
               l < AF_CW_MSG_LEN - 1)    
            line[l++] = txt[pos++];    
        line[l] = '\0';    
        // обрезать хвостовые пробелы    
        while (l > 0 && line[l - 1] == ' ') line[--l] = '\0';    
        memcpy(af_cw_msgs[idx], line, l + 1);   // l==0 -> пустая строка = голосовой слот    
        idx++;      
        while (pos < cap && (txt[pos] == '\r' || txt[pos] == '\n')) pos++;    
    }    
    // Оставшиеся строки — пустые (голосовые слоты)    
    for (; idx < AF_CW_MSGS; idx++) af_cw_msgs[idx][0] = '\0';    
}    

void af_fm_poll(void) {        
    // Отложенная печать из MSC-колбэков — здесь обычный контекст,    
    // printf безопасен.    
    if (fm_log_pending) {
        fm_log_pending = false;    
        Serial.println(fm_log);    
    }    
    // Коммит аплоада WAV: 1.5 с тишины после последнего сектора.      
    if (ul_active && (int32_t)(millis() - ul_last_ms) >= 1500)    
        ul_commit();    
    // Дебаунс: хост пишет файл пачкой секторов — ждём 800 мс тишины,      
    // потом парсим SET.TXT целиком и пишем банки во flash.      
    if (fm_dirty && (int32_t)(millis() - fm_dirty_ms) >= 800) {        
        fm_dirty = false;      
        const char* txt = (const char*)&fm_disk[AF_FM_SET_RAM][0];          
        bool applied = af_cli_apply_set_txt(txt,        
                          AF_FM_SET_CLUSTERS * AF_FM_SECTOR_SIZE);        
        Serial.printf("# fm: SET.TXT %s\n",        
                      applied ? "parsed -> banks written" : "parse failed");        
        fm_stale = true;               // перерендерить из записанных банков           
    }        
    // CW.TXT: тот же дебаунс 800 мс -> парсер -> flash -> re-render.    
    if (fm_cw_dirty && (int32_t)(millis() - fm_cw_ms) >= 800) {    
        fm_cw_dirty = false;    
        cw_txt_parse((const char*)&fm_disk[AF_FM_CW_RAM][0],      
                     AF_FM_CW_CLUSTERS * AF_FM_SECTOR_SIZE);    
        af_store_cw_save();            // flash-окно + af_fm_request_sync    
        Serial.println("# fm: CW.TXT parsed -> msgs written");    
    }    
    if (fm_stale) af_fm_sync();            
    // Пере-энумерация при любой смене размеров RECn.WAV (запись,  
    // заливка voice14, аплоад): у хоста закэшированы dir+FAT —  
    // без перемонтирования он работает по устаревшей раскладке.  
    if (fm_sizes_changed) {  
        fm_sizes_changed = false;  
        af_usb_reenumerate();  
    }  
}      
  
bool     af_fm_dirty(void)    { return fm_dirty; }  
uint32_t af_fm_set_size(void) { return fm_set_len; }  
  
// ---------------------------------------------------------------------------  
// MSC-колбэки  
// ---------------------------------------------------------------------------  
  
// WAV-заголовок 44 Б: PCM mono 16-bit, rate = параметр    
// (8000 для PCM-слотов, 7732 для ADPCM — экспорт без ресемплинга).      
static void wav_header(uint8_t* h, uint32_t data_bytes, uint32_t rate) {      
    memset(h, 0, 44);      
    memcpy(h + 0, "RIFF", 4);      
    uint32_t riff = 36 + data_bytes;   memcpy(h + 4, &riff, 4);      
    memcpy(h + 8, "WAVEfmt ", 8);      
    uint32_t fmt = 16;                 memcpy(h + 16, &fmt, 4);      
    uint16_t pcm = 1, ch = 1, bits = 16, align = 2;      
    uint32_t brate = rate * 2;      
    memcpy(h + 20, &pcm, 2);   memcpy(h + 22, &ch, 2);    
    memcpy(h + 24, &rate, 4);  memcpy(h + 28, &brate, 4);    
    memcpy(h + 32, &align, 2); memcpy(h + 34, &bits, 2);    
    memcpy(h + 36, "data", 4); memcpy(h + 40, &data_bytes, 4);    
}    
    
// --- IMA-ADPCM декодер для экспорта (локальное состояние — не делить    
// с декодером воспроизведения в af_rec.cpp, тот на Core 1).    
// Хост читает файл последовательно: кэшируем позицию; при скачке    
// назад — сброс и декод с нуля (до ~224К отсчётов, миллисекунды)
static const int16_t fm_adpcm_step[89] = {    
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,    
    50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,    
    253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,    
    1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,    
    3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,    
    10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,    
    27086,29794,32767    
};    
static const int8_t fm_adpcm_idx[16] = {    
    -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8    
};    
// Состояние декодера — PER-BANK: хост читает файлы с interleaving    
// (копия REC1 + антивирус/Explorer трогают REC2..4, dir, FAT).    
// Однобанковый кэш инвалидировался при каждом переключении — сектор    
// декодировался с нуля до k (до 224К итераций), суммарно O(n²)    
// на файл -> watchdog -> зависание/reboot при копировании.    
static int16_t  fm_dec_pred[AF_FM_WAV_FILES + 1];   // [bank 1..4]    
static int8_t   fm_dec_idx [AF_FM_WAV_FILES + 1];    
static uint32_t fm_dec_pos [AF_FM_WAV_FILES + 1];   // следующий отсчёт    
static bool     fm_dec_init[AF_FM_WAV_FILES + 1];   // состояние валидно    
    
// Чекпоинты каждые FM_DEC_CKPT отсчётов — тоже per-bank:    
// скачок назад рестартует с ближайшего снапшота своего банка.    
#define FM_DEC_CKPT   4096u        
#define FM_DEC_CKPTS  (AF_FM_WAV_MAX_SAMP / FM_DEC_CKPT + 1)   // ~56    
static int16_t fm_ck_pred[AF_FM_WAV_FILES + 1][FM_DEC_CKPTS];  // ~440 Б    
static int8_t  fm_ck_idx [AF_FM_WAV_FILES + 1][FM_DEC_CKPTS];  // ~220 Б    
static uint32_t fm_ck_cnt[AF_FM_WAV_FILES + 1];   // сколько чекпоинтов записано     
    
static int16_t fm_adpcm_dec(uint8_t bank, uint8_t nib) {      
    int16_t* pred = &fm_dec_pred[bank];    
    int8_t*  idx  = &fm_dec_idx [bank];    
    int32_t step = fm_adpcm_step[*idx];      
    int32_t delta = step >> 3;      
    if (nib & 4) delta += step;      
    if (nib & 2) delta += step >> 1;      
    if (nib & 1) delta += step >> 2;      
    *pred += (nib & 8) ? -delta : delta;      
    if (*pred > 32767) *pred = 32767;      
    else if (*pred < -32768) *pred = -32768;      
    *idx += fm_adpcm_idx[nib & 15];      
    if (*idx < 0) *idx = 0; else if (*idx > 88) *idx = 88;      
    return *pred;      
}      
    
// Отсчёт k ADPCM-слота: последовательный декод с кэшем позиции      
// и чекпоинтами каждые FM_DEC_CKPT отсчётов.      
// max_k — потолок индекса (len слота): декодер не выйдет за данные  
// даже при рассинхроне кэша позиции.  
static int16_t fm_adpcm_sample(uint8_t bank, const uint8_t* data, uint32_t k,    
                               uint32_t max_k) {          
    if (k > max_k) k = max_k;    
    // Рестарт: первое обращение к банку или скачок назад.    
    // Смена банка НЕ сбрасывает — у каждого банка своё состояние.    
    if (!fm_dec_init[bank] || k + 1 < fm_dec_pos[bank]) {    
        uint32_t cp = k / FM_DEC_CKPT;    
        if (cp > 0 && cp < fm_ck_cnt[bank]) {         // снапшот своего банка    
            fm_dec_pred[bank] = fm_ck_pred[bank][cp];    
            fm_dec_idx [bank] = fm_ck_idx [bank][cp];    
            fm_dec_pos [bank] = cp * FM_DEC_CKPT;    
        } else {    
            fm_dec_pred[bank] = 0; fm_dec_idx[bank] = 0; fm_dec_pos[bank] = 0;    
        }    
        fm_dec_init[bank] = true;    
    }    
    int16_t v = fm_dec_pred[bank];    
    while (fm_dec_pos[bank] <= k) {    
        uint32_t pos = fm_dec_pos[bank];    
        uint8_t b = data[pos >> 1];    
        v = fm_adpcm_dec(bank, (pos & 1) ? (b >> 4) : (b & 0x0F));    
        fm_dec_pos[bank] = ++pos;    
        if ((pos % FM_DEC_CKPT) == 0) {               // снапшот на границе    
            uint32_t cp = pos / FM_DEC_CKPT;    
            if (cp < FM_DEC_CKPTS) {    
                fm_ck_pred[bank][cp] = fm_dec_pred[bank];    
                fm_ck_idx [bank][cp] = fm_dec_idx [bank];    
                if (cp >= fm_ck_cnt[bank]) fm_ck_cnt[bank] = cp + 1;    
            }    
        }    
    }    
    return v;    
}    
    
// IMA-ADPCM энкодер аплоада: один отсчёт -> ниббл.      
// Таблицы fm_adpcm_step/fm_adpcm_idx общие с декодером экспорта.      
static uint8_t ul_adpcm_enc(int16_t s) {      
    int32_t step = fm_adpcm_step[ul_enc_idx];      
    int32_t diff = (int32_t)s - ul_enc_pred;      
    uint8_t nib = 0;      
    if (diff < 0) { nib = 8; diff = -diff; }      
    int32_t delta = step >> 3;      
    if (diff >= step)      { nib |= 4; diff -= step;      delta += step; }      
    if (diff >= (step>>1)) { nib |= 2; diff -= (step>>1); delta += (step>>1); }      
    if (diff >= (step>>2)) { nib |= 1;                    delta += (step>>2); }      
    ul_enc_pred += (nib & 8) ? -delta : delta;      
    if (ul_enc_pred > 32767) ul_enc_pred = 32767;      
    else if (ul_enc_pred < -32768) ul_enc_pred = -32768;      
    ul_enc_idx += fm_adpcm_idx[nib & 15];      
    if (ul_enc_idx < 0) ul_enc_idx = 0; else if (ul_enc_idx > 88) ul_enc_idx = 88;      
    return nib;      
}      
      
// Синтез одного сектора WAV-файла слота (bank = 1..4) в buffer.
// off_in_file = (lba - file_lba) * 512.      
// codec=0: PCM-слот -> ресемплинг 7732->8000. codec=1: ADPCM -> декод,    
// экспорт PCM на родных 7732 Гц (без ресемплинга).    
static void wav_sector(uint8_t bank, uint32_t off_in_file, uint8_t* buffer) {        
    uint32_t len = af_rec_slot_len(bank);        
    uint8_t codec = af_rec_slot_codec(bank);    
    const bool adpcm = (codec == AF_REC_CODEC_ADPCM);    
    uint32_t out_samp = adpcm ? len    
                              : (len * AF_FM_WAV_RATE) / AF_FM_REC_RATE;        
    if (out_samp > AF_FM_WAV_MAX_SAMP)              // cap: не выйти за кластеры    
        out_samp = AF_FM_WAV_MAX_SAMP;              // RECn.WAV на виртуальном диске    
    uint32_t data_bytes = out_samp * 2;        
    const int16_t* pcm = af_rec_slot_data(bank);      
    const uint8_t* nib = (const uint8_t*)pcm;       // для ADPCM-слота    
    uint32_t rate = adpcm ? AF_FM_REC_RATE : AF_FM_WAV_RATE;    
      
    uint8_t h[44];        
    wav_header(h, data_bytes, rate);    // один раз на сектор      
    for (uint32_t i = 0; i < AF_FM_SECTOR_SIZE; i++) {        
        uint32_t p = off_in_file + i;        
        if (p < 44) {        
            buffer[i] = h[p];        
        } else {      
            uint32_t k = (p - 44) / 2;              // индекс выходного отсчёта        
            if (k >= out_samp || pcm == nullptr) { buffer[i] = 0; continue; }        
            int16_t v;      
            if (adpcm) {      
                // Жёсткий guard: читать nib[] только в пределах len нибблов  
                // слота. out_samp == len для ADPCM, но при рассинхроне  
                // (dir/FAT закэшированы хостом, слот перезаписан) чтение  
                // XIP за границей слота -> hardfault -> reboot.  
                if (k >= len) { buffer[i] = 0; continue; }  
                v = fm_adpcm_sample(bank, nib, k, len ? len - 1 : 0);   // 1:1, без ресемплинга      
            } else {    
                // Линейный ресемплинг 7732 -> 8000: src = k * 7732 / 8000      
                uint32_t num = k * AF_FM_REC_RATE;      
                uint32_t si  = num / AF_FM_WAV_RATE;      
                uint32_t fr  = num % AF_FM_WAV_RATE;      
                int32_t a = pcm[si];      
                int32_t b = (si + 1 < len) ? pcm[si + 1] : a;      
                v = (int16_t)(a + ((b - a) * (int32_t)fr) / (int32_t)AF_FM_WAV_RATE);      
            }    
            buffer[i] = (p & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;      
        }      
    }      
}      

// ---------------------------------------------------------------------------    
// Аплоад WAV: парсер заголовка + стриминг-ресемплер в ram_buf    
// ---------------------------------------------------------------------------    
  
static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }    
static uint32_t rd32(const uint8_t* p) {    
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |    
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);    
}    
  
// Сброс сессии аплоада (без записи слота).    
static void ul_abort(const char* why) {      
    if (ul_active) fm_logf("# fm: REC%u.WAV upload aborted: %s",      
                           (unsigned)ul_bank, why);      
    ul_active = false;      
    ul_tail_len = 0;      
}      
  
// Начало сессии: запись в первый сектор файла RECn.WAV.    
static bool ul_begin(uint8_t bank, uint32_t lba) {      
    if (af_rec_is_busy()) {      
        fm_logf("# fm: upload rejected — rec/play busy"); 
        return false;      
    }      
    ul_active   = true;          
    ul_bank     = bank;          
    ul_lba_next = lba;          
    // Аплоад один за раз: stale-кластера других банков не должны    
    // участвовать в best_cl-матчинге — иначе данные угоняются    
    // в чужой слот (баг "REC2/REC4 не перезаписываются").    
    for (uint8_t i = 1; i <= AF_FM_WAV_FILES; i++)    
        if (i != bank) dir_first_cl[i] = 0;    
    fm_logf("# fm: upload begin: REC%u.WAV lba=%lu cl=%u",    
            (unsigned)bank, (unsigned long)lba,    
            (unsigned)dir_first_cl[bank]);  
    ul_last_ms  = millis();    
    ul_file_off = 0;    
    ul_data_end = 0;    
    ul_rate = 0; ul_channels = 0; ul_bits = 0; ul_balign = 0; ul_codec = 1;      
    ul_in_pos = 0; ul_out_pos = 0; ul_prev = 0;      
    ul_enc_pred = 0; ul_enc_idx = 0;   // ADPCM-энкодер с нуля —      
                                       // как dec.pred=0 в af_rec_play      
    ul_trunc = false; ul_tail_len = 0;      
    return true;      
}    
  
// Разбор WAV-заголовка из первых байт файла (пришли в первом секторе).    
// Возвращает смещение начала PCM-данных или 0 при ошибке.    
static uint32_t ul_parse_header(const uint8_t* buf, uint32_t avail) {    
    if (avail < 44) return 0;                       // нужен минимум RIFF+fmt+data    
    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0)    
        return 0;    
    // Ищем чанки fmt и data (между ними могут быть LIST/INFO и т.п.)    
    uint32_t pos = 12;    
    bool have_fmt = false;    
    while (pos + 8 <= avail && pos + 8 <= 512) {    
        uint32_t csz = rd32(buf + pos + 4);    
        if (memcmp(buf + pos, "fmt ", 4) == 0 && csz >= 16) {    
            uint16_t tag = rd16(buf + pos + 8);      
            if (tag != 1 && tag != 6 && tag != 7) return 0;  // PCM/A-law/μ-law  
            ul_codec    = tag;  
            ul_channels = rd16(buf + pos + 10);      
            ul_rate     = rd32(buf + pos + 12);      
            ul_balign   = rd16(buf + pos + 20);      
            ul_bits     = rd16(buf + pos + 22);      
            if (ul_channels < 1 || ul_channels > 2) return 0;      
            if (ul_codec == 1) {  
                if (ul_bits != 8 && ul_bits != 16 && ul_bits != 24 && ul_bits != 32)  
                    return 0;  
            } else {  
                ul_bits = 8;                        // G.711 — всегда 8 бит/сэмпл  
            }  
            if (ul_balign == 0)    
                ul_balign = (uint16_t)(ul_channels * (ul_bits / 8));    
            if (ul_rate < 4000 || ul_rate > 192000) return 0;    
            have_fmt = true;    
        } else if (memcmp(buf + pos, "data", 4) == 0) {    
            if (!have_fmt) return 0;    
            ul_data_end = pos + 8 + csz;            // конец PCM-данных    
            return pos + 8;                         // смещение первого сэмпла    
        }    
        pos += 8 + csz + (csz & 1);                 // чанки выровнены на 2    
    }    
    return 0;                                       // data-чанк не найден в 1-м секторе    
}    

// ДЕКОДЕРЫ

// G.711 A-law -> int16 (стандартное разворачивание).  
static int16_t alaw_decode(uint8_t a) {  
    a ^= 0x55;  
    int32_t t = (a & 0x0F) << 4;  
    uint8_t seg = (a & 0x70) >> 4;  
    if (seg == 0)      t += 8;  
    else if (seg == 1) t += 0x108;  
    else               t = (t + 0x108) << (seg - 1);  
    return (int16_t)((a & 0x80) ? t : -t);  
}  
  
// G.711 μ-law -> int16.  
static int16_t ulaw_decode(uint8_t u) {  
    u = ~u;  
    int32_t t = (((u & 0x0F) << 3) + 0x84) << ((u & 0x70) >> 4);  
    return (int16_t)((u & 0x80) ? (0x84 - t) : (t - 0x84));  
}  

// Извлечение одного входного отсчёта (mono = усреднение каналов) -> int16.    
static int16_t ul_sample(const uint8_t* p) {      
    int32_t sum = 0;      
    for (uint16_t c = 0; c < ul_channels; c++) {      
        const uint8_t* s = p + c * (ul_bits / 8);      
        int32_t v;      
        if (ul_codec == 6)      v = alaw_decode(s[0]);  
        else if (ul_codec == 7) v = ulaw_decode(s[0]);  
        else switch (ul_bits) {      
            case 8:  v = ((int32_t)s[0] - 128) << 8; break;    
            case 16: v = (int16_t)rd16(s); break;    
            case 24: v = ((int32_t)s[0] | ((int32_t)s[1] << 8) |    
                          ((int32_t)(int8_t)s[2] << 16)) >> 8; break;    
            default: v = (int32_t)rd32(s) >> 16; break;   // 32 бит    
        }    
        sum += v;    
    }    
    int32_t r = sum / ul_channels;    
    if (r > 32767) r = 32767;    
    if (r < -32768) r = -32768;    
    return (int16_t)r;    
}    
  
// Ресемплер: входной отсчёт -> выходные отсчёты в ram_buf (линейная интерполяция).    
// Шаг: in_pos продвигается на ul_rate за каждый выходной отсчёт 7732 Гц.    
static void ul_push(int16_t cur) {      
    if (ul_out_pos >= AF_REC_RAM_SAMP) { ul_trunc = true; return; }  
    // Интерполяция между ul_prev (in_pos-1) и cur (in_pos):    
    // окно [cur_in, cur_in + REC_RATE) соответствует входному интервалу.    
    uint64_t cur_in  = (ul_in_pos > 0) ? (uint64_t)(ul_in_pos - 1) * AF_FM_REC_RATE : 0;    
    uint64_t next_in = (uint64_t)ul_out_pos * ul_rate;    
    uint8_t* rb = (uint8_t*)af_rec_ram_buf();   // ADPCM-поток: 2 отсчёта/байт    
    while (next_in < cur_in + AF_FM_REC_RATE) {    
        uint32_t fr = (uint32_t)(next_in - cur_in);    
        int32_t v = ul_prev + (int32_t)(((int64_t)(cur - ul_prev) * fr) / AF_FM_REC_RATE);    
        uint8_t nib = ul_adpcm_enc((int16_t)v);    
        if (ul_out_pos & 1) rb[ul_out_pos >> 1] |= (uint8_t)(nib << 4);    
        else                rb[ul_out_pos >> 1]  = nib;   // чётный — младший ниббл    
        ul_out_pos++;    
        if (ul_out_pos >= AF_REC_RAM_SAMP) { ul_trunc = true; break; }   
        next_in = (uint64_t)ul_out_pos * ul_rate;    
    }    
    ul_prev = cur;    
    ul_in_pos++;    
}    
  
// Обработка одного записанного сектора WAV-файла.    
static void ul_feed(const uint8_t* buf) {    
    ul_last_ms = millis();    
    uint32_t pos = 0;    
  
    if (ul_file_off == 0) {      
        uint32_t data_off = ul_parse_header(buf, AF_FM_SECTOR_SIZE);      
        if (data_off == 0) {    
            fm_logf("# fm: REC%u.WAV bad header: %02x %02x %02x %02x",    
                    (unsigned)ul_bank, buf[0], buf[1], buf[2], buf[3]);    
            ul_abort("bad WAV header");    
            return;    
        }        
        fm_logf("# fm: REC%u.WAV upload: %lu Hz %u ch %u bit codec=%u",        
                (unsigned)ul_bank, (unsigned long)ul_rate,        
                (unsigned)ul_channels, (unsigned)ul_bits, (unsigned)ul_codec);        
        pos = data_off;    
    }    
  
    // Склеиваем хвост прошлого сектора с началом текущего    
    while (pos < AF_FM_SECTOR_SIZE && !ul_trunc) {    
        uint32_t file_pos = ul_file_off + pos;    
        if (file_pos >= ul_data_end) break;             // за пределами data-чанка    
        // Собираем полный фрейм (ul_balign байт)    
        uint8_t frame[8];                               // max 2ch * 32bit = 8    
        uint8_t need = (uint8_t)ul_balign;    
        uint8_t got = 0;    
        if (ul_tail_len) {    
            memcpy(frame, ul_tail, ul_tail_len);    
            got = ul_tail_len;    
            ul_tail_len = 0;    
        }    
        uint32_t room = AF_FM_SECTOR_SIZE - pos;    
        uint32_t take = (need - got < room) ? (need - got) : room;    
        // Не выходить за data_end    
        if (file_pos + take > ul_data_end) take = ul_data_end - file_pos;    
        memcpy(frame + got, buf + pos, take);    
        got += (uint8_t)take;    
        pos += take;    
        if (got < need) {                               // фрейм разрезан — в хвост    
            memcpy(ul_tail, frame, got);    
            ul_tail_len = got;    
            break;    
        }    
        ul_push(ul_sample(frame));    
    }    
    ul_file_off += AF_FM_SECTOR_SIZE;    
}    
  
// Коммит: таймаут после последнего сектора -> запись слота во flash.    
static void ul_commit(void) {    
    uint8_t bank = ul_bank;    
    uint32_t n = ul_out_pos;    
    ul_active = false;    
    if (n == 0) {      
        fm_logf("# fm: REC%u.WAV upload: no samples", (unsigned)bank);      
        return;      
    }      
    fm_logf("# fm: REC%u.WAV -> slot %u, %lu samples%s",        
            (unsigned)bank, (unsigned)bank, (unsigned long)n,        
            ul_trunc ? " (truncated)" : "");        
    // ram_buf уже содержит ADPCM-поток (инкрементальный encode в ul_push) —      
    // batch-encode не нужен.      
    if (af_rec_slot_write(bank, n, AF_REC_CODEC_ADPCM)) {            
        af_fm_request_sync();      
        fm_sizes_changed = true;   // аплоад — всегда re-enumerate (содержимое новое)      
        dir_first_cl[bank] = 0;    // кластер отработан — не оставлять stale    
    } else {         
        fm_logf("# fm: slot write failed");
    }  
}    

int32_t af_fm_msc_read(uint32_t lba, void* buffer, uint32_t bufsize) {    
    if (buffer == nullptr || bufsize == 0 || lba >= AF_FM_SECTOR_COUNT)    
        return -1;    
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;    
    if (lba == 0) {                                // boot      
        memcpy(buffer, fm_disk[0], AF_FM_SECTOR_SIZE);      
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    if (lba <= AF_FM_FAT_SECTORS) {                // FAT 1..80 — синтез    
        fat_sector(lba, (uint8_t*)buffer);    
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    if (lba == AF_FM_DIR_LBA) {                    // root dir    
        memcpy(buffer, fm_disk[AF_FM_DIR_RAM], AF_FM_SECTOR_SIZE);      
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    if (lba >= AF_FM_SET_LBA && lba < AF_FM_SET_LBA + AF_FM_SET_CLUSTERS) {    
        memcpy(buffer, fm_disk[AF_FM_SET_RAM + (lba - AF_FM_SET_LBA)],    
               AF_FM_SECTOR_SIZE);      
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    if (lba == AF_FM_CW_LBA) {                     // CW.TXT    
        memcpy(buffer, fm_disk[AF_FM_CW_RAM], AF_FM_SECTOR_SIZE);      
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    // WAV-область: определяем файл по диапазону LBA.    
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        uint32_t start = AF_FM_WAV_LBA(i);    
        if (lba >= start && lba < start + AF_FM_WAV_CLUSTERS) {    
            wav_sector(i + 1, (lba - start) * AF_FM_SECTOR_SIZE, (uint8_t*)buffer);    
            return (int32_t)AF_FM_SECTOR_SIZE;    
        }    
    }    
    memset(buffer, 0, AF_FM_SECTOR_SIZE);          // зазор между файлами    
    return (int32_t)AF_FM_SECTOR_SIZE;    
}    
  
void af_fm_msc_flush(void) {      
    if (fm_dirty)    fm_dirty_ms = millis();    
    if (fm_cw_dirty) fm_cw_ms    = millis();    
}      
  
int32_t af_fm_msc_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {          
    if (buffer == nullptr || lba >= AF_FM_SECTOR_COUNT) return -1;          
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;          
  
    // WAV-область: перезапись RECn.WAV = аплоад в слот n.    
    // Всегда возвращаем OK — ошибка провоцирует бесконечные ретраи хоста.    
    if (lba >= AF_FM_WAV_LBA(0)) {        
        // Свободный пул: данные нового файла. Банк — по первому кластеру  
        // из dir-записи (dir_track), а не по LBA.  
        uint8_t bank = 0;  
        if (lba >= AF_FM_FREE_LBA) {  
            uint16_t cl = (uint16_t)(lba - AF_FM_DATA_START + 2);   // LBA -> кластер    
            // Ближайший первый кластер <= cl: break по первому dir_first_cl[i]    
            // с cl >= dir_first_cl[i] обрывал поиск, если запись i была stale    
            // (кластер из прошлой сессии) — отсюда "no dir match" при cl=1610.    
            uint16_t best_cl = 0;    
            for (uint8_t i = 1; i <= AF_FM_WAV_FILES; i++) {    
                if (dir_first_cl[i] && dir_first_cl[i] <= cl &&    
                    dir_first_cl[i] > best_cl) {    
                    best_cl = dir_first_cl[i];    
                    bank = i;    
                }    
            }    
            if (bank == 0) {  
                fm_logf("# fm: pool write lba=%lu cl=%u — no dir match",
                              (unsigned long)lba, (unsigned)cl);  
                return (int32_t)AF_FM_SECTOR_SIZE;    // пул без имени — проглотить  
            }  
        } else {  
            for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {        
                uint32_t start = AF_FM_WAV_LBA(i);        
                if (lba >= start && lba < start + AF_FM_WAV_CLUSTERS) {        
                    bank = i + 1;  
                    break;  
                }  
            }  
            if (bank == 0) {  
                fm_logf("# fm: gap write lba=%lu — swallowed",
                              (unsigned long)lba);  
                return (int32_t)AF_FM_SECTOR_SIZE;    // зазор — проглотить  
            }  
        }  
        if (!ul_active) {        
            if (!ul_begin(bank, lba))    
                return (int32_t)AF_FM_SECTOR_SIZE;   // rec/play busy — проглотить    
        } else if (ul_bank != bank) {  
            fm_logf("# fm: write to REC%u while REC%u active — swallowed",
                          (unsigned)bank, (unsigned)ul_bank);  
            return (int32_t)AF_FM_SECTOR_SIZE;       // чужой файл — проглотить  
        }  
        if (lba != ul_lba_next) {             // только последовательно        
            fm_logf("# fm: REC%u.WAV non-seq LBA %lu (expect %lu)",
                          (unsigned)ul_bank, (unsigned long)lba,    
                          (unsigned long)ul_lba_next);    
            ul_abort("non-sequential LBA");        
            return (int32_t)AF_FM_SECTOR_SIZE;        
        }        
        ul_feed(buffer);        
        ul_lba_next = lba + 1;        
        return (int32_t)AF_FM_SECTOR_SIZE;        
    }        
    
    // FAT (LBA 1..AF_FM_FAT_SECTORS) — read-only для хоста: наша FAT    
    // синтезируется и описывает виртуальную раскладку RECn.WAV, а хост    
    // при записи файла отдаёт свою версию и затирает цепочки.    
    // Проглатываем с OK: отказ провоцирует ретраи и remount.    
    if (lba >= 1 && lba <= AF_FM_FAT_SECTORS)    
        return (int32_t)AF_FM_SECTOR_SIZE;    
    
    // Маппинг LBA -> RAM-образ: dir=1, SET=2..17, CW=18.    
    // Прочие LBA (зазоры между RAM-областью и WAV) — read-only.    
    uint32_t ram;    
    if (lba == AF_FM_DIR_LBA)                                          ram = AF_FM_DIR_RAM;    
    else if (lba >= AF_FM_SET_LBA &&    
             lba < AF_FM_SET_LBA + AF_FM_SET_CLUSTERS)                 ram = AF_FM_SET_RAM + (lba - AF_FM_SET_LBA);    
    else if (lba == AF_FM_CW_LBA)                                      ram = AF_FM_CW_RAM;    
    else {      
        // Зазор между RAM-областью и WAV — проглотить с OK:      
        // отказ провоцирует ретраи хоста и remount.      
        fm_logf("# fm: write swallowed lba=%lu (gap)", (unsigned long)lba);      
        return (int32_t)AF_FM_SECTOR_SIZE;      
    }      
    memcpy(fm_disk[ram], buffer, AF_FM_SECTOR_SIZE);                    
    // Перехват root dir: чужие записи (SVI и т.п.) помечаем удалёнными,    
    // затем хост создал/обновил RECn.WAV — запомнить кластер.        
    if (lba == AF_FM_DIR_LBA) {    
        dir_fix_foreign(fm_disk[ram]);    
        dir_track(fm_disk[ram]);    
    }    

    // Парсер запускаем только при записи в data-область SET.TXT (LBA 82..97)  
    // Записи хоста в boot/FAT/dir при монтировании (метки тома, индексы)    
    // не должны триггерить парсинг и перемонтирование — иначе цикл.    
    if (lba >= AF_FM_SET_LBA && lba < AF_FM_SET_LBA + AF_FM_SET_CLUSTERS) {      
        fm_dirty   = true;          
        fm_dirty_ms = millis();      // дебаунс -> парсер в af_fm_poll          
    }      
    if (lba >= AF_FM_CW_LBA && lba < AF_FM_CW_LBA + AF_FM_CW_CLUSTERS) {    
        fm_cw_dirty = true;    
        fm_cw_ms    = millis();      // дебаунс -> cw_txt_parse в af_fm_poll    
    }    
    return (int32_t)AF_FM_SECTOR_SIZE;                    
}    
