// src/af_fm.cpp  
//  
// FAT12-диск ~1.6 МБ, RAM-образ ~14 КБ (метаданные + SET.TXT). Раскладка:    
//   LBA 0      boot sector    
//   LBA 1..10  FAT (10 секторов, кластер = 1 сектор)    
//   LBA 11     root dir (SET.TXT + CW.TXT + REC1..REC4.WAV)        
//   LBA 12..27 data: SET.TXT (RAM)        
//   LBA 28     data: CW.TXT (RAM, 4 строки сообщений магнитофона)      
//   LBA 29..   REC1..REC4.WAV — виртуальные, синтез в af_fm_msc_read        
//  
// SET.TXT генерируется af_cli_format_banks() — тот же текст, что 'banks'.      
// Запись с ПК: дебаунс 800 мс -> af_cli_apply_set_txt -> банки во flash.      
//
// CW.TXT — 4 строки вызовов (af_cw_msgs, af_store.cpp). Пустая строка =      
// голосовой слот. Запись с ПК: дебаунс 800 мс -> парсер -> af_store_cw_save.    
//      
// Аплоад: перезапись RECn.WAV -> стриминг-конвертер -> ram_buf (PCM-стейджинг    
// ~7.2 с) -> encode ADPCM -> слот n. Любой WAV (PCM 8/16/24/32 бит,    
// mono/stereo, любая частота): ресемплинг в 7732 Гц mono int16. Коммит —    
// по таймауту 1.5 с после последнего сектора (flash-окно в af_rec_slot_write).    
// Экспорт: codec=0 — PCM WAV 8000 Гц (ресемплинг), codec=1 — декод ADPCM    
// в PCM WAV 7732 Гц (родная частота, без ресемплинга).    
  
#include <Arduino.h>  
#include <string.h>  
#include <stdio.h>  

#include "af_config.h"  
#include "af_fm.h"
#include "af_cli.h"
#include "af_usb.h"          // af_usb_reenumerate — перемонтирование диска
#include "af_store.h"        // af_cw_msgs, af_store_cw_save — CW.TXT 

// Рендер таблицы banks в буфер (af_cli.cpp — там AF_PARAMS/bank_field).    
// Возвращает длину текста без NUL.    
extern size_t af_cli_format_banks(char* buf, size_t cap);    
  
#include "af_rec.h"          // af_rec_slot_len/data — WAV-экспорт    
    
static uint8_t  fm_disk[AF_FM_RAM_SECTORS][AF_FM_SECTOR_SIZE];  // ~14 КБ    
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
        if (p[0] == 0xE5 || p[0] == 0x00) continue;  // удалённая/пустая запись    
        for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
            if (memcmp(p, wav_names[i], 11) == 0) {    
                uint16_t cl = rd16(p + 26);  
                if (cl != dir_first_cl[i + 1]) {  
                    dir_first_cl[i + 1] = cl;  
                    Serial.printf("# fm: dir REC%u.WAV -> cluster %u\n",  
                                  (unsigned)(i + 1), (unsigned)cl);  
                }  
            }  
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
    const uint16_t ts16 = (uint16_t)AF_FM_SECTOR_COUNT;   // ~3199 < 65536    
    memcpy(&b[19], &ts16, 2);      
    b[21] = 0xF8;                        // media      
    const uint16_t fat_sz = (uint16_t)AF_FM_FAT_SECTORS;      
    memcpy(&b[22], &fat_sz, 2);          // sectors/FAT = 10    
    b[24] = 1;  b[25] = 0;               // spt  
    b[26] = 1;  b[27] = 0;               // heads  
    b[36] = 0x80; b[38] = 0x29;  
    const uint32_t vid = 0xA0D5F000;  
    memcpy(&b[39], &vid, 4);  
    memcpy(&b[43], "AUDIO_DSP  ", 11);  
    memcpy(&b[54], "FAT12   ", 8);  
    b[510] = 0x55; b[511] = 0xAA;  
}  
  
// FAT12: запись 12-битного значения в элемент n.  
static void fat12_set(uint8_t* fat, uint16_t n, uint16_t v) {  
    uint32_t off = n + n / 2;  
    if (n & 1) {  
        fat[off]     = (fat[off] & 0x0F) | ((v << 4) & 0xF0);  
        fat[off + 1] = (uint8_t)(v >> 4);  
    } else {  
        fat[off]     = (uint8_t)v;  
        fat[off + 1] = (fat[off + 1] & 0xF0) | ((v >> 8) & 0x0F);  
    }  
}  
  
// Цепочка из n кластеров начиная с first; FAT лежит в fm_disk[1..10].    
static void fat_chain(uint8_t* f, uint16_t first, uint16_t n) {    
    for (uint16_t c = 0; c < n; c++)    
        fat12_set(f, first + c, (c == n - 1) ? 0xFFF : (uint16_t)(first + c + 1));    
}    
    
// Полная перестройка FAT: SET.TXT (set_clusters) + 4 WAV по их размерам.      
static void fm_build_fat(uint16_t set_clusters, const uint16_t* wav_clusters) {    
    uint8_t* f = &fm_disk[1][0];    
    memset(f, 0, AF_FM_FAT_SECTORS * AF_FM_SECTOR_SIZE);    
    f[0] = 0xF8; f[1] = 0xFF; f[2] = 0xFF;    
    fat_chain(f, 2, AF_FM_SET_CLUSTERS);   // вся территория SET.TXT занята —    
                                           // хост не выделит кластеры 3..17    
                                           // новому файлу (инцидент с REC1)    
    fat_chain(f, 2 + AF_FM_SET_CLUSTERS, AF_FM_CW_CLUSTERS);  // CW.TXT — кластер 18    
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        uint16_t first = (uint16_t)(2 + AF_FM_SET_CLUSTERS + AF_FM_CW_CLUSTERS    
                                    + i * AF_FM_WAV_CLUSTERS);    
        fat_chain(f, first, AF_FM_WAV_CLUSTERS);  
    }  
    // Свободный пул в конце диска: кластеры за пределами цепочек RECn  
    // остаются неразмеченными -> Windows видит свободное место и  
    // пропускает проверку при копировании/замене файлов.  
    // (ничего дополнительно делать не нужно — неразмеченные = свободные)  
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
    uint8_t* d = fm_disk[AF_FM_DIR_LBA];    
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
    static const uint16_t zero4[AF_FM_WAV_FILES] = {0};      
    static const uint32_t zsz4[AF_FM_WAV_FILES]  = {0};      
    fm_build_fat(0, zero4);      
    fm_build_dir(0, 0, zsz4);      
    fm_set_len = 0;  
    fm_dirty   = false;  
    fm_stale   = true;                 // первый sync — после af_store_init  
}  
  
void af_fm_request_sync(void) { fm_stale = true; }  
  
void af_fm_sync(void) {    
    // SET.TXT рендерится в RAM-область (LBA 12..27).    
    char* txt = (char*)&fm_disk[AF_FM_SET_LBA][0];    
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
        // Re-enumerate только при появлении файла (пусто -> непусто):    
        // заливка voice14, первая запись в слот. Изменение размера    
        // существующего файла — без перемонтирования.    
        if (!first_sync && prev_sizes[i] == 44 && wav_sizes[i] != 44)      
            fm_sizes_changed = true;      
        prev_sizes[i] = wav_sizes[i];    
    }        
    first_sync = false;    
    // CW.TXT: 4 строки из af_cw_msgs, CRLF, хвост — пробелы (чистый вид    
    // в редакторе; парсер пробелы/CRLF пропускает).    
    {    
        uint8_t* cw = &fm_disk[AF_FM_CW_LBA][0];    
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
    fm_build_fat(set_clusters, wav_clusters);        
    fm_build_dir(fm_set_len, set_clusters, wav_sizes);        
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
    // Коммит аплоада WAV: 1.5 с тишины после последнего сектора.    
    if (ul_active && (int32_t)(millis() - ul_last_ms) >= 1500)    
        ul_commit();    
    // Дебаунс: хост пишет файл пачкой секторов — ждём 800 мс тишины,      
    // потом парсим SET.TXT целиком и пишем банки во flash.      
    if (fm_dirty && (int32_t)(millis() - fm_dirty_ms) >= 800) {        
        fm_dirty = false;      
        const char* txt = (const char*)&fm_disk[AF_FM_SET_LBA][0];        
        bool applied = af_cli_apply_set_txt(txt,        
                          AF_FM_SET_CLUSTERS * AF_FM_SECTOR_SIZE);        
        Serial.printf("# fm: SET.TXT %s\n",        
                      applied ? "parsed -> banks written" : "parse failed");        
        fm_stale = true;               // перерендерить из записанных банков           
    }        
    // CW.TXT: тот же дебаунс 800 мс -> парсер -> flash -> re-render.    
    if (fm_cw_dirty && (int32_t)(millis() - fm_cw_ms) >= 800) {    
        fm_cw_dirty = false;    
        cw_txt_parse((const char*)&fm_disk[AF_FM_CW_LBA][0],    
                     AF_FM_CW_CLUSTERS * AF_FM_SECTOR_SIZE);    
        af_store_cw_save();            // flash-окно + af_fm_request_sync    
        Serial.println("# fm: CW.TXT parsed -> msgs written");    
    }    
    if (fm_stale) af_fm_sync();            
    // Пере-энумерация только при появлении файла (пусто -> непусто:    
    // заливка voice14, первая запись) и после аплоада с ПК.    
    // Обычный старт и перезапись существующего слота — без мигания.    
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
// назад — сброс и декод с нуля (до ~229К отсчётов, миллисекунды).    
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
static int16_t fm_dec_pred = 0;    
static int8_t  fm_dec_idx  = 0;    
static uint32_t fm_dec_pos = 0;    // следующий отсчёт, который выдаст декодер    
static uint8_t  fm_dec_bank = 0;   // банк, для которого валиден кэш    
    
static int16_t fm_adpcm_dec(uint8_t nib) {    
    int32_t step = fm_adpcm_step[fm_dec_idx];    
    int32_t delta = step >> 3;    
    if (nib & 4) delta += step;    
    if (nib & 2) delta += step >> 1;    
    if (nib & 1) delta += step >> 2;    
    fm_dec_pred += (nib & 8) ? -delta : delta;    
    if (fm_dec_pred > 32767) fm_dec_pred = 32767;    
    else if (fm_dec_pred < -32768) fm_dec_pred = -32768;    
    fm_dec_idx += fm_adpcm_idx[nib & 15];    
    if (fm_dec_idx < 0) fm_dec_idx = 0; else if (fm_dec_idx > 88) fm_dec_idx = 88;    
    return (int16_t)fm_dec_pred;    
}    
    
// Отсчёт k ADPCM-слота: последовательный декод с кэшем позиции.    
static int16_t fm_adpcm_sample(uint8_t bank, const uint8_t* data, uint32_t k) {    
    if (bank != fm_dec_bank || k < fm_dec_pos) {   // другой слот/скачок — с нуля    
        fm_dec_pred = 0; fm_dec_idx = 0; fm_dec_pos = 0; fm_dec_bank = bank;    
    }    
    int16_t v = fm_dec_pred;    
    while (fm_dec_pos <= k) {    
        uint8_t b = data[fm_dec_pos >> 1];    
        v = fm_adpcm_dec((fm_dec_pos & 1) ? (b >> 4) : (b & 0x0F));    
        fm_dec_pos++;    
    }    
    return v;    
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
                v = fm_adpcm_sample(bank, nib, k);  // 1:1, без ресемплинга    
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
    if (ul_active) Serial.printf("# fm: REC%u.WAV upload aborted: %s\n",    
                                 (unsigned)ul_bank, why);    
    ul_active = false;    
    ul_tail_len = 0;    
}    
  
// Начало сессии: запись в первый сектор файла RECn.WAV.    
static bool ul_begin(uint8_t bank, uint32_t lba) {      
    if (af_rec_is_busy()) {      
        Serial.println("# fm: upload rejected — rec/play busy");      
        return false;      
    }      
    ul_active   = true;      
    ul_bank     = bank;      
    ul_lba_next = lba;      
    Serial.printf("# fm: upload begin: REC%u.WAV lba=%lu (file start %lu)\n",  
                  (unsigned)bank, (unsigned long)lba,  
                  (unsigned long)AF_FM_WAV_LBA(bank - 1));
    Serial.printf("# fm: dir_first_cl[] =");  
    for (uint8_t i = 1; i <= AF_FM_WAV_FILES; i++)  
        Serial.printf(" %u", (unsigned)dir_first_cl[i]);  
    Serial.println();  
    ul_last_ms  = millis();    
    ul_file_off = 0;    
    ul_data_end = 0;    
    ul_rate = 0; ul_channels = 0; ul_bits = 0; ul_balign = 0; ul_codec = 1;      
    ul_in_pos = 0; ul_out_pos = 0; ul_prev = 0;    
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
    if (ul_out_pos >= AF_REC_UL_MAX_SAMP) { ul_trunc = true; return; }
    // Интерполяция между ul_prev (in_pos-1) и cur (in_pos):  
    // окно [cur_in, cur_in + REC_RATE) соответствует входному интервалу.  
    uint64_t cur_in  = (ul_in_pos > 0) ? (uint64_t)(ul_in_pos - 1) * AF_FM_REC_RATE : 0;  
    uint64_t next_in = (uint64_t)ul_out_pos * ul_rate;  
    while (next_in < cur_in + AF_FM_REC_RATE) {  
        uint32_t fr = (uint32_t)(next_in - cur_in);  
        int32_t v = ul_prev + (int32_t)(((int64_t)(cur - ul_prev) * fr) / AF_FM_REC_RATE);  
        af_rec_ram_buf()[ul_out_pos++] = (int16_t)v;  
        if (ul_out_pos >= AF_REC_UL_MAX_SAMP) { ul_trunc = true; break; } 
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
            Serial.printf("# fm: REC%u.WAV bad header, first 32 bytes:\n# ",  
                          (unsigned)ul_bank);  
            for (uint8_t i = 0; i < 32; i++)  
                Serial.printf("%02x ", buf[i]);  
            Serial.printf("\n# ascii: ");  
            for (uint8_t i = 0; i < 32; i++)  
                Serial.printf("%c", (buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');  
            Serial.println();  
            ul_abort("bad WAV header");  
            return;  
        }      
        Serial.printf("# fm: REC%u.WAV upload: %lu Hz %u ch %u bit codec=%u\n",      
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
        Serial.printf("# fm: REC%u.WAV upload: no samples\n", (unsigned)bank);    
        return;    
    }    
    Serial.printf("# fm: REC%u.WAV -> slot %u, %lu samples%s\n",      
                  (unsigned)bank, (unsigned)bank, (unsigned long)n,      
                  ul_trunc ? " (truncated)" : "");      
    af_rec_adpcm_encode_buf(n);          // PCM -> ADPCM in-place в ram_buf    
    if (af_rec_slot_write(bank, n, AF_REC_CODEC_ADPCM)) {          
        af_fm_request_sync();      
        fm_sizes_changed = true;   // аплоад — всегда re-enumerate (содержимое новое)      
        dir_first_cl[bank] = 0;    // кластер отработан — не оставлять stale    
    } else {         
        Serial.println("# fm: slot write failed");  
    }  
}    

int32_t af_fm_msc_read(uint32_t lba, void* buffer, uint32_t bufsize) {    
    if (buffer == nullptr || bufsize == 0 || lba >= AF_FM_SECTOR_COUNT)    
        return -1;    
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;    
    if (lba < AF_FM_RAM_SECTORS) {                 // boot/FAT/dir/SET.TXT    
        memcpy(buffer, fm_disk[lba], AF_FM_SECTOR_SIZE);    
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
                Serial.printf("# fm: pool write lba=%lu cl=%u — no dir match\n",  
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
                Serial.printf("# fm: gap write lba=%lu — swallowed\n",  
                              (unsigned long)lba);  
                return (int32_t)AF_FM_SECTOR_SIZE;    // зазор — проглотить  
            }  
        }  
        if (!ul_active) {        
            if (!ul_begin(bank, lba))    
                return (int32_t)AF_FM_SECTOR_SIZE;   // rec/play busy — проглотить    
        } else if (ul_bank != bank) {  
            Serial.printf("# fm: write to REC%u while REC%u active — swallowed\n",  
                          (unsigned)bank, (unsigned)ul_bank);  
            return (int32_t)AF_FM_SECTOR_SIZE;       // чужой файл — проглотить  
        }  
        if (lba != ul_lba_next) {             // только последовательно        
            Serial.printf("# fm: REC%u.WAV non-seq LBA %lu (expect %lu)\n",    
                          (unsigned)ul_bank, (unsigned long)lba,    
                          (unsigned long)ul_lba_next);    
            ul_abort("non-sequential LBA");        
            return (int32_t)AF_FM_SECTOR_SIZE;        
        }        
        ul_feed(buffer);        
        ul_lba_next = lba + 1;        
        return (int32_t)AF_FM_SECTOR_SIZE;        
    }        
    
    if (lba >= AF_FM_RAM_SECTORS) {  
        Serial.printf("# fm: write rejected lba=%lu (>= RAM_SECTORS %lu)\n",  
                      (unsigned long)lba, (unsigned long)AF_FM_RAM_SECTORS);  
        return -1;                                  // зазоры — read-only  
    }  
    memcpy(fm_disk[lba], buffer, AF_FM_SECTOR_SIZE);              
    // Перехват root dir: хост создал/обновил RECn.WAV — запомнить кластер.  
    if (lba == AF_FM_DIR_LBA) dir_track(fm_disk[lba]);  

    // Парсер запускаем только при записи в data-область SET.TXT (LBA 12..27).    
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
