// src/af_fm.cpp  
//  
// FAT12-диск ~1.6 МБ, RAM-образ ~14 КБ (метаданные + SET.TXT). Раскладка:    
//   LBA 0      boot sector    
//   LBA 1..10  FAT (10 секторов, кластер = 1 сектор)    
//   LBA 11     root dir (SET.TXT + REC1..REC7.WAV)    
//   LBA 12..27 data: SET.TXT (RAM)    
//   LBA 28..   REC1..REC7.WAV — виртуальные, синтез в af_fm_msc_read    
//              (WAV 16000 Гц mono, ресемплинг из слота 7732 Гц)    
//  
// SET.TXT генерируется af_cli_format_banks() — тот же текст, что 'banks'.    
// Запись с ПК: дебаунс 800 мс -> af_cli_apply_set_txt -> банки во flash.    
  
#include <Arduino.h>  
#include <string.h>  
#include <stdio.h>  

#include "af_config.h"  
#include "af_fm.h"
#include "af_cli.h"
#include "af_usb.h"          // af_usb_reenumerate — перемонтирование диска
 
// Рендер таблицы banks в буфер (af_cli.cpp — там AF_PARAMS/bank_field).    
// Возвращает длину текста без NUL.    
extern size_t af_cli_format_banks(char* buf, size_t cap);    
  
#include "af_rec.h"          // af_rec_slot_len/data — WAV-экспорт    
    
static uint8_t  fm_disk[AF_FM_RAM_SECTORS][AF_FM_SECTOR_SIZE];  // ~14 КБ    
static uint32_t fm_set_len = 0;        // байт в SET.TXT  
static bool     fm_dirty   = false;    // ПК писал — ждёт парсера    
static bool     fm_stale   = true;     // SET.TXT устарел — регенерировать    
static uint32_t fm_dirty_ms = 0;       // millis() последней записи (дебаунс)    
  
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
    
// Полная перестройка FAT: SET.TXT (set_clusters) + 7 WAV по их размерам.    
static void fm_build_fat(uint16_t set_clusters, const uint16_t* wav_clusters) {    
    uint8_t* f = &fm_disk[1][0];    
    memset(f, 0, AF_FM_FAT_SECTORS * AF_FM_SECTOR_SIZE);    
    f[0] = 0xF8; f[1] = 0xFF; f[2] = 0xFF;    
    if (set_clusters) fat_chain(f, 2, set_clusters);    
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        uint16_t first = (uint16_t)(2 + AF_FM_SET_CLUSTERS + i * AF_FM_WAV_CLUSTERS);    
        if (wav_clusters[i]) fat_chain(f, first, wav_clusters[i]);    
    }    
}    
  
static void dir_entry(uint8_t* e, const char* name83, uint16_t cl, uint32_t size) {    
    memset(e, 0, 32);    
    memcpy(e, name83, 11);    
    e[11] = 0x20;                                // archive    
    memcpy(e + 26, &cl, 2);    
    memcpy(e + 28, &size, 4);    
}    
    
// Каталог: SET.TXT + REC1..REC7.WAV. wav_sizes[i] = байт в файле (44 = пусто).    
static void fm_build_dir(uint32_t set_size, uint16_t set_clusters,    
                         const uint32_t* wav_sizes) {    
    uint8_t* d = fm_disk[AF_FM_DIR_LBA];    
    memset(d, 0, AF_FM_SECTOR_SIZE);    
    dir_entry(&d[0], "SET     TXT", set_clusters ? 2 : 0, set_size);    
    static const char wav_names[AF_FM_WAV_FILES][12] = {  
        "REC1    WAV", "REC2    WAV", "REC3    WAV", "REC4    WAV",  
        "REC5    WAV", "REC6    WAV", "REC7    WAV" };   
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        uint16_t first = (uint16_t)(2 + AF_FM_SET_CLUSTERS + i * AF_FM_WAV_CLUSTERS);    
        dir_entry(&d[32 * (i + 1)], wav_names[i], wav_sizes[i] ? first : 0, wav_sizes[i]);  
    }    
}    
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
  
void af_fm_init(void) {  
    memset(fm_disk, 0, sizeof(fm_disk));  
    fm_build_boot();    
    static const uint16_t zero7[7] = {0};    
    static const uint32_t zsz7[7]  = {0};    
    fm_build_fat(0, zero7);    
    fm_build_dir(0, 0, zsz7);    
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
    for (uint8_t i = 0; i < AF_FM_WAV_FILES; i++) {    
        uint32_t len = af_rec_slot_len(i + 1);    
        uint32_t out_samp = (len * AF_FM_WAV_RATE) / AF_FM_REC_RATE;    
        wav_sizes[i] = 44 + out_samp * 2;    
        wav_clusters[i] = (uint16_t)((wav_sizes[i] + 511) / 512);    
    }    
    fm_build_fat(set_clusters, wav_clusters);    
    fm_build_dir(fm_set_len, set_clusters, wav_sizes);    
    fm_stale = false;    
}    
  
void af_fm_poll(void) {    
    // Дебаунс: хост пишет файл пачкой секторов — ждём 800 мс тишины,    
    // потом парсим SET.TXT целиком и пишем банки во flash.    
    if (fm_dirty && (int32_t)(millis() - fm_dirty_ms) >= 800) {      
        fm_dirty = false;      
        const char* txt = (const char*)&fm_disk[AF_FM_SET_LBA][0];        
        bool applied = af_cli_apply_set_txt(txt,        
                          AF_FM_SET_CLUSTERS * AF_FM_SECTOR_SIZE);        
        Serial.printf("# fm: SET.TXT %s\n",        
                      applied ? "parsed -> banks written" : "parse failed");        
        if (applied) {      
            af_fm_sync();              // перерендерить из записанных банков      
            af_usb_reenumerate();      // хост перечитает образ — иначе      
                                       // его кеш перезапишет наши правки      
        } else {      
            fm_stale = true;           // парсинг не удался — вернуть образ      
        }      
    }      
    if (fm_stale) af_fm_sync();      
}    
  
bool     af_fm_dirty(void)    { return fm_dirty; }  
uint32_t af_fm_set_size(void) { return fm_set_len; }  
  
// ---------------------------------------------------------------------------  
// MSC-колбэки  
// ---------------------------------------------------------------------------  
  
// WAV-заголовок 44 Б: PCM mono 16-bit, rate = AF_FM_WAV_RATE.    
static void wav_header(uint8_t* h, uint32_t data_bytes) {    
    memset(h, 0, 44);    
    memcpy(h + 0, "RIFF", 4);    
    uint32_t riff = 36 + data_bytes;   memcpy(h + 4, &riff, 4);    
    memcpy(h + 8, "WAVEfmt ", 8);    
    uint32_t fmt = 16;                 memcpy(h + 16, &fmt, 4);    
    uint16_t pcm = 1, ch = 1, bits = 16, align = 2;    
    uint32_t rate = AF_FM_WAV_RATE, brate = AF_FM_WAV_RATE * 2;    
    memcpy(h + 20, &pcm, 2);   memcpy(h + 22, &ch, 2);    
    memcpy(h + 24, &rate, 4);  memcpy(h + 28, &brate, 4);    
    memcpy(h + 32, &align, 2); memcpy(h + 34, &bits, 2);    
    memcpy(h + 36, "data", 4); memcpy(h + 40, &data_bytes, 4);    
}    
    
// Синтез одного сектора WAV-файла слота (bank = 1..7) в buffer.    
// off_in_file = (lba - file_lba) * 512.    
static void wav_sector(uint8_t bank, uint32_t off_in_file, uint8_t* buffer) {    
    uint32_t len = af_rec_slot_len(bank);    
    uint32_t out_samp = (len * AF_FM_WAV_RATE) / AF_FM_REC_RATE;    
    uint32_t data_bytes = out_samp * 2;    
    const int16_t* pcm = af_rec_slot_data(bank);    
    
    uint8_t h[44];      
    wav_header(h, data_bytes);          // один раз на сектор    
    for (uint32_t i = 0; i < AF_FM_SECTOR_SIZE; i++) {      
        uint32_t p = off_in_file + i;      
        if (p < 44) {      
            buffer[i] = h[p];      
        } else {    
            uint32_t k = (p - 44) / 2;              // индекс выходного отсчёта    
            if (k >= out_samp || pcm == nullptr) { buffer[i] = 0; continue; }    
            // Линейный ресемплинг 7732 -> 16000: src = k * 7732 / 16000    
            uint32_t num = k * AF_FM_REC_RATE;    
            uint32_t si  = num / AF_FM_WAV_RATE;    
            uint32_t fr  = num % AF_FM_WAV_RATE;    // 0..15999    
            int32_t a = pcm[si];    
            int32_t b = (si + 1 < len) ? pcm[si + 1] : a;    
            int16_t v = (int16_t)(a + ((b - a) * (int32_t)fr) / (int32_t)AF_FM_WAV_RATE);    
            buffer[i] = (p & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;    
        }    
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
    if (fm_dirty) fm_dirty_ms = millis();   // flush = конец пачки записи    
}    
  
int32_t af_fm_msc_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {      
    if (buffer == nullptr || lba >= AF_FM_SECTOR_COUNT) return -1;      
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;      
    if (lba >= AF_FM_WAV_LBA(0)) return -1;        // WAV — read-only      
    if (lba >= AF_FM_RAM_SECTORS) return -1;       // зазоры — read-only      
    memcpy(fm_disk[lba], buffer, AF_FM_SECTOR_SIZE);        
    fm_dirty   = true;    
    fm_dirty_ms = millis();          // дебаунс -> парсер в af_fm_poll    
    return (int32_t)AF_FM_SECTOR_SIZE;        
}  
