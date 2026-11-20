// src/af_fm.cpp  
//  
// FAT12-диск 16 КБ в RAM. Раскладка:  
//   LBA 0    boot sector  
//   LBA 1    FAT (1 сектор, кластер = 1 сектор)  
//   LBA 2    root dir (16 записей, используется одна — SET.TXT)  
//   LBA 3..  data: SET.TXT, кластеры 2..N подряд  
//  
// SET.TXT генерируется af_cli_format_banks() — тот же текст, что 'banks'.  
// Запись с ПК копируется в образ и ставит fm_dirty (парсер — этап 2).  
  
#include "af_config.h"  
#include "af_fm.h"  
#include <Arduino.h>  
#include <string.h>  
#include <stdio.h>  
  
// Рендер таблицы banks в буфер (af_cli.cpp — там AF_PARAMS/bank_field).  
// Возвращает длину текста без NUL.  
extern size_t af_cli_format_banks(char* buf, size_t cap);  
  
static uint8_t  fm_disk[AF_FM_SECTOR_COUNT][AF_FM_SECTOR_SIZE];  
static uint32_t fm_set_len = 0;        // байт в SET.TXT  
static bool     fm_dirty   = false;    // ПК писал — ждёт парсера (этап 2)  
static bool     fm_stale   = true;     // SET.TXT устарел — регенерировать  
  
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
    const uint16_t ts = AF_FM_SECTOR_COUNT;  
    memcpy(&b[19], &ts, 2);  
    b[21] = 0xF8;                        // media  
    b[22] = 1;  b[23] = 0;               // sectors/FAT  
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
  
static void fm_build_fat(uint16_t clusters) {   // clusters = занятых кластеров  
    uint8_t* f = fm_disk[1];  
    memset(f, 0, AF_FM_SECTOR_SIZE);  
    f[0] = 0xF8; f[1] = 0xFF; f[2] = 0xFF;       // media + EOF кластера 1  
    for (uint16_t c = 0; c < clusters; c++) {  
        uint16_t cl = 2 + c;                     // первый data-кластер = 2  
        fat12_set(f, cl, (c == clusters - 1) ? 0xFFF : (uint16_t)(cl + 1));  
    }  
}  
  
static void fm_build_dir(uint32_t size, uint16_t first_cluster) {  
    uint8_t* d = fm_disk[2];  
    memset(d, 0, AF_FM_SECTOR_SIZE);  
    memcpy(&d[0], "SET     TXT", 11);            // 8.3 имя  
    d[11] = 0x20;                                // archive  
    // дата/время — нули, хост покажет дефолт  
    memcpy(&d[26], &first_cluster, 2);  
    memcpy(&d[28], &size, 4);  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
  
void af_fm_init(void) {  
    memset(fm_disk, 0, sizeof(fm_disk));  
    fm_build_boot();  
    fm_build_fat(0);  
    fm_build_dir(0, 0);  
    fm_set_len = 0;  
    fm_dirty   = false;  
    fm_stale   = true;                 // первый sync — после af_store_init  
}  
  
void af_fm_request_sync(void) { fm_stale = true; }  
  
void af_fm_sync(void) {  
    // Рендер прямо в data-область: SET.TXT начинается с LBA 3.  
    char* txt = (char*)&fm_disk[AF_FM_DATA_START][0];  
    size_t cap = AF_FM_DATA_SECTORS * AF_FM_SECTOR_SIZE;  
    size_t n = af_cli_format_banks(txt, cap);  
    if (n >= cap) n = cap - 1;  
    fm_set_len = (uint32_t)n;  
  
    uint16_t clusters = (uint16_t)((fm_set_len + AF_FM_SECTOR_SIZE - 1)  
                                   / AF_FM_SECTOR_SIZE);  
    fm_build_fat(clusters);  
    fm_build_dir(fm_set_len, clusters ? 2 : 0);  
    fm_stale = false;  
}  
  
void af_fm_poll(void) {  
    if (fm_stale) af_fm_sync();  
}  
  
bool     af_fm_dirty(void)    { return fm_dirty; }  
uint32_t af_fm_set_size(void) { return fm_set_len; }  
  
// ---------------------------------------------------------------------------  
// MSC-колбэки  
// ---------------------------------------------------------------------------  
  
int32_t af_fm_msc_read(uint32_t lba, void* buffer, uint32_t bufsize) {  
    if (buffer == nullptr || bufsize == 0 || lba >= AF_FM_SECTOR_COUNT)  
        return -1;  
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;   // читаем целыми секторами  
    memcpy(buffer, fm_disk[lba], AF_FM_SECTOR_SIZE);  
    return (int32_t)AF_FM_SECTOR_SIZE;  
}  
  
int32_t af_fm_msc_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {  
    if (buffer == nullptr || lba >= AF_FM_SECTOR_COUNT) return -1;  
    if (bufsize < AF_FM_SECTOR_SIZE) return -1;  
    memcpy(fm_disk[lba], buffer, AF_FM_SECTOR_SIZE);  
    fm_dirty = true;                 // этап 2: дебаунс -> парсер -> flash  
    return (int32_t)AF_FM_SECTOR_SIZE;  
}  
  
void af_fm_msc_flush(void) {  
    // этап 1: пусто. Этап 2: здесь дебаунс-метка времени последней записи.  
}
