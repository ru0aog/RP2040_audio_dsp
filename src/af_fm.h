// src/af_fm.h  
//  
// Файловый менеджер: FAT12-диск 16 КБ в RAM, один файл SET.TXT —  
// текстовый дамп таблицы banks (cur + ячейки 1..7).  
// MSC-колбэки регистрируются в af_usb.cpp (usb_msc).  
// Этап 1: запись с ПК — только в RAM (флаг dirty), парсер и flash — этап 2.  
  
#pragma once  
#include <Arduino.h>  
  
#define AF_FM_SECTOR_SIZE   512u  
#define AF_FM_SECTOR_COUNT  32u    // 16 КБ: boot + FAT + dir + 29 data  
#define AF_FM_DATA_START    3u     // LBA 0=boot, 1=FAT, 2=dir, 3..=data  
#define AF_FM_DATA_SECTORS  (AF_FM_SECTOR_COUNT - AF_FM_DATA_START)  
  
void    af_fm_init(void);              // образ диска, пустой SET.TXT  
void    af_fm_poll(void);              // отложенная регенерация (stale)  
void    af_fm_request_sync(void);      // пометить SET.TXT устаревшим  
void    af_fm_sync(void);              // регенерировать SET.TXT сейчас  
bool    af_fm_dirty(void);             // ПК писал в диск (этап 2: парсер)  
uint32_t af_fm_set_size(void);         // размер SET.TXT, байт  
  
// MSC-колбэки для usb_msc.setReadWriteCallback (af_usb.cpp).  
int32_t af_fm_msc_read (uint32_t lba, void* buffer, uint32_t bufsize);  
int32_t af_fm_msc_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize);  
void    af_fm_msc_flush(void);  
