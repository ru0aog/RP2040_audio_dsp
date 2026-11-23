// src/af_fm.h  
//  
// Файловый менеджер: FAT12-диск 16 КБ в RAM, один файл SET.TXT —  
// текстовый дамп таблицы banks (cur + ячейки 1..7).  
// MSC-колбэки регистрируются в af_usb.cpp (usb_msc).  
  
#pragma once  
#include <Arduino.h>  
  
#define AF_FM_SECTOR_SIZE   512u    
    
// Виртуальный диск: в RAM только boot+FAT+dir+SET.TXT, WAV-данные    
// синтезируются в af_fm_msc_read из flash-слотов магнитофона.    
//   LBA 0        boot    
//   LBA 1..10    FAT (10 секторов — цепочки 7 WAV по ~453 кластера)    

// Файловый менеджер: FAT12-диск в RAM, файлы SET.TXT и CW.TXT —    
//   LBA 11       root dir (9 записей: SET.TXT + CW.TXT + REC1..REC7.WAV)      
//   LBA 12..27   SET.TXT (16 кластеров резерв, ~8 КБ)      
//   LBA 28       CW.TXT (1 кластер, 4 строки сообщений магнитофона)    
//   LBA 29..     REC1..REC7.WAV, каждый по AF_FM_WAV_CLUSTERS кластеров 

#define AF_FM_FAT_SECTORS   10u    
#define AF_FM_DIR_LBA       (1u + AF_FM_FAT_SECTORS)          // 11    
#define AF_FM_DATA_START    (AF_FM_DIR_LBA + 1u)              // 12    
#define AF_FM_SET_CLUSTERS  16u                               // резерв под SET.TXT    
#define AF_FM_SET_LBA       AF_FM_DATA_START    
    
// WAV: 8000 Гц mono 16-bit, ресемплинг из 7732 Гц.      
// out_samp_max = 56000*8000/7732 + 1 ~= 57942; +44 Б заголовок.      
#define AF_FM_WAV_RATE      8000u      
#define AF_FM_REC_RATE      7732u    
#define AF_FM_WAV_MAX_SAMP  ((AF_REC_RAM_SAMP * AF_FM_WAV_RATE) / AF_FM_REC_RATE + 2u)    
#define AF_FM_WAV_MAX_BYTES (44u + AF_FM_WAV_MAX_SAMP * 2u)    
#define AF_FM_WAV_CLUSTERS  ((AF_FM_WAV_MAX_BYTES + 511u) / 512u)   // ~453    
#define AF_FM_WAV_FILES     7u
//   LBA 28       CW.TXT (1 кластер, 4 строки сообщений магнитофона)    
//   LBA 29..     REC1..REC7.WAV    
#define AF_FM_CW_CLUSTERS   1u    
#define AF_FM_CW_LBA        (AF_FM_SET_LBA + AF_FM_SET_CLUSTERS)   // 28 
#define AF_FM_WAV_LBA(i)    (AF_FM_CW_LBA + AF_FM_CW_CLUSTERS + (i) * AF_FM_WAV_CLUSTERS)   // 29+  
//#define AF_FM_SECTOR_COUNT  (AF_FM_WAV_LBA(AF_FM_WAV_FILES))   // ~3199    
#define AF_FM_RAM_SECTORS   (AF_FM_CW_LBA + AF_FM_CW_CLUSTERS)   // 29 — boot+FAT+dir+SET+CW в RAM  
  
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

// Аплоад WAV: перезапись RECn.WAV -> конвертация -> слот банка n.    
#define AF_FM_UPLOAD_TIMEOUT_MS  1500u   // коммит после последнего сектора    
#define AF_FM_WAV_HDR_MAX        512u    // заголовок WAV в пределах 1 сектора    

// af_fm.h  
#define AF_FM_FREE_SECTORS   400u  // ~200 КБ пул: сюда хост пишет данные новых файлов  
#define AF_FM_FREE_LBA       AF_FM_WAV_LBA(AF_FM_WAV_FILES)  
#define AF_FM_SECTOR_COUNT   (AF_FM_FREE_LBA + AF_FM_FREE_SECTORS)  