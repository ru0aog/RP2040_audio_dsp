// src/af_fm.h    
//    
// Файловый менеджер: FAT12-диск ~2 МБ, RAM-образ ~16 КБ
// (boot+FAT+dir+SET.TXT+CW.TXT). WAV-данные REC1..REC4.WAV    
// синтезируются в af_fm_msc_read из flash-слотов магнитофона.    
// MSC-колбэки регистрируются в af_usb.cpp (usb_msc).    
  
#pragma once  
#include <Arduino.h>  
  
#define AF_FM_SECTOR_SIZE   512u    
    
// Виртуальный диск: в RAM только boot+FAT+dir+SET.TXT+CW.TXT, WAV-данные      
// синтезируются в af_fm_msc_read из flash-слотов магнитофона.      
//   LBA 0        boot          
//   LBA 1..14    FAT (14 секторов — цепочки 4 WAV по ~876 кластеров + пул)            
//   LBA 15       root dir (6 записей: SET.TXT + CW.TXT + REC1..REC4.WAV)            
//   LBA 16..31   SET.TXT (16 кластеров резерв, ~8 КБ)            
//   LBA 32       CW.TXT (1 кластер, 4 строки сообщений магнитофона)          
//   LBA 33..     REC1..REC4.WAV, каждый по AF_FM_WAV_CLUSTERS кластеров       
  
#define AF_FM_FAT_SECTORS   14u    // 14×512/1.5 = 4778 записей >= ~4421 кластеров    
#define AF_FM_DIR_LBA       (1u + AF_FM_FAT_SECTORS)          // 15        
#define AF_FM_DATA_START    (AF_FM_DIR_LBA + 1u)              // 16        
#define AF_FM_SET_CLUSTERS  16u                               // резерв под SET.TXT    
#define AF_FM_SET_LBA       AF_FM_DATA_START    
    
// WAV-экспорт: PCM16 mono. ADPCM-слоты (codec=1) декодируются 1:1          
// на 7732 Гц; PCM-слоты (codec=0, legacy) ресемплируются в 8000 Гц.          
// Макс. размер файла = RAM-буфер: 224000 отсчётов ≈ 29 с,            
// ~438 КБ данных + 44 Б заголовок. PCM-слот меньше (~110 КБ).                  
// Экспорт капается в af_fm_sync/wav_sector до AF_FM_WAV_MAX_SAMP.          
#define AF_FM_WAV_RATE      8000u      // частота PCM-экспорта legacy-слотов    
#define AF_FM_REC_RATE      7732u      // частота слота (и ADPCM-экспорта)    
// Размер RECn.WAV привязан к RAM-буферу (224000 отсчётов ~29 с ADPCM),  
// а не к ёмкости слота (~59 с): больше буфера записать/залить нельзя.  
// 876 кластеров × 4 файла + пул 900 — FAT 14 секторов (4778 записей) хватает.    
#define AF_FM_WAV_MAX_SAMP  (AF_REC_RAM_SAMP + 2u)  
#define AF_FM_WAV_MAX_BYTES (44u + AF_FM_WAV_MAX_SAMP * 2u)      
#define AF_FM_WAV_CLUSTERS  ((AF_FM_WAV_MAX_BYTES + 511u) / 512u)   // ~876        
#define AF_FM_WAV_FILES     4    // 4 слота магнитофона = 4 сообщения (селектор GP13/GP28)      
//   LBA 32       CW.TXT (1 кластер, 4 строки сообщений магнитофона)          
//   LBA 33..     REC1..REC4.WAV          
#define AF_FM_CW_CLUSTERS   1u        
#define AF_FM_CW_LBA        (AF_FM_SET_LBA + AF_FM_SET_CLUSTERS)   // 32     
#define AF_FM_WAV_LBA(i)    (AF_FM_CW_LBA + AF_FM_CW_CLUSTERS + (i) * AF_FM_WAV_CLUSTERS)   // 33+      
//#define AF_FM_SECTOR_COUNT  (AF_FM_WAV_LBA(AF_FM_WAV_FILES))   // ~3537          
#define AF_FM_RAM_SECTORS   (AF_FM_CW_LBA + AF_FM_CW_CLUSTERS)   // 33 — boot+FAT+dir+SET+CW в RAM      
  
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
// Пул >= макс. RECn.WAV (~438 КБ): Explorer проверяет свободное место  
// до копирования и отказывает, если файла больше, чем свободных кластеров,  
// даже при перезаписи существующего RECn.WAV.  
#define AF_FM_FREE_SECTORS   900u  // ~450 КБ пул: сюда хост пишет данные новых файлов    
#define AF_FM_FREE_LBA       AF_FM_WAV_LBA(AF_FM_WAV_FILES)  
#define AF_FM_SECTOR_COUNT   (AF_FM_FREE_LBA + AF_FM_FREE_SECTORS)  