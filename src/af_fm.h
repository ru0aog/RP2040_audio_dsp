// src/af_fm.h    
//    
// Файловый менеджер: FAT16-диск ~10 МБ, RAM-образ ~10 КБ  
// (boot+dir+SET.TXT+CW.TXT; FAT синтезируется на чтении).  
// WAV-данные REC1..REC4.WAV 
// синтезируются в af_fm_msc_read из flash-слотов магнитофона.    
// MSC-колбэки регистрируются в af_usb.cpp (usb_msc).    
  
#pragma once  
#include <Arduino.h>  
  
#define AF_FM_SECTOR_SIZE   512u
    
// Виртуальный диск: в RAM только boot+dir+SET.TXT+CW.TXT (19 секторов).  
// FAT (80 секторов) и WAV-данные синтезируются в af_fm_msc_read —  
// цепочки регулярные, сектор FAT генерируется формулой, RAM не тратится.  
//   LBA 0        boot            
//   LBA 1..80    FAT (80 секторов, 20480 записей по 2 Б — синтез)              
//   LBA 81       root dir (6 записей: SET.TXT + CW.TXT + REC1..REC4.WAV)              
//   LBA 82..97   SET.TXT (16 кластеров резерв, ~8 КБ)              
//   LBA 98       CW.TXT (1 кластер, 4 строки сообщений магнитофона)            
//   LBA 99..     REC1..REC4.WAV, каждый по AF_FM_WAV_CLUSTERS кластеров         
  
// FAT16: 2 Б на запись. Диск 20480 секторов -> ~20398 кластеров ->  
// FAT = 20398*2/512 ~= 80 секторов. FAT не хранится в RAM — сектор  
// синтезируется в af_fm_msc_read по формуле цепочек (fat_sector()).  
#define AF_FM_FAT_SECTORS   80u    // 80×256 = 20480 записей >= ~20398 кластеров      
#define AF_FM_DIR_LBA       (1u + AF_FM_FAT_SECTORS)          // 81          
#define AF_FM_DATA_START    (AF_FM_DIR_LBA + 1u)              // 82          
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
// 876 кластеров × 4 файла + пул ~16.8К — FAT16 80 секторов (20480 записей).      
#define AF_FM_WAV_MAX_SAMP  (AF_REC_RAM_SAMP + 2u)  
#define AF_FM_WAV_MAX_BYTES (44u + AF_FM_WAV_MAX_SAMP * 2u)      
#define AF_FM_WAV_CLUSTERS  ((AF_FM_WAV_MAX_BYTES + 511u) / 512u)   // ~876        
#define AF_FM_WAV_FILES     4    // 4 слота магнитофона = 4 сообщения (селектор GP13/GP28)      
//   LBA 98       CW.TXT (1 кластер, 4 строки сообщений магнитофона)            
//   LBA 99..     REC1..REC4.WAV            
#define AF_FM_CW_CLUSTERS   1u          
#define AF_FM_CW_LBA        (AF_FM_SET_LBA + AF_FM_SET_CLUSTERS)   // 98       
#define AF_FM_WAV_LBA(i)    (AF_FM_CW_LBA + AF_FM_CW_CLUSTERS + (i) * AF_FM_WAV_CLUSTERS)   // 99+        
  
// В RAM только boot+dir+SET+CW — 19 секторов (~10 КБ). FAT и WAV  
// синтезируются в af_fm_msc_read. Маппинг LBA -> индекс fm_disk:  
//   lba 0            -> fm_disk[0]   (boot)  
//   lba DIR_LBA      -> fm_disk[1]   (root dir)  
//   lba SET_LBA+k    -> fm_disk[2+k] (SET.TXT, k=0..15)  
//   lba CW_LBA       -> fm_disk[18]  (CW.TXT)  
#define AF_FM_RAM_SECTORS   (2u + AF_FM_SET_CLUSTERS + AF_FM_CW_CLUSTERS)   // 19        
  
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

// Диск 10 МБ: пул ~8.4 МБ свободного места — Explorer пропускает  
// копирование файла любого размера (проверка free space до записи).  
// Записи хоста в пул проглатываются (кроме RECn.WAV -> аплоад).  
#define AF_FM_SECTOR_COUNT   20480u  // 10 МБ    
#define AF_FM_FREE_LBA       AF_FM_WAV_LBA(AF_FM_WAV_FILES)   // 3603    
#define AF_FM_FREE_SECTORS   (AF_FM_SECTOR_COUNT - AF_FM_FREE_LBA)  // ~16877    

// Маппинг RAM-образа fm_disk[]: FAT в RAM не хранится (синтез).    
#define AF_FM_DIR_RAM    1u    // fm_disk[1]  = root dir (LBA 81)    
#define AF_FM_SET_RAM    2u    // fm_disk[2..17] = SET.TXT (LBA 82..97)    
#define AF_FM_CW_RAM     18u   // fm_disk[18] = CW.TXT (LBA 98)    
// AF_FM_RAM_SECTORS = 19 — boot+dir+SET+CW    
