// src/af_cli.h  
//  
// Serial CLI (Core 0): управление полосой фильтра, NT, NR,  
// вывод диагностики. Парсинг построчный, без блокировок.  
  
#pragma once  
  
#include <Arduino.h>  
  
// Инициализация (печать приветствия/справки). Вызывать из setup().  
void af_cli_init(void);  
  
// Поллинг: читать Serial, разбирать команды. Вызывать из loop().  
void af_cli_poll(void);

// Таблица banks в текстовый буфер (SET.TXT для af_fm). Возвращает длину.  
size_t af_cli_format_banks(char* buf, size_t cap);  
