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

