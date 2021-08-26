#ifndef FLASHPROM_H_
#define FLASHPROM_H_

#include "main.h"

#define STARTADDR ((uint32_t)0x0801FC00)         // адрес, с которого будет начинаться запись во флеш (с начала 126-ой страницы F103)
#define ENDMEMORY ((uint32_t)0x0801FC00 + 1024)  // последняя ячейка флеша (F103)
#define PAGES 1                                  // количество страниц для очистки
#define BUFFSIZE 2                               // размер буфера для записи
#define DATAWIDTH 4                              // размерность данных буфера 16 бит - 2, 32 бита - 4
#define WIDTHWRITE FLASH_TYPEPROGRAM_WORD    // длина слова (16 бит) для записи в функции HAL_FLASH_Program(...), если 32бита тогда FLASH_TYPEPROGRAM_WORD
//#define DEBUG 0

//typedef uint32_t buf32_t;                        // либо uint32_t

void Erase_flash(void);
uint32_t Flash_search_adress(uint32_t address, uint16_t cnt);
void Read_control_module_info_from_flash(uint32_t *buff);
void Write_to_flash(uint32_t *buff);
void Read_last_data_in_flash(uint32_t *buff);

#endif /* FLASHPROM_H_ */
