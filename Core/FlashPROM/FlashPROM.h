#ifndef FLASHPROM_H_
#define FLASHPROM_H_

#include "main.h"
#include "stdbool.h"

#define STARTADDR_2 ((uint32_t)0x0801F800)         // адрес, с которого будет начинаться запись во флеш (с начала 125-ой страницы F103)
#define ENDMEMORY_2 ((uint32_t)0x0801F800 + 1024)  // последняя ячейка флеша (F103)
#define STARTADDR ((uint32_t)0x0801FC00)         // адрес, с которого будет начинаться запись во флеш (с начала 126-ой страницы F103)
#define ENDMEMORY ((uint32_t)0x0801FC00 + 1024)  // последняя ячейка флеша (F103)
#define PAGES 1                                  // количество страниц для очистки
#define BUFFSIZE 2                               // размер буфера для записи
#define DATAWIDTH 4                              // размерность данных буфера 16 бит - 2, 32 бита - 4
#define WIDTHWRITE FLASH_TYPEPROGRAM_WORD    // длина слова (16 бит) для записи в функции HAL_FLASH_Program(...), если 32бита тогда FLASH_TYPEPROGRAM_WORD
#define CELL_OFFSET_PWM_VALUES 2
//#define DEBUG 0

//typedef uint32_t buf32_t;                        // либо uint32_t

bool Erase_flash(uint8_t num);
uint32_t Flash_search_adress(uint32_t address, uint16_t cnt);
void Read_control_module_info_from_flash(uint32_t *buff);
void Write_to_flash(uint32_t *buff);
void Read_last_data_in_flash(uint32_t *buff);

uint16_t Read_PWM_info_from_flash();
bool Write_PWM_to_flash(uint16_t value);

#endif /* FLASHPROM_H_ */
