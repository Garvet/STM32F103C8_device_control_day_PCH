#include "FlashPROM.h"

//extern CRC_HandleTypeDef hcrc;
uint32_t res_addr;

//uint32_t rdata[BUFFSIZE] = {0x00000000, 0x00000000};
//uint32_t wdata[BUFFSIZE] = {0x00000000, 0x00000000};
//uint8_t lora_address[3] = {0x00, 0x00, 0x00};

void Read_control_module_info_from_flash(uint32_t *buff) {

	  res_addr = Flash_search_adress(STARTADDR, BUFFSIZE * DATAWIDTH);

	  Read_last_data_in_flash(buff);

//	  if (rdata[0] == 0x0000) {
//		  write_to_flash(wdata);
//	  }
//	  else {
//		  //reg_done = 1;
//		  read_last_data_in_flash(rdata);
//	  }
}

//////////////////////// ОЧИСТКА ПАМЯТИ /////////////////////////////
void Erase_flash(void)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;     // структура для очистки флеша

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // постраничная очистка, FLASH_TYPEERASE_MASSERASE - очистка всего флеша
	EraseInitStruct.PageAddress = STARTADDR;
	EraseInitStruct.NbPages = PAGES;
	//EraseInitStruct.Banks = FLASH_BANK_1; // FLASH_BANK_2 - банк №2, FLASH_BANK_BOTH - оба банка
	uint32_t page_error = 0; // переменная, в которую запишется адрес страницы при неудачном стирании

	HAL_FLASH_Unlock(); // разблокировать флеш

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK)
	{
		//uint32_t er = HAL_FLASH_GetError();
	}
	else
	{

	}

	HAL_FLASH_Lock();
}

//////////////////////// ПОИСК СВОБОДНЫХ ЯЧЕЕК /////////////////////////////
uint32_t Flash_search_adress(uint32_t address, uint16_t cnt)
{
	uint16_t count_byte = cnt;

	while(count_byte)
	{
		if(0xFF == *(uint8_t*)address++) count_byte--;
		else count_byte = cnt;

		if(address == ENDMEMORY - 1) // если достигнут конец флеша
		{
			Erase_flash();        // тогда очищаем память
			return STARTADDR;     // устанавливаем адрес для записи с самого начала
		}
	}

	return address -= cnt;
}

//////////////////////// ЗАПИСЬ ДАННЫХ /////////////////////////////
void Write_to_flash(uint32_t *buff)
{
	res_addr = Flash_search_adress(res_addr, BUFFSIZE * DATAWIDTH); // ищем свободные ячейки начиная с последнего известного адреса

	//////////////////////// ЗАПИСЬ ////////////////////////////
	HAL_FLASH_Unlock(); // разблокировать флеш

	for(uint16_t i = 0; i < BUFFSIZE; i++)
	{
		if(HAL_FLASH_Program(WIDTHWRITE, res_addr, buff[i]) != HAL_OK)
		{
			//uint32_t er = HAL_FLASH_GetError();
		}

		res_addr = res_addr + DATAWIDTH;
	}

	HAL_FLASH_Lock(); // заблокировать флеш

	//////////////////////// проверка записанного (это можно удплить если неохота проверять) ////////////////////////
//	uint32_t crcbuff[BUFFSIZE] = {0,};
//
//	for(uint16_t i = 0; i < BUFFSIZE; i++) crcbuff[i] = (uint32_t)buff[i]; // в функцию CRC32 нужно подавать 32-х битные значения, поэтому перегоняем 16-ти битный буфер в 32-х битный
//
//	uint32_t sum1 = HAL_CRC_Calculate(&hcrc, (uint32_t*)crcbuff, BUFFSIZE); // crc буфера который только что записали
//
//	buff[0] = 0;
//	read_last_data_in_flash(buff); // читаем что записали
//
//	for(uint16_t i = 0; i < BUFFSIZE; i++) crcbuff[i] = (uint32_t)buff[i];
//
//	uint32_t sum2 = HAL_CRC_Calculate(&hcrc, (uint32_t*)crcbuff, BUFFSIZE); // crc прочитанного
//
//	if(sum1 != sum2) // если суммы записанного и прочитанного не равны, тогда что-то пошло не так
//	{
//		return;
//	}
	//////////////////////// конец проверки записанного ////////////////////////
}

//////////////////////// ЧТЕНИЕ ПОСЛЕДНИХ ДАННЫХ /////////////////////////////
void Read_last_data_in_flash(uint32_t *buff)
{
	if(res_addr == STARTADDR)
	{
		return;
	}

	uint32_t adr = res_addr - BUFFSIZE * DATAWIDTH; // сдвигаемся на начало последних данных

	for(uint16_t i = 0; i < BUFFSIZE; i++)
	{
		buff[i] = *(uint32_t*)adr; // читаем
		adr = adr + DATAWIDTH;
	}
}


