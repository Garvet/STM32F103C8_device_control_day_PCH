#ifndef __LORA_MAIN_FILE_H__
#define __LORA_MAIN_FILE_H__

//#define CONTACT_DATA_MAX_PACKET 10
#include <main.h>


typedef struct {
	float lux;
	float temperature;
	float humidity;
	float pressure;
	float CO2;
	float TVOC;
	float water_temperature;
} SensorsDataTypeDef;

typedef struct {
	uint16_t device1; // lamp digital 1
	uint16_t device2; // lamp digital 2
	uint16_t device3; // lamp digital 3
} DevicesDataTypeDef;


#ifdef __cplusplus
extern "C" {
#endif



void LoRa_sleep();

uint8_t Main_cpp(DevicesDataTypeDef* devices_data);
void Contact_group_control_module();
bool Init_lora_module(SPI_HandleTypeDef *spi);
void Send_registration_packet();
void Get_control_module_info_from_main(uint32_t* id_main);
uint8_t Begin_lora_module(uint64_t frequency, bool paboost, uint8_t signal_power, uint8_t SF, uint64_t SBW, uint8_t sync_word);
void Set_sync_rtc();
bool Get_stop();



#ifdef __cplusplus
}
#endif

#endif // __LORA_MAIN_FILE_H__
