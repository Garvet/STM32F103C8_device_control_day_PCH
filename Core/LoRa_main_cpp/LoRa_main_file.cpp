#include <Grow_device.h>
#include <Grow_device_interface.h>
#include "../LoRa_main_cpp/LoRa_main_file.h"
#include <LoRa_contact_data.h>
#include <array>
#include "../Grow_device_component/Grow_device_component.h"

constexpr std::array<uint8_t, AMT_BYTES_SYSTEM_ID> MODULE_ID =
	{ 0x60, 0x86, 0xd0, 0x0e, 0x34, 0x2b, 0x73, 0x2b, 0x2d, 0x4a, 0x42, 0x77 };
//		{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x06, 0x06, 0x06, 0x01, 0x01, 0x01};
//		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


//constexpr std::array<uint8_t, AMT_BYTES_SYSTEM_ID> MODULE_ID =
//	{ 0x60, 0x86, 0xd0, 0x2a, 0x34, 0x2b, 0x73, 0x2b, 0x2d, 0x4a, 0x42, 0x78 };

//const uint8_t AMT_COMPONENT = 1;
//Type_device device_array[AMT_COMPONENT] = { Illumination_level };

//const uint8_t AMT_COMPONENT = 2;
//Type_device device_array[AMT_COMPONENT] =  { Phytolamp_digital, Phytolamp_digital};
//Type_device device_array[AMT_COMPONENT] =  { Signal_digital, Signal_digital, Signal_digital};

const uint8_t AMT_COMPONENT = 1;
Type_device device_array[AMT_COMPONENT] =  { Pumping_system };
//Type_device device_array[AMT_COMPONENT] = { Air_humidity, Air_temperature, Illumination_level };
//Type_device device_array[AMT_COMPONENT] = { Air_humidity, Air_temperature, Indicator_eCO2 };


//const uint8_t AMT_COMPONENT = 5;
//	Type_device device_array[AMT_COMPONENT] =  { Air_humidity, Air_temperature, Water_temperature, Illumination_level, Indicator_eCO2 };

LoRa_contact_data contact_data;
Grow_device grow_device(AMT_COMPONENT, device_array);

uint32_t contact_status;

uint32_t control_module_adr = 0;
uint32_t control_module_channel = 0;

extern volatile bool end_contact;

volatile bool rtc_sync_starting; // синхронизация при RTC запуске

enum {
    REGISTRATION_MODE = 0,
    WORKING_MODE
} current_mode;


extern "C" {

void Get_control_module_info_from_main(uint32_t* id_main) {
	control_module_adr = id_main[0];
	control_module_channel = id_main[1];
}

void Send_registration_packet() {
	grow_device_interface.send_registration_packet(grow_device, contact_data);
}

bool Init_lora_module(SPI_HandleTypeDef *spi) {
	return contact_data.init_lora_module(spi);
}
uint8_t Begin_lora_module(uint64_t frequency, bool paboost, uint8_t signal_power, uint8_t SF, uint64_t SBW, uint8_t sync_word) {
	grow_device.set_system_id(MODULE_ID);
	contact_data.begin_lora_module(frequency, paboost, signal_power, SF, SBW, sync_word);
	if (control_module_adr == 0x00000000) {
		current_mode = REGISTRATION_MODE;
		Send_registration_packet();
		rtc_sync_starting = false;
	}
	else {
		grow_device_interface.load_data(grow_device, contact_data, control_module_adr, control_module_channel);
		current_mode = WORKING_MODE;
		contact_data.wait_recipient(grow_device.get_address_control_module());
		grow_device.set_rtc_sync(true);
		rtc_sync_starting = true;
	}
	return 0;
}

uint8_t Main_cpp(DevicesDataTypeDef* devices_data) {
//	grow_device.set_value(0, devices_data->humidity);
//	grow_device.set_value(1, devices_data->temperature);
//	grow_device.set_value(2, devices_data->water_temperature);
//	grow_device.set_value(3, devices_data->lux);
//	grow_device.set_value(4, devices_data->CO2);
//	grow_device_interface.build_data_packet(grow_device, contact_data);

//	grow_device.set_value(0, devices_data->humidity);
//	grow_device.set_value(1, devices_data->temperature);
//	grow_device.set_value(2, devices_data->CO2);
//	grow_device_interface.build_data_packet(grow_device, contact_data);

	grow_device.set_value(0, devices_data->device1);
//	grow_device.set_value(1, devices_data->device2);
//	grow_device.set_value(2, devices_data->device3);
	return grow_device_interface.build_data_packet(grow_device, contact_data);
}

void LoRa_sleep() {
	// sleep LoRa module
    NVIC_DisableIRQ(EXTI15_10_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);
    contact_data.end_contact();
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
	// Sleep STM
}

void Contact_group_control_module() {
	contact_status = contact_data.work_contact_system();
	switch (current_mode) {
		case (REGISTRATION_MODE): {
			if(contact_data.get_signal_complete()) {
				contact_data.broadcast_receive(); // Ожидаем ответа на запрос или ошибку запроса
			}
			if(contact_data.get_state_contact() == SC_PACKET_ACCEPTED) {
				if(grow_device_interface.check_regist_packet(grow_device, contact_data)) {
					current_mode = WORKING_MODE;
					contact_data.wait_recipient(grow_device.get_address_control_module()); // Начинаем слушать на наличие управляющих пакетов
					break;
				}
			}
	        break;
		}
		case (WORKING_MODE): {
	        if(contact_status != 0) {
	        	if(grow_device_interface.check_contact_error(grow_device, contact_data)) {
					current_mode = REGISTRATION_MODE;
	        		contact_data.broadcast_receive();
	        	}
	        	else {
	        		contact_data.wait_recipient(grow_device.get_address_control_module());
	        	}
	        }
	        if(contact_data.get_signal_complete()) {
	        	grow_device_interface.read_received_data_packets(grow_device, contact_data);
	        	end_contact = true;
	        	contact_data.clear_send_packet();
	            contact_data.wait_recipient(grow_device.get_address_control_module());
	        }
		}
	}
}

void Set_sync_rtc() {
	grow_device.set_rtc_sync(true);
}
bool Get_stop() {
	return (rtc_sync_starting || grow_device.get_sleep_state());
}

} // extern "C"
