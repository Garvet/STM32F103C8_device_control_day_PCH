#ifndef DEVICE_MOSFET_H_
#define DEVICE_MOSFET_H_

#include <stm32f1xx_hal.h>
#include <stdbool.h>

struct mosfet_t {
	TIM_HandleTypeDef*  htim;
	uint32_t tim_channel;
	uint32_t max_compare;
	bool invert;

	uint32_t compare;
};

//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (i));
struct mosfet_t mosfet_init(TIM_HandleTypeDef*  htim, uint32_t tim_channel, uint32_t max_compare, bool invert);

//bool relay_toggle(struct mosfet_t* mosfet);  // Смена состояния реле на другое
//void relay_off(struct mosfet_t* mosfet);     // Смена состояния реле на 0
//void relay_on(struct mosfet_t* mosfet);      // Смена состояния реле на 1
bool mosfet_set_compare(struct mosfet_t* mosfet, uint32_t compare);      // Смена состояния реле на 1

//uint32_t relay_pin_signal(struct mosfet_t* mosfet);  // Получить состояние сигнала реле

#endif // DEVICE_MOSFET_H_
