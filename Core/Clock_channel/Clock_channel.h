#ifndef __CLOCK_CHANNEL_H__
#define __CLOCK_CHANNEL_H__

#include <Check_clock.h>
#include <stm32f1xx_hal.h>
#include <Device_relay.h>

// (-) ----- Дописать документации кода

extern RTC_HandleTypeDef hrtc; // стандартный HAL RTC

struct clock_channel_t {
	RTC_TimeTypeDef time_inclusion; /**< время включения сигнала */
	RTC_TimeTypeDef time_shutdown;  /**< время выключения сигнала */
	int8_t deviation_sec;           /**< "ширина" временных точек */
	struct relay_t* relay;          /**< включаемый модуль */
    bool state;                     /**< текущее состояние сигнала */
};

struct clock_channel_t clock_channel_init(struct relay_t* relay, RTC_TimeTypeDef time_inclusion, RTC_TimeTypeDef time_shutdown);

// проверяет время, 00 - не пересекается, 01 - пересекается с включением, 10 - пересекается с выключением, 11 - и одно, и другое
uint8_t state_change_clock(struct clock_channel_t* clock_channel);

// изменяет состояние если нужно, возвращает факт изменения
bool check_state_by_RTC(struct clock_channel_t* clock_channel);

// ищет ближайший будильник среди массива исключая текущие (+-.deviation_sec сек)
RTC_TimeTypeDef find_alarm_clock(struct clock_channel_t* clock_channels, uint8_t amt_clock_channel);

#endif // __CLOCK_CHANNEL_H__
