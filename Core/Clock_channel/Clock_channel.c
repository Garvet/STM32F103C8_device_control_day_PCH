#include <Clock_channel.h>

extern RTC_HandleTypeDef hrtc; // стандартный HAL RTC-одуль

struct clock_channel_t clock_channel_init(struct relay_t* relay, RTC_TimeTypeDef time_inclusion, RTC_TimeTypeDef time_shutdown) {
	struct clock_channel_t time_channel;
	time_channel.time_inclusion = time_inclusion;
	time_channel.time_shutdown = time_shutdown;
	time_channel.relay = relay;
	time_channel.state = false;
	time_channel.deviation_sec = 5;
    return time_channel;
}

// проверяет время, 00 - не пересекается, 01 - пересекается с включением, 10 - пересекается с выключением, 11 - и одно, и другое
// 3-й бит отвечает за текущее состояние
uint8_t state_change_clock(struct clock_channel_t* clock_channel) {
	uint8_t state = 0;
	RTC_TimeTypeDef rtc_time;
	RTC_TimeTypeDef rtc_time_dev_less;
	RTC_TimeTypeDef rtc_time_dev_more;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	rtc_time_dev_less = rtc_time;
	rtc_time_dev_more = rtc_time;

	clock_add_second(&rtc_time_dev_less, -clock_channel->deviation_sec);
	clock_add_second(&rtc_time_dev_more, clock_channel->deviation_sec);

	// Проверка на принадлежность time_inclusion в рамках погрешности deviation_sec
	if(clock_range_membership(&clock_channel->time_inclusion, &rtc_time_dev_less, &rtc_time_dev_more)) {
		state |= 0b001;
	}
	// Проверка на принадлежность time_shutdown в рамках погрешности deviation_sec
	if(clock_range_membership(&clock_channel->time_shutdown, &rtc_time_dev_less, &rtc_time_dev_more)) {
		state |= 0b010;
	}
	// Проверка на принадлежность отрезку [time_inclusion, time_shutdown]
	if(clock_range_membership(&rtc_time, &clock_channel->time_inclusion, &clock_channel->time_shutdown)) {
		state |= 0b100;
	}
	return state;
}

// изменяет состояние если нужно, возвращает факт изменения
bool check_state_by_RTC(struct clock_channel_t* clock_channel) {
	// проверяем принадлежим ли хоть одной точке
	uint8_t check_time = state_change_clock(clock_channel);
	if((check_time & 0b11) == 0) {
		if(check_time)
			clock_channel->state = true;
		else
			clock_channel->state = false;
		if(clock_channel->relay != NULL) // если есть реле, то и его переключаем
			relay_set_state(clock_channel->relay, clock_channel->state);
		return false; // нет => выходим
	}
	if(check_time == 1) {
		// да - включающей
		if(clock_channel->state)
			return false; // но уже включены => выходим
		// включаем
		clock_channel->state = true;
	}
	else if(check_time == 2) {
		// да - выключающей
		if(!clock_channel->state)
			return false; // но уже выключены => выходим
		// выключаем
		clock_channel->state = false;
	}
	else {
		// Да - меняющий (попали на порог включения и выключения одновременно)
		clock_channel->state = !clock_channel->state;
	}
	if(clock_channel->relay != NULL) // если есть реле, то и его переключаем
		relay_set_state(clock_channel->relay, clock_channel->state);
	return true; // сообщаем о переключении
}

// ищет ближайший будильник среди массива исключая текущие (+-.deviation_sec сек)
RTC_TimeTypeDef find_alarm_clock(struct clock_channel_t* clock_channels, uint8_t amt_clock_channel) {
	RTC_TimeTypeDef rtc_time, near_time;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	near_time = rtc_time;
	for(int i = 0; (i < amt_clock_channel) || (i == 255); ++i){
		if(search_nearest_clock(&rtc_time, &(clock_channels[i].time_inclusion), &near_time)) {
			near_time = clock_channels[i].time_inclusion;
		}
		if(search_nearest_clock(&rtc_time, &(clock_channels[i].time_shutdown), &near_time)) {
			near_time = clock_channels[i].time_shutdown;
		}
	}
	return near_time;
}
