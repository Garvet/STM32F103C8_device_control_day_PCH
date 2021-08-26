#include <Device_mosfet.h>

struct mosfet_t mosfet_init(TIM_HandleTypeDef* htim, uint32_t tim_channel, uint32_t max_compare, bool invert) {
	struct mosfet_t mosfet;
	mosfet.htim = htim;
	mosfet.tim_channel = tim_channel;
	mosfet.max_compare = max_compare;
	mosfet.invert = invert;
	HAL_TIM_PWM_Start(htim, tim_channel);
	return mosfet;
}

bool mosfet_set_compare(struct mosfet_t* mosfet, uint32_t compare) {
	if(compare > mosfet->max_compare)
		return true;
	if(mosfet->invert)
		mosfet->compare = mosfet->max_compare - compare;
	else
		mosfet->compare = compare;
	__HAL_TIM_SET_COMPARE(mosfet->htim, mosfet->tim_channel, mosfet->compare);
	return false;
}
