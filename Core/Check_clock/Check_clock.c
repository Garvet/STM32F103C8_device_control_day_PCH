#include <Check_clock.h>

void clock_add_second(RTC_TimeTypeDef* corrected_time, int8_t sec) {
	if(sec == 0)
		return;
	if(sec > 0) {
		if(sec >= 60)
			clock_add_second(corrected_time, (sec-60));
		if(corrected_time->Seconds > (60 - sec)) {
			corrected_time->Seconds = corrected_time->Seconds - 60 + sec;
			if(corrected_time->Minutes == 59) {
				corrected_time->Minutes = 0;
				if(corrected_time->Hours == 23) {
					corrected_time->Hours = 0;
				}
				else {
					corrected_time->Hours = corrected_time->Hours + 1;
				}
			}
			else {
				corrected_time->Minutes = corrected_time->Minutes + 1;
			}
		}
		else {
			corrected_time->Seconds = corrected_time->Seconds + sec;
		}

	}
	else {
		// sec < 0
		sec = -sec;
		if(sec >= 60)
			clock_add_second(corrected_time, -(sec-60));

		if(corrected_time->Seconds < sec) {
			corrected_time->Seconds = 60 + corrected_time->Seconds - sec;

			if(corrected_time->Minutes == 0) {
				corrected_time->Minutes = 59;

				if(corrected_time->Hours == 0) {
					corrected_time->Hours = 23;
				}
				else {
					corrected_time->Hours = corrected_time->Hours - 1;
				}
			}
			else {
				corrected_time->Minutes = corrected_time->Minutes - 1;
				corrected_time->Hours = corrected_time->Hours;
			}
		}
		else {
			corrected_time->Seconds = corrected_time->Seconds - sec;
			corrected_time->Minutes = corrected_time->Minutes;
			corrected_time->Hours = corrected_time->Hours;
		}
	}
}

bool clock_comparison(const RTC_TimeTypeDef* less, const RTC_TimeTypeDef* more) {
	if(less->Hours < more->Hours)
		return true;
	if(less->Hours > more->Hours)
		return false;
	// Часы равны
	if(less->Minutes < more->Minutes)
		return true;
	if(less->Minutes > more->Minutes)
		return false;
	// Минуты равны
	if(less->Seconds < more->Seconds)
		return true;
	return false;
}

bool clock_range_membership(const RTC_TimeTypeDef* checked_time, const RTC_TimeTypeDef* lower_limit, const RTC_TimeTypeDef* upper_limit) {
	if(clock_comparison(lower_limit, upper_limit)) { // М < Б
		if((!clock_comparison(checked_time, lower_limit)) && (!clock_comparison(upper_limit, checked_time)))
			return true;  // М <= Т <= Б
	}
	else if(clock_comparison(upper_limit, lower_limit)) { // Б < М
		if((!clock_comparison(checked_time, lower_limit)) || (!clock_comparison(upper_limit, checked_time)))
			return true;  // (Т <= Б < М) || (Б < М <= Т)
	}
	else { // Б = М
		if((!clock_comparison(checked_time, lower_limit)) && (!clock_comparison(upper_limit, checked_time)))
			return true;  // Т = Б = М
	}
	return false;
}

RTC_TimeTypeDef clock_subtraction(const RTC_TimeTypeDef* lhs, const RTC_TimeTypeDef* rhs) {
	RTC_TimeTypeDef result;
	bool carryover = false;

	result.Seconds = lhs->Seconds - rhs->Seconds;
	if(lhs->Seconds < rhs->Seconds) {
		result.Seconds += 60;
		carryover = true;
	}

	result.Minutes = lhs->Minutes - rhs->Minutes - carryover;
	if(lhs->Minutes < (rhs->Minutes - carryover)) {
		result.Minutes += 60;
		carryover = true;
	}
	else {
		carryover = false;
	}

	result.Hours = lhs->Hours - rhs->Hours - carryover;
	if(lhs->Hours < (rhs->Hours - carryover)) {
		result.Hours += 24;
	}

	return result;
}

unsigned long clock_get_sec(const RTC_TimeTypeDef* time) {
	return ((((unsigned long)time->Hours * 24UL) + time->Minutes) * 60UL) + time->Seconds;
}

// true => near_time - ближний, distant_time - дальний; false => distant_time - ближний, near_time - дальний,
// самое большое расстояние между start_time и start_time
bool search_nearest_clock(const RTC_TimeTypeDef* start_time, const RTC_TimeTypeDef* near_time, const RTC_TimeTypeDef* distant_time) {
	bool from_near_to_distant = clock_range_membership(start_time, near_time, distant_time);
	bool from_distant_to_near = clock_range_membership(start_time, distant_time, near_time);
	if(from_distant_to_near) {
		// D <= T <= N
		if(from_near_to_distant) {
			// (T == D) || (T == N)
			if((!clock_comparison(start_time, distant_time)) && (!clock_comparison(distant_time, start_time)))
				return true; // T == D
		}
		else // D < T < N
			return true;
	}
	// (N <= T < D)
	return false;
}
