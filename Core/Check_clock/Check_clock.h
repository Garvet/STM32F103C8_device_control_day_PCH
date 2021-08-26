#ifndef CHECK_CLOCK_H_
#define CHECK_CLOCK_H_

#include <stm32f1xx_hal.h>
#include <stdbool.h>

// (-) ----- Дописать документации кода

/**
  * @brief  Добавление секунд ко времени
  */
void clock_add_second(RTC_TimeTypeDef* corrected_time, int8_t sec);
/**
  * @brief  Проверка меньшего времени
  */
bool clock_comparison(const RTC_TimeTypeDef* less, const RTC_TimeTypeDef* more);
/**
  * @brief  Проверка нахождения времени в диапазоне
  */
bool clock_range_membership(const RTC_TimeTypeDef* checked_time, const RTC_TimeTypeDef* lower_limit, const RTC_TimeTypeDef* upper_limit);

/**
  * @brief  Расчёт разницы во времени между первым и вторым аргументами.
  */
RTC_TimeTypeDef clock_subtraction(const RTC_TimeTypeDef* lhs, const RTC_TimeTypeDef* rhs);
/**
  * @brief  Получение количества секунд от 00:00:00
  */
unsigned long clock_get_sec(const RTC_TimeTypeDef* time);

/**
  * @brief  Проверка ближайшего времени
  * @details Данная функция служит для поиска ближайшего времени
  *         к стартовому, не влючая его.
  * @param[in]  start_time: текущее время
  * @param[in]  near_time: ближнее время
  * @param[in]  distant_time: дальнее время
  * @retval возвращает истину, если near_time ближе к start_time,
  *         чем distant_time по ходу течения времени.
  */
bool search_nearest_clock(const RTC_TimeTypeDef* start_time, const RTC_TimeTypeDef* near_time, const RTC_TimeTypeDef* distant_time);


#endif // CHECK_CLOCK_H_
