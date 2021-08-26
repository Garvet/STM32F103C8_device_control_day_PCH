#ifndef __DEVICE_RELAY_H__
#define __DEVICE_RELAY_H__

#include <stm32f1xx_hal.h>
#include <stdbool.h>

/**
  * @brief  Структура реле
  * @details Данная структура служит для упрощения контроля 
  *         разными типами реле и иных цифровых модулей, 
  *         путём разграничения управления на состояние 
  *         модуля и на управляющий сигнал, беря на себя 
  *         контроль над последним (это позволяет реализовывать 
  *         функции вне зависимости от того является ли 
  *         модуль нормально выключенным или включённым).
  */
struct relay_t {
    GPIO_TypeDef*  port; /**< порт выхода реле */
    uint16_t pin;        /**< номер выхода реле */
    bool invert;         /**< инверсия состояние-сигнал */
    bool state;          /**< текущее состояние реле */
};

/**
  * @brief  Генерация структуры реле
  * @details Генерация структуры реле модуля, с автоматическим 
  *         переходом в выключенное состояние.
  * @param[in]  port: порт выхода с реле
  * @param[in]  pin: пин выхода с реле
  * @param[in]  invert: флаг инверсии, значение на выходе 
  *         при выключенном состоянии
  * @retval Структуру, используемую для включения и выключения 
  *         реле
  */
struct relay_t relay_init(GPIO_TypeDef* port, uint16_t pin, bool invert);

/**
  * @brief  Установка значения реле
  * @param[in,out]  relay: реле
  * @param[in]  state: состояние в которое переводится реле
  * @retval None
  */
void relay_set_state(struct relay_t* relay, bool state);
/**
  * @brief  Смена значения реле на противоположное
  * @param[in,out]  relay: реле
  * @retval Установленное значение
  */
bool relay_toggle(struct relay_t* relay);
/**
  * @brief  Выключение реле
  * @param[in,out]  relay: реле
  * @retval None
  */
void relay_off(struct relay_t* relay);
/**
  * @brief  Включение реле
  * @param[in,out]  relay: реле
  * @retval None
  */
void relay_on(struct relay_t* relay);

/**
  * @brief  Получить состояния управляющего сигнала
  * @param[in,out]  relay: реле
  * @retval Состояние сигнала
  */
GPIO_PinState relay_pin_signal(struct relay_t* relay);

#endif // __DEVICE_RELAY_H__
