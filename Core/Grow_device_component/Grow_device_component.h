#ifndef __GROW_DEVICE_COMPONENT_H__
#define __GROW_DEVICE_COMPONENT_H__

#if defined( ESP32 )
#include <Arduino.h>
#else
#include <main.h>
#endif

// Количество всех типов устройств
#define COUNT_TYPE_DEVICE 6
// Список всех типов устройств
enum Type_device {
	Signal_PWM = 0,
	Signal_digital,
	Fan_PWM,
	Pumping_system,
	Phytolamp_digital,
	Phytolamp_PWM
};

class Grow_device_component {
private:
    enum Type_device type_device_; // тип устройства
    uint8_t id_;  // номер устройства данного типа
    uint16_t value_; // состояние устройства
public:
    Grow_device_component(Type_device type_device=Signal_digital, uint8_t id=0);
    ~Grow_device_component() = default;

    // получение типа датчика
    enum Type_device get_type() const;

    // получение id
    uint8_t get_id() const;
    // установка id
    void set_id(uint8_t id);

    // Получить значение считанного показателя
    uint16_t get_value() const;
    // Установить значение считанного показателя
    bool set_value(uint16_t value);

    // Проверка совпадения неизменяемых программно частей модуля
    bool filter(Grow_device_component &component);

    /// Функции превращения в байтовую строку (-) ----- доработать под код Дениса
    // Получение размера строки
    size_t get_size();
    // Заполение массива байтов, возврат количество байт (должен совпадать с размером строки)
    size_t get_data(uint8_t *data);
    // Заполнение объекта класса по байтовой строке, возврат количество использованных байт
    size_t set_data(uint8_t *data, size_t available_size);
};

#endif // __GROW_DEVICE_COMPONENT_H__
