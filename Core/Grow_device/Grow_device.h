#ifndef __GROW_DEVICE_H__
#define __GROW_DEVICE_H__

#include <vector>
#include "../Grow_device_component/Grow_device_component.h"

// (!) ----- (-) ----- перенести на статическую память

#include <Packet_analyzer.h>

class Grow_device {
private:
	std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id_;
    uint16_t address_ = 0xFFFF; // адрес датчика
    std::vector<Grow_device_component> component_; // вектор компонентов датчиков (не как плата, а как механический модуль)

#if !defined( ESP32 )
    LoRa_address address_control_module_;
#endif

    unsigned long read_time_; // время последнего считывания
    unsigned long end_time_; // время последней проверки для считывания
    unsigned long period_; // период считывания в мс
    bool readout_signal_; // флаг готовности считывания
    uint8_t active_;  // состояние активации 0 - не активен, 1 - в процессе регистрации, 2 - зарегистрирован // (-) ----- обнулить значения при false запретить функции обработки
    bool change_value_; // флаг изменённого значения считанных показателей

    bool sleep_state{false};
    uint8_t setting_; // настройски (для LoRa-протокола)

    bool rtc_sync = false;

    friend class Grow_device_interface;
public:
    Grow_device() = default;
    Grow_device(uint8_t amt_component, enum Type_device* type_device);
    Grow_device(uint8_t amt_component, uint8_t* type_device);
    Grow_device(const std::vector<enum Type_device>& type_device);
    ~Grow_device() = default;

    /// --- Поля класса-платы ---

    // установка индивидуального номера платы
    void set_system_id(std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id);
    // получение индивидуального номера платы
    std::array<uint8_t, AMT_BYTES_SYSTEM_ID> get_system_id() const;

    // установка состояния активности
    void set_active(uint8_t active);
    // получение состояния активности
    uint8_t get_active() const;

    // установка режима
    void set_sleep_state(bool state);
    // получение режима
    bool get_sleep_state() const;

    // получение флага об изменённом состоянии)
    bool get_change_value() const;
    // сброс флага изменённого состояния
    void clear_change_value();

    // установка адреса модуля (ветви, без группы)
    bool set_address(uint16_t address);
    // получение адреса модуля (ветви, без группы)
    uint16_t get_address() const;

#if !defined( ESP32 )
    // установка адреса МУГа ()
    bool set_address_control_module(LoRa_address address);
    // получение адреса МУГа
    LoRa_address get_address_control_module() const;
#endif

    // Установка настроек LoRa-передачи
    void set_setting(uint8_t setting);
    // Получение настроек LoRa-передачи
    uint8_t get_setting() const;

    /// --- Обработка времени ---

    // Установить период опроса модуля
    void set_period(unsigned long period);
    // Получить период опроса модуля
    unsigned long get_period() const;

    // Проверка необходимости считывания
    bool check_time(unsigned long time);
    // Обновление времени считывания
    void update();
    // Получение сигнала готовности к считыванию
    bool read_signal(bool clear=false);


    /// --- Поля компонентов ---

    // получение типа компонента (передаётся в result), если ошибка возврат true
    bool get_type(uint8_t num, enum Type_device &result) const;
    bool get_type(uint8_t num, uint8_t &result) const;
    // получение вектора типов компонентов
    std::vector<enum Type_device> get_type() const;
    // получение id компонента (передаётся в result | не путать с ID платы, этот номер для количества повторений), если ошибка возврат true
    bool get_id(uint8_t num, uint8_t &result) const;
    // получение вектора id компонентов
    std::vector<uint8_t> get_id() const;

    // Установить значение считанного показателя, если ошибка возврат true
    bool set_value(uint8_t num, uint16_t value);
    // Получить значение считанного показателя, если ошибка возврат true
    bool get_value(uint8_t num, uint16_t &result);
    // Установить значения считанных показателей, если ошибка возврат true
    bool set_value(const std::vector<uint16_t>& value);
    // Получить вектор значений считанных показателей
    std::vector<uint16_t> get_value();

    // Установить флаг необходимости синхронизации времени
    void set_rtc_sync(bool flag);
    // Получить флаг необходимости синхронизации времени
    bool get_rtc_sync() const;

    /// --- Информации о компонентах ---

    // Получить количество компонентов
    uint8_t get_count_component() const;

    // Получить определённый компонент
    Grow_device_component get_component(uint8_t num) const;
    // Получить вектор компонентов
    std::vector<Grow_device_component> get_component() const;

    /// --- Внешняя связь ---

    // Проверить совподает ли содержимое модулей, без учёта настроек (для отфильтровывания среди неподходящих)
    bool filter(Grow_device &device); // (-) ----- убрать привязку к порядку

};


#endif // __GROW_DEVICE_H__



#ifdef NOT_USE

#ifndef __GROW_DEVICE_H__
#define __GROW_DEVICE_H__

#if defined( ESP32 )
#include <Arduino.h>
#define SERIAL_LOG_OUTPUT
#else
#include <main.h>
#endif

#include <vector>
#include "../Grow_device_component/Grow_device_component.h"

#include <array>
#include <Packet_analyzer.h>

// (!) ----- (-) ----- перенести на статическую память

class Grow_device {
private:
	std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id_;
    uint16_t address_ = 0xFFFF; // адрес датчика
    std::vector<Grow_device_component> component_; // вектор компонентов датчиков (не как плата, а как механический модуль)

#if !defined( ESP32 )
    LoRa_address address_control_module_;
#endif

    unsigned long read_time_; // время последнего считывания
    unsigned long end_time_; // время последней проверки для считывания
    unsigned long period_; // период считывания в мс
    bool readout_signal_; // флаг готовности считывания
    uint8_t active_;  // состояние активации 0 - не активен, 1 - в процессе регистрации, 2 - зарегистрирован // (-) ----- обнулить значения при false запретить функции обработки
    bool change_value_; // флаг изменённого значения считанных показателей

    uint8_t setting_; // настройски (для LoRa-протокола)

    friend class Grow_device_interface;
public:
    Grow_device() = default;
    Grow_device(uint8_t amt_component, enum Type_device* type_device);
    Grow_device(uint8_t amt_component, uint8_t* type_device);
    Grow_device(const std::vector<enum Type_device>& type_device);
    ~Grow_device() = default;

    /// --- Поля класса-платы ---

    // установка индивидуального номера платы
    void set_system_id(std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id);
    // получение индивидуального номера платы
    std::array<uint8_t, AMT_BYTES_SYSTEM_ID> get_system_id() const;

    // установка состояния активности
    void set_active(uint8_t active);
    // получение состояния активности
    uint8_t get_active() const;

    // получение флага об изменённом состоянии)
    bool get_change_value() const;
    // сброс флага изменённого состояния
    void clear_change_value();

    // установка адреса модуля (ветви, без группы)
    bool set_address(uint16_t address);
    // получение адреса модуля (ветви, без группы)
    uint16_t get_address() const;

#if !defined( ESP32 )
    // установка адреса МУГа ()
    bool set_address_control_module(LoRa_address address);
    // получение адреса МУГа
    LoRa_address get_address_control_module() const;
#endif

    // Установка настроек LoRa-передачи
    void set_setting(uint8_t setting);
    // Получение настроек LoRa-передачи
    uint8_t get_setting() const;

    /// --- Обработка времени ---

    // Установить период опроса модуля
    void set_period(unsigned long period);
    // Получить период опроса модуля
    unsigned long get_period() const;  

    // Проверка необходимости считывания 
    bool check_time(unsigned long time);
    // Обновление времени считывания
    void update();
    // Получение сигнала готовности к считыванию
    bool read_signal(bool clear=false);


    /// --- Поля компонентов ---

    // получение типа компонента (передаётся в result), если ошибка возврат true
    bool get_type(uint8_t num, enum Type_device &result) const;
    bool get_type(uint8_t num, uint8_t &result) const;
    // получение вектора типов компонентов
    std::vector<enum Type_device> get_type() const;
    // получение id компонента (передаётся в result | не путать с ID платы, этот номер для количества повторений), если ошибка возврат true
    bool get_id(uint8_t num, uint8_t &result) const;
    // получение вектора id компонентов
    std::vector<uint8_t> get_id() const;

    // Установить значение считанного показателя, если ошибка возврат true
    bool set_value(uint8_t num, float value);
    bool set_value(uint8_t num, uint32_t value);
    // Получить значение считанного показателя, если ошибка возврат true
    bool get_value(uint8_t num, float &result) const;
    bool get_value(uint8_t num, uint32_t &result) const;
    // Установить значения считанных показателей, если ошибка возврат true
    bool set_value(const std::vector<float>& value);
    // Получить вектор значений считанных показателей
    std::vector<float> get_value() const;

    /// --- Информации о компонентах ---

    // Получить количество компонентов
    uint8_t get_count_component() const;

    // Получить определённый компонент
    Grow_device_component get_component(uint8_t num) const;
    // Получить вектор компонентов
    std::vector<Grow_device_component> get_component() const;

    /// --- Внешняя связь ---

    // Проверить совподает ли содержимое модулей, без учёта настроек (для отфильтровывания среди неподходящих)
    bool filter(Grow_device &device); // (-) ----- убрать привязку к порядку

#if defined (SERIAL_LOG_OUTPUT)
    /// вывод класса в Serial
    void print();
#endif
};


#endif // __GROW_DEVICE_H__

#endif
