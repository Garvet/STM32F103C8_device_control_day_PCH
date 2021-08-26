#ifndef __LORA_REGISTER_H__
#define __LORA_REGISTER_H__

#if defined( ESP32 )
#include <Arduino.h>
#include <iostream>
#include <SPI.h>
#include <array>
#else
#include <main.h>
#endif
#include <Address_field.h>
#include <LoRa_register_data.h>

#define AMT_FLAGS 8
#define LORA_DATA_SIZE 0x70

class LoRa_register {
// private:
public:
#if defined( ESP32 )
    SPIClass* _spi;
    SPISettings* _setting;
    uint8_t _nss;
#else
	SPI_HandleTypeDef* _spi;
    uint16_t _nss_pin;
    GPIO_TypeDef* _nss_port;
#endif
    uint8_t _registers_data[LORA_DATA_SIZE];
    bool   _registers_state[LORA_DATA_SIZE]; // 0 - неизвестно, 1 - считано
    bool _send; // = False

    Address_field _flags[AMT_FLAGS] = {RxTimeout, RxDone, PayloadCrcError, ValidHeader, TxDone, CadDone, FhssChangeChannel, CadDetected};

    uint8_t _read_register(uint8_t address);
    void _write_register(uint8_t address, uint8_t value);
    uint8_t _single_transfer(uint8_t address, uint8_t value);

    std::array<uint8_t, 80> result_field_registers_data;
    uint8_t result_field_registers_len;

    std::array<uint8_t, 80> result_check_missing_register_data;
    uint8_t result_check_missing_register_len;


    std::array<uint8_t, 80> reg_read;
    uint8_t reg_read_len;
    std::array<uint8_t, 80> reg_not_read;
    uint8_t reg_not_read_len;

    // Выдаёт адреса регистров, относящихся к данному полю
    uint8_t field_registers(Address_field field);
    uint8_t field_registers(Address_field* fields, uint8_t amt_fields);
    // Проверка получения номеров регистров, которые нужно считать для данного списка полей
    uint8_t check_missing_register(Address_field field);
    uint8_t check_missing_register(Address_field* fields, uint8_t amt_fields);

    // Проверка на необходимость считывания перед записью новых значений (т.е. занимают ли поля весь объём данных регистра)
    bool check_read(Address_field field);
    bool check_read(Address_field* fields, uint8_t amt_fields);

public:
    LoRa_register();
    ~LoRa_register();
#if defined( ESP32 )
    LoRa_register(SPIClass* spi, SPISettings* setting, uint8_t nss);
    void init(SPIClass* spi, SPISettings* setting, uint8_t nss);
#else
    LoRa_register(SPI_HandleTypeDef* spi, GPIO_TypeDef* nss_port, uint16_t nss_pin);
    void init(SPI_HandleTypeDef* spi, GPIO_TypeDef* nss_port, uint16_t nss_pin);
#endif

    void clear();
    bool get_send();

    // считывает регистры относящиеся к полю(ям)
    uint8_t register_read(Address_field field, bool update=true);
    uint8_t register_read(Address_field* fields, uint8_t amt_fields, bool update=true);
    // записывает регистры относящиеся к полю(ям)
    uint8_t register_write(bool clear=true);
    uint8_t register_write(Address_field field, bool clear=true, bool error_clear=false);
    uint8_t register_write(Address_field* fields, uint8_t amt_fields, bool clear=true, bool error_clear=false);
    // Установка значения(ий) полю(ям)
    uint8_t set_field_value(Address_field field, uint32_t value);
    uint8_t set_field_value(Address_field* fields, uint32_t* values, uint8_t amt);
    // Получение значения(ий) поля(ей)
    uint8_t get_field_value(Address_field field, uint32_t* value, bool read=false);
    uint8_t get_field_value(Address_field* fields, uint32_t* values, uint8_t amt, bool read=false);
    // Сброс значения(ий) флага(ов)
    uint8_t clear_flags(Address_field flag, bool back_value=false);
    uint8_t clear_flags(Address_field* flags, uint8_t amt_flags, bool back_value=false);
};

#endif // __LORA_REGISTER_H__
