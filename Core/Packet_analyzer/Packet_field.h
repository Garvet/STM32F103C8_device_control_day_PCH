#ifndef __PACKET_FIELD_H__
#define __PACKET_FIELD_H__

#if defined ( ESP32 )
#include <Arduino.h>
#endif
#include <Address_field.h>

// Шапка пакетов, адресация, номера и типы
const uint16_t LORA_ADDRESS_GROUP = 9;
const uint16_t LORA_ADDRESS_BRANCH = 24 - LORA_ADDRESS_GROUP;

const Register Hendler[11] = {Register(0, 8, 0), Register(1, 1, 7), Register(1, 7, 0), Register(2, 8, 0),
                              Register(3, 8, 0), Register(4, 1, 7), Register(4, 7, 0), Register(5, 8, 0),
                              Register(6, 8, 0),
                              Register(7, 8, 0), Register(8, 8, 0)};

const Address_field field_dest_adr_group(&Hendler[0], 2, true),  // Destination Address group (адресс группы устройства назначения)
                    field_dest_adr_branch(&Hendler[2], 2, true), // Destination Address branch (адресс ветви устройства назначения)
                    field_sour_adr_group(&Hendler[4], 2, true),  // Source Address group (адресс группы отправляющего устройства)
                    field_sour_adr_branch(&Hendler[6], 2, true), // Source Address branch (адресс ветви отправляющего устройства)
                    field_type(&Hendler[8], 1, true),   // Packet Type (тип пакетп)
                    field_number(&Hendler[9], 2, true); // Packet Number (номер пакета)


const uint8_t num_dest_adr_group = 0,  // Destination Address group
              num_dest_adr_branch = 1, // Destination Address branch
              num_sour_adr_group = 2,  // Source Address group
              num_sour_adr_branch = 3, // Source Address branch
              num_type = 4,   // Packet Type
              num_number = 5; // Packet Number

const uint16_t count_field_packet_header = 6;
const Address_field *field_packet_header[6] = {&field_dest_adr_group, &field_dest_adr_branch,
                                               &field_sour_adr_group, &field_sour_adr_branch,
                                               &field_type, &field_number};


// Типы данных
const Register register_PWM[2]   = {Register(0, 4, 0), Register(1, 8, 0)},
               register_byte[1]  = {Register(0, 8, 0)},
               register_float[4] = {Register(0, 8, 0), Register(1, 8, 0), Register(2, 8, 0), Register(3, 8, 0)},
               register_bit[8]   = {Register(0, 1, 0), Register(0, 1, 1), Register(0, 1, 2), Register(0, 1, 3),
                                    Register(0, 1, 4), Register(0, 1, 5), Register(0, 1, 6), Register(0, 1, 7)};

const Address_field field_PWM(&register_PWM[0], 2, true),
                    field_byte(&register_byte[0]),
                    field_float(&register_float[0], 4, false),
                    field_bit[8] = {Address_field(&register_bit[0]), Address_field(&register_bit[1]),
                                    Address_field(&register_bit[2]), Address_field(&register_bit[3]),
                                    Address_field(&register_bit[4]), Address_field(&register_bit[5]),
                                    Address_field(&register_bit[6]), Address_field(&register_bit[7])};


// ----- connect (пакеты контакта) -----
const uint8_t CONNECT_COMMAND_DATA = 0x0C + 1; 
enum class Connect_packet {
    ESTABLISHING_CONNECTION_REQUEST = 0,      // установка соединения (запрос)
    ESTABLISHING_CONNECTION_RESPONSE,         // установка соединения (ответ)
    RESET_WAIT_CONNECTION,                    // сброс ожидания установки соединения
    PACKETS_TRANSFER_ACKNOWLEDGMENT_REQUEST,  // подтверждение передачи пакетов (запрос)
    PACKETS_TRANSFER_ACKNOWLEDGMENT_RESPONSE, // подтверждение передачи пакетов (ответ)
    PACKET_NUMBERS_REQUEST,                   // передача номеров пакетов (запрос)
    PACKET_NUMBERS_RESPONSE,                  // передача номеров пакетов (ответ)
    BREAK_CONNECTION_REQUEST,                 // разрыв соединения (запрос)
    BREAK_CONNECTION_RESPONSE,                // разрыв соединения (ответ)
    TRANSMITTER_SETTINGS_NEXT_PACKET,         // дополнительные настройки (для следующего пакета)
    TRANSMITTER_SETTINGS,                     // настройки передатчика (для всех пакетов)
    TRANSMITTER_STATUS_REQUEST,               // состояние передатчика (запрос)
    TRANSMITTER_STATUS_RESPONSE               // состояние передатчика (ответ)
};
const uint8_t connect_com_data[CONNECT_COMMAND_DATA] = {1, 0xFF, 0, 0, 1, 0, 0xFF, 0, 1, 0xFF, 0xFF, 0, 0xFF};

// ----- sensor (пакеты датчиков) -----
const uint8_t SENSOR_PARAM_DATA = 0x0C + 1;
// типы датчиков находятся в "Grow_sensor_component.h"
const uint8_t sensor_param_data[SENSOR_PARAM_DATA] = {2, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

// ----- device (пакеты устройств) -----
const uint8_t DEVICE_OBJECT = 0x08 + 1;
// типы устройств находятся в "Grow_device_component.h"
enum class Command_code {
    // device_com_data_0 - device_com_data_5
    REQUEST_VALUE = 0, // запросить значение
    SEND_VALUE,        // отправить значение
    // device_com_data_6
    REQUEST_TIME = 0, // запросить время
    REQUEST_DATA,     // запросить дату
    SEND_TIME,        // отправить время
    SEND_DATA,        // отправить дату
    // device_com_data_7 - TimerChannel (2 периода, включено и выключено, по 4 байта, в секундах)
    // запросить количество каналов
    // отправить количество каналов
    // запросить настройки канала
    // отправить настройки канала
    // device_com_data_8 - GrowTimer  (1 период, отталкивается от настроек времени, начала, длительности и смещения периода)
    // запросить количество каналов
    // отправить количество каналов
    // запросить настройки канала
    // отправить настройки канала
    // запросить состояния сигналов
    // отправить данные о состоянии сигналов
};
const uint8_t DEVICE_COMMAND[DEVICE_OBJECT] = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x04, 0x04, 0x06};
const uint8_t device_com_data_0[0x02] = {0, 2},                // ШИМ сигнал
              device_com_data_1[0x02] = {0, 1},                // Цифровой сигнал
              device_com_data_2[0x02] = {0, 2},                // Вентилятор
              device_com_data_3[0x02] = {0, 1},                // Насосная система
              device_com_data_4[0x02] = {0, 1},                // Фитолампа (0|1)
              device_com_data_5[0x02] = {0, 2},                // Фитолампа (ШИМ)
              device_com_data_6[0x04] = {0, 0, 3, 3},          // RTC
              device_com_data_7[0x04] = {0, 1, 1, 8},          // TimeChannel
              device_com_data_8[0x06] = {0, 1, 1, 3, 0, 0xFF}; // Grow_timer | Day_management

const uint8_t* device_object_data[DEVICE_OBJECT] = {device_com_data_0, device_com_data_1, device_com_data_2,
                                                    device_com_data_3, device_com_data_4, device_com_data_5,
                                                    device_com_data_6, device_com_data_7};

// ----- system (системные пакеты) -----
const uint8_t SYSTEM_COMMAND_DATA = 0x07 + 1;
const uint8_t system_com_data[SYSTEM_COMMAND_DATA] = {0xFF, 3+AMT_BYTES_SYSTEM_ID, 2, 0, 5, 2, 3, 0xFF};
// const uint8_t SYSTEM_COMMAND_DATA = 0x11;
// uint8_t system_com_data[SYSTEM_COMMAND_DATA] = {0xFF, 0, 0xFF, 3, 0, 3, 3, 2, 5, 0xFF, 3, 3, 0, 1, 2, 6, 6};

#endif // __PACKET_FIELD_H__
