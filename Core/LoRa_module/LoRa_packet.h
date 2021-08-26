#ifndef __LORA_PACKET_H__
#define __LORA_PACKET_H__

#if defined( ESP32)
#include <Arduino.h>
#else
#include <main.h>
#endif
#include <vector>

#define USE_STANDARD_ARRAY
#if defined( USE_STANDARD_ARRAY )
#include <array>
#include <algorithm>
#endif

#if defined( ESP32 )
constexpr size_t SIZE_LORA_PACKET_MAX_LEN = 250; // - 254 max
constexpr size_t SIZE_LORA_PACKET_BUFFER = 100;
#else
constexpr size_t SIZE_LORA_PACKET_MAX_LEN = 50; // - 254 max
constexpr size_t SIZE_LORA_PACKET_BUFFER = 80;
#endif

class LoRa_packet_data;
class LoRa_packet;

class LoRa_packet_data {
private:
    bool free_object_ = true; // Свободный объект
public:
    uint8_t len = 0; // Количество байт
#if defined( USE_STANDARD_ARRAY )
    std::array<uint8_t, SIZE_LORA_PACKET_MAX_LEN> data;
#else
    uint8_t data[SIZE_LORA_PACKET_MAX_LEN]; // Байты
#endif

    LoRa_packet_data() = default;

    bool add_data(uint8_t data_byte);
    bool add_data(const uint8_t* data_byte, uint8_t amt_byte);

    bool set_data(const uint8_t* data, uint8_t len);
    bool set_data(const std::vector<uint8_t>& data);
    void set_data(const class LoRa_packet& lora_packet);
    void set_data(const class LoRa_packet_data& lora_packet);
    void set_data(class LoRa_packet_data&& lora_packet);

    bool free() const;

    class LoRa_packet_data& operator=(const class LoRa_packet& right);
    class LoRa_packet_data& operator=(const class LoRa_packet_data& right);

    friend class LoRa_packet;
};

class LoRa_packet {
// private: (-) -----
public:
    LoRa_packet_data* packet_data;
    uint8_t rssi_ = 0;  // RSSI соединения
    bool crc_error_   = false; // Ошибка контрольной суммы

    friend class LoRa_packet_data;
    bool search_data();
public:
    LoRa_packet();
    LoRa_packet(const uint8_t* data, uint8_t len, bool crc_err=false, uint8_t rssi=0);
    LoRa_packet(const std::vector<uint8_t>& data, bool crc_err=false, uint8_t rssi=0);
    LoRa_packet(const LoRa_packet& right);
    LoRa_packet(LoRa_packet&& right);
    ~LoRa_packet();

    // Функция заполенния объекта
    bool add_packet_data(uint8_t data);
    bool add_packet_data(const uint8_t* data, uint8_t len);
    bool set_packet(const uint8_t* data, uint8_t len, bool crc_err=false, uint8_t rssi=0);
    bool set_packet(const std::vector<uint8_t>& data, bool crc_err=false, uint8_t rssi=0);

    // Очистка пакета
    void clear_packet();

    // получение содержимого пакета
    LoRa_packet_data* get_packet() const;
    std::vector<uint8_t> get_data() const; // (-) -----
    // получение одного байта
    uint8_t get_data(int num) const;
    // получение длины пакета
    uint8_t get_len() const;
    // получение ошибки передачи пакета
    bool    get_crc_error() const;
    // получение RSSI пакета
    uint8_t get_rssi() const;

    // получение одного байта
    uint8_t& operator[](const int index);
    const uint8_t& operator[](const int index) const;
    // перегрузка оператора копирования
    LoRa_packet& operator=(const LoRa_packet& right);
    // перегрузка оператора перемещения
    LoRa_packet& operator=(LoRa_packet&& right);

}typedef LoRa_packet;


#endif // __LORA_PACKET_H__
