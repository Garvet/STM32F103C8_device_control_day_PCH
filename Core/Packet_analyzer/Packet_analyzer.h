#ifndef __PACKET_ANALYZER_H__
#define __PACKET_ANALYZER_H__


#if defined ( ESP32 )
#include <Arduino.h>
#endif
#include <Address_field.h>
#include <LoRa_packet.h>

#define AMT_BYTES_SYSTEM_ID 12
#define PACKET_HEADER_SIZE 9
extern const uint16_t LORA_ADDRESS_BRANCH;

// #define PACKET_CONNECTION 0x00
// #define PACKET_SENSOR 0x01
// #define PACKET_DEVICE 0x02
// #define PACKET_SYSTEM 0x03

// (?) ----- использовать пространство?

// Адресация в LoRa-сети
// (!) ----- Поменяй глобальные адреса на рассчитываемые
#define LORA_GLOBAL_ADDRESS    0x01FF7FFF
#define LORA_GLOBAL_ADR_GROUP  0x1FF
#define LORA_GLOBAL_ADR_BRANCH 0x7FFF
class LoRa_address {
public:
    uint16_t group = 0xFFFF;  // Адрес группы
    uint16_t branch = 0xFFFF; // Адрес ветви
public:
    LoRa_address() = default;
    LoRa_address(const uint8_t* bytes, const uint8_t len=3); // массив из 3-х
    LoRa_address(const uint16_t group, const uint16_t branch);
    LoRa_address(const uint32_t adr);
    LoRa_address(const LoRa_address &adr) = default;
    ~LoRa_address() = default;

    bool global();

    friend bool operator==(const LoRa_address& left, const LoRa_address& right);
    friend bool operator!=(const LoRa_address& left, const LoRa_address& right);
    friend bool operator==(const LoRa_address& left, const uint32_t& right);
    friend bool operator!=(const LoRa_address& left, const uint32_t& right);
    friend bool operator==(const uint32_t& left, const LoRa_address& right);
    friend bool operator!=(const uint32_t& left, const LoRa_address& right);
};

// Пакеты в LoRa-сети
// Типы пакетов
enum class Packet_Type {
    CONNECTION = 0x00,
    SENSOR,
    DEVICE,
    SYSTEM
};

class Packet_analyzer {
protected:
    const Address_field **field_header_; // поля шапки
    const Address_field **field_packet_; // поля пакета
    uint16_t amt_field_header_; // количество полей шапки
    uint16_t amt_field_packet_; // количество полей пакета

    uint8_t setting_; // настройки передатчика (общие)

    size_t last_filled_byte = 0; // номер последнего заполненного байта пакета (не шапки)
    size_t last_read_byte = 0;  // номер последнего прочитанного байта пакета (не шапки)

    // // LoRa_packet* packet;
    // uint8_t lenght_;
    // uint8_t receiv_lenght_;
    // uint8_t send_lenght_;
    // Address_field **field_;
    // uint16_t count_field_;
    // uint16_t max_address_;
    // uint8_t setting_;
    // bool creat_packet_;
    // bool set_field(Address_field **field, uint16_t count_field);
public:
    Packet_analyzer();
    ~Packet_analyzer()=default;
    // bool select_packet(LoRa_packet* packet);
    // uint8_t get_receiv_lenght();
    // uint8_t get_send_lenght();

    uint16_t     get_dest_adr_group (const LoRa_packet& packet);  // Адрес группы адресанта
    uint16_t     get_dest_adr_branch(const LoRa_packet& packet);  // Адрес ветви  адресанта
    LoRa_address get_dest_adr       (const LoRa_packet& packet);  // Адрес адресанта
    uint16_t     get_sour_adr_group (const LoRa_packet& packet);  // Адрес группы отправителя
    uint16_t     get_sour_adr_branch(const LoRa_packet& packet);  // Адрес ветви  отправителя
    LoRa_address get_sour_adr       (const LoRa_packet& packet);  // Адрес отправителя
    Packet_Type  get_packet_type    (const LoRa_packet& packet);  // Тип пакета
    uint16_t     get_packet_number  (const LoRa_packet& packet);  // Номер пакета

    bool set_dest_adr_group (LoRa_packet& packet, uint16_t adr);       // Адрес группы адресанта
    bool set_dest_adr_branch(LoRa_packet& packet, uint16_t adr);       // Адрес ветви  адресанта
    bool set_dest_adr       (LoRa_packet& packet, LoRa_address adr);   // Адрес адресанта
    bool set_sour_adr_group (LoRa_packet& packet, uint16_t adr);       // Адрес группы отправителя
    bool set_sour_adr_branch(LoRa_packet& packet, uint16_t adr);       // Адрес ветви  отправителя
    bool set_sour_adr     (LoRa_packet& packet, LoRa_address adr);     // Адрес отправителя
    bool set_packet_type  (LoRa_packet& packet, Packet_Type pac_type); // Тип пакета
    bool set_packet_type  (LoRa_packet& packet, uint8_t pac_type);     // Тип пакета
    bool set_packet_number(LoRa_packet& packet, uint16_t num);         // Номер пакета

    virtual bool set_setting(uint8_t setting=0);
    uint8_t get_setting();
};


class Packet_Connection: public Packet_analyzer {
private:
    uint8_t command_ = 0;
public:
    virtual bool set_setting(uint8_t setting=0); // установить настройки

    /// --- Запись в пакет ---
    // Установить команду
    uint8_t set_command(LoRa_packet& packet, uint8_t com);
    // Установить данные
    uint8_t set_data(LoRa_packet& packet, uint8_t *data, uint8_t len);
    // Занести в пакет параметры и данные
    uint8_t set_packet_data(LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len);

    /// --- Чтение из пакета ---
    // Получить команду
    uint8_t get_command(const LoRa_packet& packet, uint8_t *com);
    // Получить данные
    uint8_t get_data(const LoRa_packet& packet, uint8_t *data, uint8_t *len);
    // Получить из пакета параметры и данные
    uint8_t get_packet_data(const LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len);

    /// --- Расчёты ---
    // Узнать объём поля данных по параметрам
    uint8_t get_size_by_data(const uint8_t *com, const uint8_t *len, uint8_t &size_data);
    // Узнать объём поля данных по содержимому пакета
    uint8_t get_size_by_packet(const LoRa_packet& packet, uint8_t &size_data);
};

class Packet_Sensor: public Packet_analyzer {
private:
    uint8_t param_;
public:
    virtual bool set_setting(uint8_t setting=0); // установить настройки (-) ----- нет get

    // занести в пакет параметры и данные
    uint8_t set_packet_data(LoRa_packet& packet, uint8_t *amt, uint8_t *param, uint8_t *id, uint32_t *data);
    // получить из пакета параметры и данные
    uint8_t get_packet_data(const LoRa_packet& packet, uint8_t *amt, uint8_t *param, uint8_t *id, uint32_t *data);

    // узнать объём поля данных по параметрам
    uint8_t get_size_by_data(const uint8_t *amt, const uint8_t *param, uint8_t &size_data);
    // узнать объём поля данных по содержимому пакета
    uint8_t get_size_by_packet(const LoRa_packet& packet, uint8_t *amt, uint8_t *param, uint8_t &size_data);
};

class Packet_Device: public Packet_analyzer {
private:
    uint8_t object_;
    uint8_t command_;
public:
    virtual bool set_setting(uint8_t setting = 0); // установить настройки (-) ----- нет get

    // занести в пакет параметры и данные
    uint8_t set_packet_data(LoRa_packet& packet, uint8_t *obj, uint8_t *num, uint8_t *com, uint8_t *data, uint8_t *len);
    // получить из пакета параметры и данные
    uint8_t get_packet_data(const LoRa_packet& packet, uint8_t *obj, uint8_t *num, uint8_t *com, uint8_t *data, uint8_t *len);

    // узнать объём поля данных по параметрам
    uint8_t get_size_by_data(const uint8_t *obj, const uint8_t *com, uint8_t &size_data);
    // узнать объём поля данных по содержимому пакета
    uint8_t get_size_by_packet(const LoRa_packet& packet, uint8_t *obj, uint8_t &size_data);
};

class Packet_System: public Packet_analyzer {
private:
    uint8_t command_;
public:
    virtual bool set_setting(uint8_t setting=0); // установить настройки (-) ----- нет get

    // занести в пакет параметры и данные
    uint8_t set_packet_data(LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len);
    // получить из пакета параметры и данные
    uint8_t get_packet_data(const LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len);

    // узнать объём поля данных по параметрам
    uint8_t get_size_by_data(const uint8_t *com, const uint8_t *len, uint8_t &size_data);
    // узнать объём поля данных по содержимому пакета
    uint8_t get_size_by_packet(const LoRa_packet& packet, uint8_t &size_data);
};

extern Packet_analyzer packet_analyzer;
extern Packet_Connection packet_connection;
extern Packet_Sensor packet_sensor;
extern Packet_Device packet_device;
extern Packet_System packet_system;

#endif // __PACKET_ANALYZER_H__
