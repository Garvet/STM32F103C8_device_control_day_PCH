#ifndef __LORA_CONTACT_DATA_H__
#define __LORA_CONTACT_DATA_H__

#if defined( ESP32 )
#include <Arduino.h>
#include <Wire.h>
#define USE_VECTOR
#endif

#if defined( USE_VECTOR )
#include <vector>
#else
#include <array>
#endif

#include <LoRa.h>
// #include <RtcDS3231.h>

#include <Packet_analyzer.h>
#include <LoRa_packet.h>
#include <stdbool.h>

// #include <Grow_sensor.h>
// #include <Grow_device.h>

#if !defined ( USE_VECTOR )
#define CONTACT_DATA_MAX_PACKET 15
#endif

// Состояние модуля передачи
enum module_state {
    MS_LoRa_SLEEP = 0, // Не в процессе обмена
    MS_LoRa_SEND,      // В процессе отправки пакета (ожидаем сигнала на DIO 0)
    MS_LoRa_RECIEVE    // В процессе приёма пакета (ожидаем сигнала на DIO 0,1)
};

// Тип соединения
enum type_communication {
    TC_INITIATOR = 0, // ESP инициатор
    TC_RECIPIENT,     // ESP получатель
    TC_BROADCAST      // ESP проводит трансляцию
};
// Стадии общения
enum stade_communication {
    // Соединение
    SC_DOWNTIME = 0, // Простой, нет контакта
    SC_CONNECTION,   // Установка соединения
    SC_EXCHANGE,     // Обмен пакетами
    SC_DISCONNECT,   // Разрыв соединения
    SC_COMPLETE,     // Контакт завершён
    // Трансляция
    // SC_DOWNTIME = 0,    // Простой              (дубль)
    SC_BROADCASTING,    // Трансляция пакетов в сеть
    SC_WAITING,         // Ожидание пакетов
    SC_PACKET_ACCEPTED, // Был принят хоть один пакет
    SC_REPLY_BROADCAST  // Трансляция во время приёма  (тоже, что и SC_BROADCASTING, но под конец переходит в SC_WAITING или SC_PACKET_ACCEPTED)
    // SC_COMPLETE         // Отправка завершена   (дубль)
};
// Подстадии соединения "Установка соединения"
enum connection {
    // INITIATOR
    C_EXPECTATION = 0,  // Ожидание реакции на запрос
    C_WAIT_RESET,       // Ожидание сброса пакета
    // RECIPIENT
    C_QUEUE_CHECK = 0,  // Проверка очереди на соединение
    C_AWAITING_REQUEST, // Ожидание запроса на соединение
    // C_CONFIRMATION      // Подтверждение установки соединения
};
// Подстадии соединения "Обмен пакетами"
enum exchange {
    // INITIATOR
    E_BOARDCAST = 0,   // Передача пакетов
    E_WAITING_CONFIRMATION, // Ожидание подтверждения получения пакетов
    E_WAITING_NUMBERS, // Ожидание номеров полученных пакетов
    // RECIPIENT
    E_EXPECTATION = 0, // Ожидание пакетов
    E_WAITING_REACTION // Ожидание реакции на подтверждение
};
// Подстадии соединения "Разрыв соединения"
enum disconnect {
    // INITIATOR
    // RECIPIENT
    D_WAITING_CONNECTION_BREAK = 0 // Ожидание разрыва соединения
};
// Структура, содержащая стадии
struct Stage_control {
    enum type_communication  type_communication;
    enum stade_communication stade_communication;
    enum connection          connection;
    enum exchange            exchange;
    enum disconnect          disconnect;
};

class LoRa_contact_data {
private:
    /// LoRa-модуль
#if defined( ESP32 )
    LoRa lora_; // объект класса
#endif
    bool lora_init_ = false; // инициализация прошла успешно
    bool lora_begin_ = false; // запуск модуля прошёл успешно
    uint16_t channel_ = 0; // канал работы (0-й - регистрация)
    /// Адреса системы (этого модуля, модуля для соединения, отложенных модулей)
    LoRa_address my_adr_; // адрес этого модуля
    LoRa_address connect_adr_ = LoRa_address(LORA_GLOBAL_ADR_GROUP, LORA_GLOBAL_ADR_BRANCH); // адрес назначения (изн. широковещательный)
    LoRa_address wait_adr_ = LoRa_address(LORA_GLOBAL_ADR_GROUP, LORA_GLOBAL_ADR_BRANCH);  // адрес ожидания для приёма (изн. от всех)
    uint16_t num_end_packet_ = 0;
    /// Состояния работы системы (подробно см. выше)
    enum module_state module_state_;
    Stage_control current_stage_;
    Stage_control past_stage_;
    bool use_past_stage = false;
    /// пакеты
    LoRa_packet last_receive_packet_; // Принятый пакет
    LoRa_packet last_send_packet_;    // Пакет для отправки
    // class Exchange_packet last_receive_packet_; // Принятый пакет
    // class Exchange_packet last_send_packet_;    // Пакет для отправки
    uint8_t amt_packet_ = 0;          // количество принятых пакетов
    uint8_t expected_amt_packet_ = 0; // ожидаемое количество принятых пакетов

#if defined( USE_VECTOR )
    std::vector<LoRa_packet> send_packet_;   // Отправляемые пакеты
    std::vector<bool> send_flag_;            // Флаг был ли отправлен пакет
    std::vector<LoRa_packet> reciev_packet_; // Принятые пакеты
#else
    std::array<LoRa_packet, CONTACT_DATA_MAX_PACKET> send_packet_;   // Отправляемые пакеты
    std::array<bool, CONTACT_DATA_MAX_PACKET> send_flag_;            // Флаг был ли отправлен пакет
    std::array<LoRa_packet, CONTACT_DATA_MAX_PACKET> reciev_packet_; // Принятые пакеты
    uint8_t send_packet_len = 0;
    uint8_t reciev_packet_len = 0;
    uint8_t send_flag_len = 0;
#endif

    bool start_connect_ = false;

    bool init_ = false; // инициализирован старт для контакта

    // Очищает все данные о пакетах
    void clear();

    /// --- --- --- Работа системы --- --- ---
#if defined ( ESP32 )
    std::vector<LoRa_address> wait_connect_; // отложенные для контакта адреса
#endif
    bool send_wait_reset_packet_ = false; //  отправлен пакет сброса ожидания
    bool in_processing_ = false; // флаг процесса обработки, для избежания двойного прерывания

    ulong time_first_packet_; // время отправки первого пакета (счётчик ограничения)
    ulong time_last_packet_;  // время отправки последнего пакета (счётчик ограничения)
    struct { // Структура ограничений времени ожиданий
        // Инициатор
        ulong I_connect;      // Ответа на запрос об установке соединения
        ulong I_wait_connect; // Отложенного соединения
        ulong I_amt_pack;     // Ответа с количеством принятых пакетов
        ulong I_num_pack;     // Ответа с номерами принятых пакетов
        // Получатель
        ulong R_connect;      // Запроса после сброса ожидания (отложенного соединения)
        ulong R_data_pack;    // Пакетов данных (после отправляется пакет с количеством пришедших пакетов)
        ulong R_correct_pack; // Результатов подтверждения
        ulong R_disconnect;   // Разрыва соединения
    } time_constraints_; // Структура ограничений времени ожиданий

    uint8_t send_packet_amt_; // количество пакетов (счётчик ограничения)
    struct { // Структура ограничений количества запросов
        // количество пакетов типа...
        // Инициатор
        uint8_t I_connect;  // Запрос на установку соединения
        uint8_t I_amt_pack; // Запрос о количестве принятых пакетов
        uint8_t I_num_pack; // Запрос номеров принятых пакетов
        // Получатель
        uint8_t R_wait_connect; // Сбросов ожидания (отложенного соединения)
        uint8_t R_correct_pack; // Ответов с количеством принятых пакетов
    } packet_constraints_; // Структура ограничений количества запросов

    /// --- функции создания пакетов ---
    // создание основы пакета
    void create_packet(uint8_t size, Packet_Type type_packet);
    // создание пакета установки соединения (num = count_pack) или ответа на
    //   запрос соединения (3 реакции +, +num и -wait) (и при замене инициативы)
    bool create_connect_packet(uint8_t amt_packet=0, bool swap_type=false);
    // создание пакета на основе номера из списка
    bool create_data_packet();
    // создание пакета количества пакетов
    bool create_amt_packet();
    // создание пакета номеров пришедших пакетов
    bool create_number_packet();
    // создание пакета сброса ожидания установки соединения
    bool create_reset_wait_packet();
    // создание пакета разрыва соединения
    bool create_disconnet_packet(bool error=false); // err переменная класса (?) -----

    /// --- функции обработки пакетов ---
    // Проверка пакета на "пришёл" и "мой|глобальный", и от того, кого
    //   я жду | от всех (при глобальном адресе ожидания - тоже все)
    bool packet_dont_correct(bool global_adr=false, bool all_adr_sendler=false);
    // Проверка соответствия типа пакета
#if defined ( USE_VECTOR )
    bool check_packet_type(Packet_Type type_packet, const std::vector<uint8_t>& subtype_packet={});
#else
    bool check_packet_type(Packet_Type type_packet);
    bool check_packet_type(Packet_Type type_packet, const uint8_t& subtype_packet);
#endif

    // поиск номера в _send_packet
    int16_t search_num_packet(uint16_t number);
    // удаление номеров в _send_packet и обновление _send_flag
#if defined ( USE_VECTOR )
    uint8_t pop_num_packet(const std::vector<uint16_t>& number);
#else
    uint8_t pop_num_packet(const std::array<uint16_t, CONTACT_DATA_MAX_PACKET>& number, uint8_t len);
#endif
    // сортировка _reciev_packet
    void sort_num_packet();

    /// --- функции контроля и обработки состояний при контакте ---

    // Инициализатор
    uint32_t init_connection(Stage_control& use_stage);
    uint32_t init_connection_expect(Stage_control& use_stage);
    uint32_t init_connection_wait(Stage_control& use_stage);

    uint32_t init_exchange(Stage_control& use_stage);
    uint32_t init_exchange_boardcast(Stage_control& use_stage);
    uint32_t init_exchange_wait_confirmation(Stage_control& use_stage);
    uint32_t init_exchange_wait_numbers(Stage_control& use_stage);

    uint32_t init_disconnect(Stage_control& use_stage);

    // Получатель
    uint32_t recip_connection(Stage_control& use_stage);
    uint32_t recip_connection_queue_check(Stage_control& use_stage);
    uint32_t recip_connection_wait_request(Stage_control& use_stage);

    uint32_t recip_exchange(Stage_control& use_stage);
    uint32_t recip_exchange_expect(Stage_control& use_stage);
    uint32_t recip_exchange_wait_reaction(Stage_control& use_stage);

    uint32_t recip_disconnect(Stage_control& use_stage);

    /// --- функции контроля и обработки состояний при трансляции ---
    uint32_t broadcast_wait_packet();
    uint32_t broadcast_send_packet();

    /// --- Функции общения с LoRa ---
    void contact_complete();
    void set_LoRa_mode_receive();
    void set_LoRa_mode_send(bool first=false);
    void set_LoRa_mode_sleep();
public:
    LoRa_contact_data();
    LoRa_contact_data(LoRa_address adr); // для загрузки
    ~LoRa_contact_data();
    // Функция инициализации объекта // (33 VSPI 32 27 26)
#if defined ( ESP32 )
    bool init_lora_module(uint8_t pin_reset, uint8_t spi_bus=VSPI, uint8_t spi_nss=0,
                          uint8_t pin_dio0=0, uint8_t pin_dio1=0, uint8_t pin_dio3=0);
#else
    bool init_lora_module(SPI_HandleTypeDef *spi);
#endif
    // Функция запуска работы системы и LoRa-модуля
    uint8_t begin_lora_module(ulong frequency, bool paboost=true, uint8_t signal_power=14,
                              uint8_t SF=8, ulong SBW=250E3, uint8_t sync_word=0x4A);

    // Установка адреса этого модуля
    bool set_my_adr(LoRa_address adr);
    // Получаение адреса этого модуля
    LoRa_address get_my_adr() const;
    // Получаение адреса устройства соединения
    LoRa_address get_connect_adr() const;

    // Установка канала связи
    bool set_channel(uint16_t channel);
    // Получаение канала связи
    uint16_t get_channel() const;

    // --- Создание пакетов для обмена информацией ---
    bool add_packet(uint8_t len, uint8_t* packet);
#if defined ( USE_VECTOR )
    bool add_packet(const std::vector<uint8_t>& packet);
    uint8_t add_packet(const std::vector<std::vector<uint8_t>>& packet);
#endif
    bool add_packet(LoRa_packet packet);
    bool clear_send_packet();

    // --- Соединение через установление контакта ---
    // Инициализатор
    bool init_contact(uint16_t adr_group, uint16_t adr_branch); // clear + creat init_packet
    bool init_contact(LoRa_address address); // clear + creat init_packet
    bool start_transfer();
    // Получатель
    bool wait_recipient(LoRa_address address=LoRa_address(LORA_GLOBAL_ADR_GROUP, LORA_GLOBAL_ADR_BRANCH));
    // Завершение (true - если контакт начался)
    bool end_contact();

    // --- Соединение через трансляции пакетов в сети ---
    uint16_t broadcast_send(bool reply=false);
    uint16_t broadcast_receive(LoRa_address address=LoRa_address(LORA_GLOBAL_ADR_GROUP, LORA_GLOBAL_ADR_BRANCH));
    uint16_t complete_broadcast_receive();

    // Получить количество отправляемых/принимаемых пакетов
    uint8_t get_amt_packet();
    // Получить текущую стадию контакта/трансляции
    int8_t get_state_contact();

    // Изъять пакет из списка принятых (удаляется)
    LoRa_packet get_packet();
    // Изъять все пришедшие пакеты
#if defined ( USE_VECTOR )
    std::vector<LoRa_packet> get_all_packet();
#else
    std::array<LoRa_packet, CONTACT_DATA_MAX_PACKET> get_all_packet(uint8_t& count);
#endif

    // Функция работы системы (запускается прерываниями на DIO,
    //   или в цикличной функции с предварительной проверкой DIO)
    uint32_t work_contact_system();

    // Выдаёт сигнал, если текущая стадия SC_COMPLETE
    bool get_signal_complete();
    bool get_signal_start_connect();
};


#endif // __LORA_CONTACT_DATA_H__
