#ifndef __GROW_DEVICE_INTERFACE_H__
#define __GROW_DEVICE_INTERFACE_H__

#include <Grow_device.h>
#include <Packet_analyzer.h>
#include <LoRa_packet.h>
#include <LoRa_contact_data.h>

#define GROUP_CONTROL_MODULE

class Grow_device_interface {
private:
	uint8_t system_package_handler(Grow_device &grow_device, LoRa_contact_data& contact_data, LoRa_packet& packet);
	uint8_t contact_package_handler(Grow_device &grow_device, LoRa_contact_data& contact_data, LoRa_packet& packet);
	uint8_t device_package_handler(Grow_device &grow_device, LoRa_contact_data& contact_data, LoRa_packet& packet);
public:
    Grow_device_interface() = default;
    ~Grow_device_interface() = default;

    /// --- Сохранение в энергонезависимую память ---
    // заполнение классов, загруженными значениями
    void load_data(Grow_device &grow_device, LoRa_contact_data& contact_data, uint32_t adr, uint32_t channel);
    // получение значений для сохранения классов
    bool save_data(const Grow_device &grow_device, const LoRa_contact_data& contact_data, uint32_t &adr, uint32_t &channel);

    /// --- LoRa-соединение ---
    // Регистрация (представиться) кодирование и декодирование
    void send_registration_packet(const Grow_device &grow_device, LoRa_contact_data& contact_data);
    bool check_contact_error(Grow_device &grow_device, LoRa_contact_data& contact_data);
    bool check_regist_packet(Grow_device &grow_device, LoRa_contact_data& contact_data);
    // Отправка и приём данных
    uint8_t build_data_packet(Grow_device &grow_device, LoRa_contact_data& contact_data);
    uint8_t add_data_packet(Grow_device &grow_device, LoRa_contact_data& contact_data, uint8_t num);
    uint8_t read_received_data_packets(Grow_device &grow_device, LoRa_contact_data& contact_data);

};

extern Grow_device_interface grow_device_interface;

#endif // __GROW_DEVICE_INTERFACE_H__
