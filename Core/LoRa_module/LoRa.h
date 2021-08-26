#ifndef __LORA_H__
#define __LORA_H__

#include <main.h>
#include <vector>
#include <LoRa_register.h>
#include <LoRa_packet.h>


class LoRa_class {
private:
	SPI_HandleTypeDef* _spi;
    uint16_t _reset_pin;
    GPIO_TypeDef* _reset_port;
    uint16_t _nss_pin;
    GPIO_TypeDef* _nss_port;
    uint16_t _dio0_pin;
    GPIO_TypeDef* _dio0_port;
    uint16_t _dio1_pin;
    GPIO_TypeDef* _dio1_port;
    uint16_t _dio3_pin;
    GPIO_TypeDef* _dio3_port;
    uint64_t _frequency;

    uint8_t _FifoTxBaseAddr;
    uint8_t _packet_length;
    class LoRa_register _reg_field;
//    class LoRa_packet packet;

    friend class LoRa_register;
public:
    LoRa_class();
    LoRa_class(SPI_HandleTypeDef *spi);
    ~LoRa_class();

    void init(SPI_HandleTypeDef *spi);

    uint8_t field_set(Address_field field, uint32_t value, bool write=true);
    uint8_t field_set(Address_field* fields, uint32_t* values, uint8_t amt, bool write=true);

    uint8_t field_get(Address_field field, uint32_t* value, bool read=false);
    uint8_t field_get(Address_field* fields, uint32_t* values, uint8_t amt, bool read=false);

    uint8_t begin(uint64_t frequency, bool paboost=false, uint8_t signal_power=14,
                  uint8_t SF=11, uint64_t SBW=125E3, uint8_t sync_word=0x4A);
    void end();
    uint8_t set_mode(uint8_t mode);
    uint8_t mode_sleep();
    uint8_t mode_STDBY();
    uint8_t mode_TX(bool set_dio=true);
    uint8_t mode_FSTX(); // Подготовки к отправке
    uint8_t mode_FSRX();
    uint8_t mode_RX_continuous(bool set_dio=true);
    uint8_t mode_RX_single(bool set_dio=true);
    uint8_t mode_CAD(bool set_dio=true);
    uint8_t set_TX_power(uint8_t power, bool paboost, uint8_t max_power=0x07);
    uint8_t set_frequency(uint64_t frequency);
    uint8_t set_spreading_factor(uint8_t SF);
    uint8_t set_signal_bandwidth(uint64_t sbw);
    uint8_t set_preamble_length(unsigned int length);
    uint8_t set_sync_word(uint8_t SW);
    uint8_t crc_enable();
    uint8_t crc_disable();
    class LoRa_packet receiver_packet(uint8_t count=1, uint32_t wait=10000, bool rssi=false, bool snr=false);
    class LoRa_packet read_packet_data(bool crc_err, bool rssi=false, bool snr=false);
    uint8_t packet_rssi();
    float packet_snr();
    bool sender_packet(uint8_t* packet, uint8_t len, uint32_t wait=100);
    bool sender_packet(std::vector<uint8_t> packet, uint32_t wait=100);
    bool packet_begin();
    bool packet_write(uint8_t* packet, uint8_t len);
    bool packet_write(std::vector<uint8_t> packet);
    bool packet_end(uint32_t wait=2000, bool sleep=false);
};
extern class LoRa_class LoRa;

#endif // __LORA_H__
