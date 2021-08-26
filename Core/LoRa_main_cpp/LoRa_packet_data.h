#ifndef __LORA_PACKET_DATA_H__
#define __LORA_PACKET_DATA_H__

#include <stdbool.h>

struct lora_packet {
    uint8_t* _data; // = nullptr;
    uint8_t _len; // = 0;
    uint8_t _rssi; // = 0;
    uint8_t _num; // = 0;
    float _snr; // = 0;
    bool _crc_error; // = false;
};

#endif // __LORA_PACKET_DATA_H__
