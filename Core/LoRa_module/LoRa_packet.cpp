#include "LoRa_packet.h"


// ----- LoRa_packet_data -----


#if defined( USE_STANDARD_ARRAY )
std::array<LoRa_packet_data, SIZE_LORA_PACKET_BUFFER> lora_packet_data;
#else
LoRa_packet_data lora_packet_data[SIZE_LORA_PACKET_BUFFER];
#endif


bool LoRa_packet_data::add_data(uint8_t data_byte) {
    if(len >= SIZE_LORA_PACKET_MAX_LEN)
        return true;
    data[len++] = data_byte;
    return false;
}
bool LoRa_packet_data::add_data(const uint8_t* data_byte, uint8_t amt_byte) {
    if((len + amt_byte) > SIZE_LORA_PACKET_MAX_LEN)
        return true;
    for(size_t i = 0; i < amt_byte; ++i)
        data[i+len] = data_byte[i];
    len += amt_byte;
    return false;
}

bool LoRa_packet_data::set_data(const uint8_t* set_data, uint8_t set_len) {
    if(set_len > SIZE_LORA_PACKET_MAX_LEN)
        return true;
    if((set_data != nullptr) && (set_len != 0)){
        len = set_len;
        for(int i = 0; i < set_len; ++i)
            data[i] = set_data[i];
    }
    else
        len = 0;
    return false;
}
bool LoRa_packet_data::set_data(const std::vector<uint8_t>& set_data){
    if(set_data.size() > SIZE_LORA_PACKET_MAX_LEN)
        return true;
    if(set_data.empty())
        len = 0;
    else {
        len = data.size();
        for(size_t i = 0; i < data.size(); ++i)
            data[i] = set_data[i];
    }
    return false;
}
void LoRa_packet_data::set_data(const class LoRa_packet& lora_packet) {
    *this = lora_packet;
}
void LoRa_packet_data::set_data(const class LoRa_packet_data& lora_packet) {
    *this = lora_packet;
}
void LoRa_packet_data::set_data(class LoRa_packet_data&& lora_packet) {
    *this = std::move(lora_packet);
}
bool LoRa_packet_data::free() const {
    return free_object_;
}
class LoRa_packet_data& LoRa_packet_data::operator=(const class LoRa_packet& right) {
    if((!(right.packet_data->free_object_)) && (right.packet_data->len != 0)){
        len = right.packet_data->len;
        for(int i = 0; i < right.packet_data->len; ++i)
            data[i] = right.packet_data->data[i];
    }
    else
        len = 0;
    return *this;
}
class LoRa_packet_data& LoRa_packet_data::operator=(const class LoRa_packet_data& right) {
    if((!(right.free_object_)) && (right.len != 0)){
        len = right.len;
        for(int i = 0; i < right.len; ++i)
            data[i] = right.data[i];
    }
    else
        len = 0;
    return *this;
}


// ----- LoRa_packet -----
bool LoRa_packet::search_data() {
#if defined( USE_STANDARD_ARRAY )
    packet_data = std::find_if(lora_packet_data.begin(), lora_packet_data.end(),
                        [](const LoRa_packet_data &data){return data.free();} );
    if(packet_data == lora_packet_data.end()) {
        packet_data = nullptr;
#if defined( ESP32 )
        Serial.println("!lora_packet_data memory error!");
#endif
        return true;
    }
    packet_data->free_object_ = false;
    packet_data->len = 0;
    return false;
#else
    for(int i = 0; i < SIZE_LORA_PACKET_BUFFER; ++i) {
        if(lora_packet_data[i].free_object_) {
            lora_packet_data[i].free_object_ = false;
            lora_packet_data[i].len = 0;
            packet_data = &lora_packet_data[i];
            return false;
        }
    }
    packet_data = nullptr;
    return true;
#endif
}


LoRa_packet::LoRa_packet() {
    search_data();
}
LoRa_packet::LoRa_packet(const uint8_t* data, uint8_t len, bool crc_error, uint8_t rssi) {
    search_data();
    set_packet(data, len, crc_error, rssi);
}
LoRa_packet::LoRa_packet(const std::vector<uint8_t>& data, bool crc_error, uint8_t rssi) {
    search_data();
    set_packet(data, crc_error, rssi);
}
LoRa_packet::LoRa_packet(const LoRa_packet& right) {
    search_data();
    *this = right;
}
LoRa_packet::LoRa_packet(LoRa_packet&& right) {
    packet_data = nullptr;
    *this = std::move(right);
}

LoRa_packet::~LoRa_packet() {
    if(packet_data != nullptr)
        packet_data->free_object_ = true;
}

bool LoRa_packet::add_packet_data(uint8_t data) {
    return packet_data->add_data(data);
}
bool LoRa_packet::add_packet_data(const  uint8_t* data, uint8_t len) {
    return packet_data->add_data(data, len);
}
bool LoRa_packet::set_packet(const uint8_t* data, uint8_t len, bool crc_error, uint8_t rssi) {
    if(packet_data->set_data(data, len))
        return true;
    crc_error_ = crc_error;
    rssi_ = rssi;
    return false;
}
bool LoRa_packet::set_packet(const std::vector<uint8_t>& data, bool crc_error, uint8_t rssi) {
    if(packet_data->set_data(data))
        return true;
    crc_error_ = crc_error;
    rssi_ = rssi;
    return false;
}

void LoRa_packet::clear_packet() {
    if(packet_data == nullptr)
        search_data();
    else
        packet_data->len = 0;
    rssi_ = 0;
    crc_error_  = false;
}

std::vector<uint8_t> LoRa_packet::get_data() const {
    std::vector<uint8_t> data;
    for(int i = 0; i < packet_data->len; ++i)
        data.push_back(packet_data->data[i]);
    return data;
}
uint8_t LoRa_packet::get_data(int num) const {
    return packet_data->data[num];
}
uint8_t LoRa_packet::get_len() const {
    if(packet_data == nullptr)
        return 0;
    return packet_data->len;
}
bool LoRa_packet::get_crc_error() const {
    return crc_error_;
}
uint8_t LoRa_packet::get_rssi() const {
    return rssi_;
}
uint8_t& LoRa_packet::operator[] (const int index) {
    return packet_data->data[index];
}
const uint8_t& LoRa_packet::operator[](const int index) const {
    return packet_data->data[index];
}
class LoRa_packet& LoRa_packet::operator=(const class LoRa_packet& right) {
    // Проверка на самоприсваивание
    if (this == &right)
        return *this;
    // Проверка на пустой объект
    if(packet_data == nullptr)
        search_data();
    // Перенос значений
    packet_data->len = right.packet_data->len;
    crc_error_ = right.crc_error_;
    rssi_ = right.rssi_;
    for(int i = 0; i < packet_data->len; ++i)
        packet_data->data[i] = right.packet_data->data[i];
    return *this;
}

class LoRa_packet& LoRa_packet::operator=(class LoRa_packet&& right) {
    // Проверка на самоприсваивание
    if (this == &right)
        return *this;
    // Перенос значений
    if(packet_data != nullptr) {
        packet_data->free_object_=true;
    }
    packet_data = right.packet_data;
    crc_error_ = right.crc_error_;
    rssi_ = right.rssi_;
    right.packet_data = nullptr;
    return *this;
}
