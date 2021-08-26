#include "../Grow_device_component/Grow_device_component.h"

Grow_device_component::Grow_device_component(Type_device type_device, uint8_t id) {
    type_device_ = type_device;
    id_ = id;
    value_ = 0;
}

enum Type_device Grow_device_component::get_type() const {
    return type_device_;
}
uint8_t Grow_device_component::get_id() const {
    return id_;
}
void Grow_device_component::set_id(uint8_t id) {
    id_ = id;
}
uint16_t Grow_device_component::get_value() const {
    return value_;
}
bool Grow_device_component::set_value(uint16_t value) {
    if(value_ == value)
        return false;
    value_ = value;
    return true;
}

bool Grow_device_component::filter(Grow_device_component &component) {
    return (type_device_ == component.type_device_);
}


size_t Grow_device_component::get_size() {
    // enum Type_device _type_device; << 255
    return (1);
}
size_t Grow_device_component::get_data(uint8_t *data) {
    if(data == nullptr)
        return 0;
    size_t size = 0;
    data[size++] = (uint8_t)(type_device_ & 0xFF);
    return size;
}
size_t Grow_device_component::set_data(uint8_t *data, size_t available_size) {
    if((data == nullptr) || (available_size < get_size()))
        return 0;
    size_t size = 0;
    type_device_ = (enum Type_device)(data[size++]);
    return size;
}
