#include "Address_field.h"

uint32_t value_range(uint32_t value, uint32_t min_value, uint32_t max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}
uint16_t value_range(uint16_t value, uint16_t min_value, uint16_t max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}
uint8_t  value_range(uint8_t  value, uint8_t  min_value, uint8_t  max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

//   ----- ----- ----- ---- ----- ----- -----
// ----- ----- ----- Register ----- ----- -----
//   ----- ----- ----- ---- ----- ----- -----

Register::Register(uint16_t address, uint8_t bit_count, uint8_t bit_bias): address_(address), bit_count_(bit_count), bit_bias_(bit_bias) {
    uint8_t bit_in_reg = 8;
    bit_count = value_range(bit_count, 1, bit_in_reg);
    bit_bias = value_range(bit_bias, 0, (bit_in_reg - bit_count));
    uint32_t mask = 0x00;
    for (uint8_t i = 0; i < bit_in_reg; ++i) {
        if (i < bit_count)
            mask = (mask << 1) + 1;
        else
            if (i < (bit_count + bit_bias))
                mask <<= 1;
    }
    mask_ = mask;
}

uint16_t Register::address() const {
    return address_;
}
uint8_t Register::bit_count() const {
    return bit_count_;
}
uint8_t Register::bit_bias() const {
    return bit_bias_;
}
uint32_t Register::mask() const {
    return mask_;
}

bool operator==(const Register& left, const Register& right) {
    if(left.address_ != right.address_)
        return false;
    if(left.bit_count_ != right.bit_count_)
        return false;
    if(left.bit_bias_ != right.bit_bias_)
        return false;
    if(left.mask_ != right.mask_)
        return false;
    return true;
}

bool operator!=(const Register& left, const Register& right) {
    return !(left == right);
}

//   ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- -----  Address_field  ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- -----

Address_field::Address_field(const Register* registers, const uint8_t reg_count, const bool reg_revers, const char mode, const uint32_t min_value,
                             const uint32_t max_value, const uint32_t *reserved_value, const uint32_t reserv_count) {
    init_address_field(registers, reg_count, reg_revers, mode, min_value, max_value, reserved_value, reserv_count);
}

bool Address_field::init_address_field(const Register *registers, const uint8_t reg_count, const bool reg_revers, const char mode, const uint32_t min_value,
                                       const uint32_t max_value, const uint32_t *reserved_value, const uint32_t reserv_count) {
    if ((mode != 'r') && (mode != 'w') && (mode != 'c'))
        return true;
    uint8_t bit_count = 0;
    uint32_t value = 0;
    mode_ = mode;
    registers_ = registers;
    reg_count_ = reg_count;
    reg_revers_ = reg_revers;
    reserved_value_ = reserved_value;
    reserv_count_ = reserv_count;
    max_address_ = 0;
    for (int i = 0; i < reg_count_; ++i) {
        bit_count += registers_[i].bit_count();
        if (max_address_ < registers_[i].address())
            max_address_ = registers_[i].address();
    }
    for (int i = 0; i < bit_count; ++i)
        value = (value << 1) + 1;
    min_value_ = value_range(min_value, 0, value);
    max_value_ = value_range(max_value, min_value, value);
    return false;
}


uint32_t Address_field::get_value(const uint8_t *register_value, int register_count) const {
    if (register_count < (max_address_ + 1))
        return -1;
    uint32_t value = 0;
    if(reg_revers_) {
        for (int i = 0; i < reg_count_; ++i) {
            value <<= registers_[i].bit_count();
            value |= (register_value[registers_[i].address()] & registers_[i].mask()) >> registers_[i].bit_bias();
        }
    }
    else {
        for (int i = (reg_count_ - 1); i >= 0; --i) {
            value <<= registers_[i].bit_count();
            value |= (register_value[registers_[i].address()] & registers_[i].mask()) >> registers_[i].bit_bias();
        }
    }
    return value;
}

bool Address_field::set_value(uint32_t value, uint8_t *register_value, int register_count) const {
    if ((register_count < (max_address_ + 1)) || (mode_ == 'r') ||
                (value < min_value_) || (value > max_value_))
        return true;
    for (uint32_t i = 0; i < reserv_count_; ++i)
        if (value == reserved_value_[i])
            return true;
    if(reg_revers_) {
        for (int i = (reg_count_ - 1); i >= 0; --i) {
            register_value[registers_[i].address()] &= ~registers_[i].mask();
            register_value[registers_[i].address()] |= (value & (registers_[i].mask() >> registers_[i].bit_bias())) << registers_[i].bit_bias();
            value >>= registers_[i].bit_count();
        }
    }
    else {
        for (int i = 0; i < reg_count_; ++i) {
            register_value[registers_[i].address()] &= ~registers_[i].mask();
            register_value[registers_[i].address()] |= (value & (registers_[i].mask() >> registers_[i].bit_bias())) << registers_[i].bit_bias();
            value >>= registers_[i].bit_count();
        }
    }
    return false;
}

#if defined( ADD_LORA_PACKET_CODE )
uint32_t Address_field::get_value(const class LoRa_packet& packet, uint8_t bias) const {
    if ((SIZE_LORA_PACKET_MAX_LEN - bias) < ((uint32_t)max_address_ + 1))
        return -1;
    uint32_t value = 0;
    if(reg_revers_) {
        for (int i = 0; i < reg_count_; ++i) {
            value <<= registers_[i].bit_count();
            value |= (packet[registers_[i].address() + bias] & registers_[i].mask()) >> registers_[i].bit_bias();
        }
    }
    else {
        for (int i = (reg_count_ - 1); i >= 0; --i) {
            value <<= registers_[i].bit_count();
            value |= (packet[registers_[i].address() + bias] & registers_[i].mask()) >> registers_[i].bit_bias();
        }
    }
    return value;
}
bool Address_field::set_value(uint32_t value, class LoRa_packet& packet, uint8_t bias) const {
    if (((SIZE_LORA_PACKET_MAX_LEN - bias) < ((uint32_t)max_address_ + 1)) || (mode_ == 'r') ||
                (value < min_value_) || (value > max_value_))
        return true;
    for (uint32_t i = 0; i < reserv_count_; ++i)
        if (value == reserved_value_[i])
            return true;

    // while(packet.get_len() < max_address_ + bias)
    //     packet.add_packet_data(0);
    packet.add_packet_data(&packet[packet.get_len()-1], max_address_ + bias - packet.get_len() + 1);

    if(reg_revers_) {
        for (int i = (reg_count_ - 1); i >= 0; --i) {
            packet[registers_[i].address() + bias] &= ~registers_[i].mask();
            packet[registers_[i].address() + bias] |= (value & (registers_[i].mask() >> registers_[i].bit_bias())) << registers_[i].bit_bias();
            value >>= registers_[i].bit_count();
        }
    }
    else {
        for (int i = 0; i < reg_count_; ++i) {
            packet[registers_[i].address() + bias] &= ~registers_[i].mask();
            packet[registers_[i].address() + bias] |= (value & (registers_[i].mask() >> registers_[i].bit_bias())) << registers_[i].bit_bias();
            value >>= registers_[i].bit_count();
        }
    }
    return false;
}
#endif

const Register *Address_field::get_registers() const {
    return registers_;
}
uint8_t Address_field::get_reg_count() const {
    return reg_count_;
}
uint16_t Address_field::get_max_address() const {
    return max_address_;
}
uint32_t Address_field::get_min_value() const {
    return min_value_;
}
uint32_t Address_field::get_max_value() const {
    return max_value_;
}
const uint32_t *Address_field::get_reserved_value() const {
    return reserved_value_;
}
uint32_t Address_field::get_reserv_count() const {
    return reserv_count_;
}
char Address_field::get_mode() const {
    return mode_;
}

bool operator==(const Address_field& left, const Address_field& right) {
    if(left.reg_count_ != right.reg_count_)
        return false;
    if(left.max_address_ != right.max_address_)
        return false;
    if(left.min_value_ != right.min_value_)
        return false;
    if(left.max_value_ != right.max_value_)
        return false;
    if(left.reserv_count_ != right.reserv_count_)
        return false;
    if(left.mode_ != right.mode_)
        return false;
    for(int i = 0; i < left.reg_count_; ++i) {
        if(left.registers_[i] != right.registers_[i])
            return false;
    }
    for(uint32_t i = 0; i < left.reserv_count_; ++i) {
        if(left.reserved_value_[i] != right.reserved_value_[i])
            return false;
    }
    return true;
}

bool operator!=(const Address_field& left, const Address_field& right) {
    if(left.reg_count_ != right.reg_count_)
        return true;
    if(left.max_address_ != right.max_address_)
        return true;
    if(left.min_value_ != right.min_value_)
        return true;
    if(left.max_value_ != right.max_value_)
        return true;
    if(left.reserv_count_ != right.reserv_count_)
        return true;
    if(left.mode_ != right.mode_)
        return true;
    for(int i = 0; i < left.reg_count_; ++i) {
        if(left.registers_[i] != right.registers_[i])
            return true;
    }
    for(uint32_t i = 0; i < left.reserv_count_; ++i) {
        if(left.reserved_value_[i] != right.reserved_value_[i])
            return true;
    }
    return false;
}

#if defined (SERIAL_LOG_OUTPUT)

void Address_field::print() {
    Serial.print("reg_count = ");
    Serial.print(reg_count_);
    if ((reg_count_) != 0) {
        Serial.println(" :");
        for (int i = 0; i < reg_count_; ++i){
            Serial.print("-registers: ");
            Serial.println(i);

            Serial.print("  address: ");
            Serial.println(registers_[i].address());

            Serial.print("  bit_count: ");
            Serial.println(registers_[i].bit_count());

            Serial.print("  bit_bias: ");
            Serial.println(registers_[i].bit_bias());

            Serial.print("  mask: ");
            Serial.println(registers_[i].mask(), 2);
        }
    }
    else
        Serial.println();

    Serial.print("max_address = ");
    Serial.println(max_address_, 16);

    Serial.print("min_value = ");
    Serial.println(min_value_, 16);

    Serial.print("max_value = ");
    Serial.println(max_value_, 16);

    Serial.print("reserv_count = ");
    Serial.print(reserv_count_);
    if ((reserv_count_) != 0) {
        Serial.println(" :");
        for (int i = 0; i < reserv_count_; ++i){
            Serial.print("  ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print("reserved_value = ");
            Serial.println(reserved_value_[i]);
        }
    }
    else
        Serial.println();

    Serial.print("mode = ");
    Serial.println(mode_);
}

#endif
