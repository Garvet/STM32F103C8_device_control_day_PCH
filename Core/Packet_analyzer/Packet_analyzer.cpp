#include "Packet_analyzer.h"
#include "Packet_field.h"

Packet_analyzer packet_analyzer;
Packet_Connection packet_connection;
Packet_Sensor packet_sensor;
Packet_Device packet_device;
Packet_System packet_system;

LoRa_address::LoRa_address(const uint8_t* bytes, const uint8_t len) {
	group = field_dest_adr_group.get_value(bytes, len);
	branch = field_dest_adr_branch.get_value(bytes, len);
}

LoRa_address::LoRa_address(const uint16_t group, const uint16_t branch):group(group),branch(branch){}

LoRa_address::LoRa_address(const uint32_t adr):group((uint16_t)(adr >> 16)),branch((uint16_t)(adr)){}

bool LoRa_address::global() {
    return ((group == LORA_GLOBAL_ADR_GROUP) && (branch == LORA_GLOBAL_ADR_BRANCH));
}
bool operator==(const LoRa_address& left, const LoRa_address& right) {
    return ((left.group == right.group) && (left.branch == right.branch));
}
bool operator!=(const LoRa_address& left, const LoRa_address& right) {
    return ((left.group != right.group) || (left.branch != right.branch));
}
bool operator==(const LoRa_address& left, const uint32_t& right) {
    return ((left.group == ((right >> 16) & 0xFFFF)) && (left.branch == (right & 0xFFFF)));
}
bool operator!=(const LoRa_address& left, const uint32_t& right) {
    return ((left.group != ((right >> 16) & 0xFFFF)) || (left.branch != (right & 0xFFFF)));
}
bool operator==(const uint32_t& left, const LoRa_address& right) {
    return ((((left >> 16) & 0xFFFF) == right.group) && ((left & 0xFFFF) == right.branch));
}
bool operator!=(const uint32_t& left, const LoRa_address& right) {
    return ((((left >> 16) & 0xFFFF) != right.group) || ((left & 0xFFFF) != right.branch));
}

//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- ----- ----- ----- Packet_analyzer ----- ----- ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

Packet_analyzer::Packet_analyzer() {
    field_packet_ = nullptr;
    amt_field_packet_ = 0;
    field_header_ = field_packet_header;
    amt_field_header_ = count_field_packet_header;
}

// Адрес группы адресанта
uint16_t Packet_analyzer::get_dest_adr_group(const LoRa_packet& packet) {
    return field_header_[num_dest_adr_group]->get_value(packet);
    // return field_header_[num_dest_adr_group]->get_value(&(packet[0]), packet.get_len());
}
// Адрес ветви  адресанта
uint16_t Packet_analyzer::get_dest_adr_branch(const LoRa_packet& packet) {
    return field_header_[num_dest_adr_branch]->get_value(packet);
    // return field_header_[num_dest_adr_branch]->get_value(&(packet[0]), packet.get_len());
}
// Адрес адресанта
LoRa_address Packet_analyzer::get_dest_adr(const LoRa_packet& packet) {
    return {get_dest_adr_group(packet), get_dest_adr_branch(packet)};
}
// Адрес группы отправителя
uint16_t Packet_analyzer::get_sour_adr_group(const LoRa_packet& packet) {
    return field_header_[num_sour_adr_group]->get_value(packet);
    // return field_header_[num_sour_adr_group]->get_value(&(packet[0]), packet.get_len());
}
// Адрес ветви  отправителя
uint16_t Packet_analyzer::get_sour_adr_branch(const LoRa_packet& packet) {
    return field_header_[num_sour_adr_branch]->get_value(packet);
    // return field_header_[num_sour_adr_branch]->get_value(&(packet[0]), packet.get_len());
}
// Адрес отправителя
LoRa_address Packet_analyzer::get_sour_adr(const LoRa_packet& packet) {
    return {get_sour_adr_group(packet), get_sour_adr_branch(packet)};
}
// Тип пакета
Packet_Type Packet_analyzer::get_packet_type(const LoRa_packet& packet) {
    return (Packet_Type)field_header_[num_type]->get_value(packet);
    // return field_header_[num_type]->get_value(&(packet[0]), packet.get_len());
}
// Номер пакета
uint16_t Packet_analyzer::get_packet_number(const LoRa_packet& packet) {
    return field_header_[num_number]->get_value(packet);
    // return field_header_[num_number]->get_value(&(packet[0]), packet.get_len());
}

// Адрес группы адресанта
bool Packet_analyzer::set_dest_adr_group(LoRa_packet& packet, uint16_t adr) {
    return field_header_[num_dest_adr_group]->set_value(adr, packet);
    // return field_header_[num_dest_adr_group]->set_value(adr, &(packet[0]));
}
// Адрес ветви  адресанта
bool Packet_analyzer::set_dest_adr_branch(LoRa_packet& packet, uint16_t adr) {
    return field_header_[num_dest_adr_branch]->set_value(adr, packet);
    // return field_header_[num_dest_adr_branch]->set_value(adr, &(packet[0]));
}
// Адрес адресанта
bool Packet_analyzer::set_dest_adr(LoRa_packet& packet, LoRa_address adr) {
    uint16_t buffer = get_dest_adr_group(packet);
    if(set_dest_adr_group(packet, adr.group))
        return true;
    if(set_dest_adr_branch(packet, adr.branch)) {
        set_dest_adr_group(packet, buffer);
        return true;
    }
    return false;
}
// Адрес группы отправителя
bool Packet_analyzer::set_sour_adr_group(LoRa_packet& packet, uint16_t adr) {
    return field_header_[num_sour_adr_group]->set_value(adr, packet);
    // return field_header_[num_sour_adr_group]->set_value(adr, &(packet[0]));
}
// Адрес ветви  отправителя
bool Packet_analyzer::set_sour_adr_branch(LoRa_packet& packet, uint16_t adr) {
    return field_header_[num_sour_adr_branch]->set_value(adr, packet);
    // return field_header_[num_sour_adr_branch]->set_value(adr, &(packet[0]));
}
// Адрес отправителя
bool Packet_analyzer::set_sour_adr(LoRa_packet& packet, LoRa_address adr) {
    uint16_t buffer = get_sour_adr_group(packet);
    if(set_sour_adr_group(packet, adr.group))
        return true;
    if(set_sour_adr_branch(packet, adr.branch)) {
        set_sour_adr_group(packet, buffer);
        return true;
    }
    return false;
}
// Тип пакета
bool Packet_analyzer::set_packet_type(LoRa_packet& packet, Packet_Type pac_type) {
    return  field_header_[num_type]->set_value((uint32_t)pac_type, packet);
}
bool Packet_analyzer::set_packet_type(LoRa_packet& packet, uint8_t pac_type) {
    return field_header_[num_type]->set_value(pac_type, packet);
    // return field_header_[num_type]->set_value(pac_type, &(packet[0]));
}
// Номер пакета
bool Packet_analyzer::set_packet_number(LoRa_packet& packet, uint16_t num) {
    return field_header_[num_number]->set_value(num, packet);
    // return field_header_[num_number]->set_value(num, &(packet[0]));
}

bool Packet_analyzer::set_setting(uint8_t setting) {
    return false; // настройки индивидуальны, у общего класса их нет
}
uint8_t Packet_analyzer::get_setting() {
    return setting_;
}



//   ----- ----- ----- ----- -----  ----- -----  ----- ----- ----- ----- -----
// ----- ----- ----- ----- ----- Packet_Connection ----- ----- ----- ----- -----
//   ----- ----- ----- ----- -----  ----- -----  ----- ----- ----- ----- -----

bool Packet_Connection::set_setting(uint8_t setting) {
    return false; // не зависит от настроек
}

/// --- Запись в пакет ---

// Установить команду
uint8_t Packet_Connection::set_command(LoRa_packet& packet, uint8_t com) {
    // Проверка корректнгости команды
    if (!(com < CONNECT_COMMAND_DATA))
        return 1;
    command_ = com;
    // Заполение поля
    bool error = false;
    last_filled_byte = PACKET_HEADER_SIZE; // Команда всегда находится сразу после шапки
    error = field_byte.set_value(command_, packet, last_filled_byte);
    // error = field_byte.set_value(command_, &(packet[last_filled_byte]), (SIZE_LORA_PACKET_MAX_LEN - last_filled_byte));
    if (error)
        return 2;
    ++last_filled_byte;
    return 0;
}
// Установить данные
uint8_t Packet_Connection::set_data(LoRa_packet& packet, uint8_t *data, uint8_t len) {
    uint8_t len_data;
    uint8_t error = 0;

    error = get_size_by_data(&command_, &len, len_data);
    if(error)
        return error;


    // Получение длины в соответствии с командой
    if (connect_com_data[command_] == 0xFF) {
        // Расчётное соответствие команды длинне
        switch (command_) {
        case 0x01:
        case 0x09:
        case 0x0A: {
            break;
        }
        case 0x06:
        case 0x0C: {
            error = field_byte.set_value(len, packet, last_filled_byte);
            // error = field_byte.set_value((len), &(packet[last_filled_byte]), (SIZE_LORA_PACKET_MAX_LEN - last_filled_byte));
            ++last_filled_byte;
            if (error)
                return 4;
            break;
        }
        default:
            return 6;
        }
    }
    // Заполение данных
    for (int i = 0; i < len_data; ++i) {
        error = field_byte.set_value(data[i], packet, last_filled_byte);
        // error = field_byte.set_value(data[i], &(packet[last_filled_byte]), (SIZE_LORA_PACKET_MAX_LEN - last_filled_byte));
        ++last_filled_byte;
        if (error)
            return 8;
    }
    // (!) ----- packet.len = last_filled_byte
    return 0;
}
// Занести в пакет параметры и данные
uint8_t Packet_Connection::set_packet_data(LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len) {
    uint8_t error;
    error = set_command(packet, *com);
    if(error)
        return error;
    error = set_data(packet, data, *len);
    return error;
}

/// --- Чтение из пакета ---

// Получить команду
uint8_t Packet_Connection::get_command(const LoRa_packet& packet, uint8_t *com) {
    last_read_byte = PACKET_HEADER_SIZE; // Команда всегда находится сразу после шапки
    // Проверка некорректного аргумента
    if(com == nullptr)
        return 1;
    // Чтение комманды
    command_ = field_byte.get_value(packet, last_read_byte);
    // command_ = field_byte.get_value(&(packet[last_read_byte]), (packet.get_len() - last_read_byte));
    ++last_read_byte;
    // Проверка команды
    if (!(command_ < CONNECT_COMMAND_DATA)) {
        command_ = 0xFF;
        return 2;
    }
    *com = command_;
    return 0;
}
// Получить данные
uint8_t Packet_Connection::get_data(const LoRa_packet& packet, uint8_t *data, uint8_t *len) {
    uint8_t len_data;
    uint8_t error = 0;
    // Проверка некорректного аргумента
    if(len == nullptr)
        return 1;
    // Получение длины в соответствии с командой
    error = get_size_by_data(&command_, nullptr, len_data);
    if(error == 2) {
        if(command_ == 0x01) {
            error = get_size_by_data(&command_, len, len_data);
        }
        else {
            *len = field_byte.get_value(packet, last_read_byte);
            ++last_read_byte;
            // *len = field_byte.get_value(&(packet[last_read_byte]), (packet.get_len() - last_read_byte));
            error = get_size_by_data(&command_, len, len_data);
            if (command_ == 0x06) {
                *len = len_data/2;
            }
            else {
                *len = len_data;
            }
        }
    }
    else if(error == 3) {
        if (((packet.get_len() - last_read_byte) == 1) || ((packet.get_len() - last_read_byte) == 2)) {
            len_data = packet.get_len() - last_read_byte;
            *len = len_data;
            error = 0;
        }
    }
    else
        *len = len_data;
    if(error)
        return error;

    if(len_data > 0) {
        // Проверка некорректного аргумента
        if(data == nullptr)
            return 6;
        // Получение данных
        for (int i = 0; i < len_data; ++i) {
            data[i] = field_byte.get_value(packet, last_read_byte);
            // data[i] = field_byte.get_value(&(packet[last_read_byte]), (packet.get_len() - last_read_byte));
            ++last_read_byte;
        }
    }
    return 0;
}
// Получить из пакета параметры и данные
uint8_t Packet_Connection::get_packet_data(const LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len) {
    uint8_t error;
    error = get_command(packet, com);
    if(error)
        return error;
    error = get_data(packet, data, len);
    return error;
}

/// --- Расчёты ---

// Узнать объём поля данных по параметрам
uint8_t Packet_Connection::get_size_by_data(const uint8_t *com, const uint8_t *len, uint8_t &size_data) {
    size_data = 0;
    // Комманда, проверка на выход за границы
    if (!(*com < CONNECT_COMMAND_DATA)) {
        return 1;
    }
    // Получение объёма
    if (connect_com_data[*com] != 0xFF) {
        size_data = connect_com_data[*com];
    }
    else {
        // Проверка некорректного аргумента
        if(len == nullptr)
            return 2;
        switch (*com)
        {
        case 0x01:
            if ((*len != 1) && (*len != 2))
                return 3;
            size_data = *len;
            break;
        case 0x06:
            size_data = *len;
            break;
        case 0x09:
            size_data = 0; // (!) ----- ----- ----- ----- -----
            break;
        case 0x0A:
            size_data = 0; // (!) ----- ----- ----- ----- -----
            break;
        case 0x0C:
            size_data = *len;
            break;
        default:
            return 4;
        }
    }
    return 0;
}
// Узнать объём поля данных по содержимому пакета
uint8_t Packet_Connection::get_size_by_packet(const LoRa_packet& packet, uint8_t &size_data) {
    uint8_t error;
    error = get_command(packet, &command_);
    if(error)
        return error;
    error = get_size_by_data(&command_, nullptr, size_data);
    if(error == 2) {
        uint8_t len = field_byte.get_value(packet, last_read_byte);
        // uint8_t len = field_byte.get_value(&(packet[last_read_byte]), (packet.get_len() - last_read_byte));
        error = get_size_by_data(&command_, &len, size_data);
        last_read_byte = PACKET_HEADER_SIZE + 1;
    }
    if(error == 3) {
        if (((packet.get_len() - last_read_byte) == 1) || ((packet.get_len() - last_read_byte) == 2)) {
            size_data = packet.get_len() - last_read_byte;
            error = 0;
        }
    }
    return error;
}



//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- ----- ----- -----  Packet_Sensor  ----- ----- ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

bool Packet_Sensor::set_setting(uint8_t setting) {
    // if по типам
    setting_ = setting;
    return false;
}


uint8_t Packet_Sensor::set_packet_data(LoRa_packet& packet, uint8_t* amt, uint8_t* param, uint8_t* id, uint32_t* data) {
    uint8_t count = 1;
    bool error = false;
    last_filled_byte = PACKET_HEADER_SIZE;
    // количество параметров
    if (field_bit[0].get_value(&setting_) != 0) {
        field_byte.set_value(*amt, packet, last_filled_byte);
        count = *amt;
        ++last_filled_byte;
    }

    for (int i = 0; i < count; ++i) {
        // параметр
        if (!(param[i] < SENSOR_PARAM_DATA)) {
            param_ = 0xFF;
            return 2;
        }
        param_ = param[i];
        if (field_bit[1].get_value(&setting_) != 1) {
            error = field_byte.set_value(param_, packet, last_filled_byte);
            ++last_filled_byte;
        }
        if (error)
            return 3;

        // индивидуальный номер
        if (field_bit[2].get_value(&setting_) != 0) {
            error = field_byte.set_value(id[i], packet, last_filled_byte);
            ++last_filled_byte;
        }
        if (error)
            return 4;

        // данные
        switch (sensor_param_data[param_]) {
        case 1:
            error = field_byte.set_value(*(data + i), packet, last_filled_byte);
            last_filled_byte += 1;
            break;
        case 2:
            error = field_PWM.set_value(*(data + i), packet, last_filled_byte);
            last_filled_byte += 2;
            break;
        case 4:
            error = field_float.set_value(*(data + i), packet, last_filled_byte);
            last_filled_byte += 4;
            break;
        default:
            return 5;
            break;
        }
        if (error)
            return 6;
    }
    return 0;
}

uint8_t Packet_Sensor::get_packet_data(const LoRa_packet& packet, uint8_t* amt, uint8_t* param, uint8_t* id, uint32_t* data) {
    uint8_t count = 1;
    last_read_byte = PACKET_HEADER_SIZE;
    // количество параметров
    if (field_bit[0].get_value(&setting_) != 0) {
        count = field_byte.get_value(packet, last_read_byte);
        *amt = count;
        ++last_read_byte;
    }

    for (int i = 0; i < count; ++i) {
        // параметр
        if (field_bit[1].get_value(&setting_) != 1) {
            param_ = field_byte.get_value(packet, last_read_byte);
            ++last_read_byte;
        }
        else {
            param_ = param[i];
        }

        if (!(param_ < SENSOR_PARAM_DATA)) {
            param_ = 0xFF;
            return 2;
        }
        param[i] = param_;


        // индивидуальный номер
        if (field_bit[2].get_value(&setting_) != 0) {
            id[i] = field_byte.get_value(packet, last_read_byte);
            ++last_read_byte;
        }

        // данные
        switch (sensor_param_data[param_]) {
        case 1:
            data[i] = field_byte.get_value(packet, last_read_byte); // & 0xFF;
            last_read_byte += 1;
            break;
        case 2:
            data[i] = field_PWM.get_value(packet, last_read_byte); // & 0xFFFF; (?)
            last_read_byte += 2;
            break;
        case 4:
            data[i] = field_float.get_value(packet, last_read_byte);
            last_read_byte += 4;
            break;
        default:
            return 3;
        }
    }
    return 0;
}



uint8_t Packet_Sensor::get_size_by_data(const uint8_t *amt, const uint8_t *param, uint8_t &size_data) {
    last_read_byte = PACKET_HEADER_SIZE;
    size_data = 0;
    uint8_t count = 1;
    // количество параметров
    if (field_bit[0].get_value(&setting_) != 0) {
        count = *amt;
        ++last_read_byte;
    }

    for (int i = 0; i < count; ++i) {
        // параметр
        if (!(param[i] < SENSOR_PARAM_DATA)) {
            param_ = 0xFF;
            return 2;
        }
        param_ = param[i];

        if (field_bit[1].get_value(&setting_) != 1) {
            ++last_read_byte;
        }

        // индивидуальный номер
        if (field_bit[2].get_value(&setting_) != 0) {
            ++last_read_byte;
        }

        // данные
        switch (sensor_param_data[param_]) {
        case 1:
            size_data += 1;
            last_read_byte += 1;
            break;
        case 2:
            size_data += 2;
            last_read_byte += 2;
            break;
        case 4:
            size_data += 4;
            last_read_byte += 4;
            break;
        default:
            return 5;
            break;
        }
    }
    return 0;
}
uint8_t Packet_Sensor::get_size_by_packet(const LoRa_packet& packet, uint8_t *amt, uint8_t *param, uint8_t &size_data) {
    *amt = 1;
    size_data = 0;
    last_read_byte = PACKET_HEADER_SIZE;

    // количество параметров
    if (field_bit[0].get_value(&setting_) != 0) {
        *amt = field_byte.get_value(packet, last_read_byte);
        ++last_read_byte;
    }

    for (int i = 0; i < *amt; ++i) {
        if (field_bit[1].get_value(&setting_) != 1) {
            param_ = field_byte.get_value(packet, last_read_byte);
            if(param != nullptr)
                param[i] = param_;
            ++last_read_byte;
        }
        else {
            param_ = param[i];
        }
        if (!(param_ < SENSOR_PARAM_DATA)) {
            param_ = 0xFF;
            return 2;
        }


        // индивидуальный номер
        if (field_bit[2].get_value(&setting_) != 0) {
            ++last_read_byte;
        }

        // данные
        switch (sensor_param_data[param_]) {
        case 1:
            size_data += 1;
            last_read_byte += 1;
            break;
        case 2:
            size_data += 2;
            last_read_byte += 2;
            break;
        case 4:
            size_data += 4;
            last_read_byte += 4;
            break;
        default:
            return 3;
        }
    }
    return 0;
}


//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- ----- ----- -----  Packet_Device  ----- ----- ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----


bool Packet_Device::set_setting(uint8_t setting) {
    // if по типам
    setting_ = setting;
    return false;
}



uint8_t Packet_Device::set_packet_data(LoRa_packet& packet, uint8_t *obj, uint8_t *num, uint8_t *com, uint8_t *data, uint8_t *len) {
    bool error = false;
    last_filled_byte = PACKET_HEADER_SIZE;


    // объект
    if (!(*obj < DEVICE_OBJECT)) {
        object_ = 0xFF;
        return 2;
    }
    object_ = *obj;
    if (field_bit[0].get_value(&setting_) != 1) {
        error = field_byte.set_value(object_, packet, last_filled_byte);
        ++last_filled_byte;
    }
    if (error)
        return 3;


    // индивидуальный номер
    if (field_bit[1].get_value(&setting_) != 0) {
        error = field_byte.set_value(*num, packet, last_filled_byte);
        ++last_filled_byte;
    }
    if (error)
        return 4;

    // комманда
    if (!(*com < DEVICE_COMMAND[object_])) {
        command_ = 0xFF;
        return 5;
    }
    command_ = *com;
    error = field_byte.set_value(command_, packet, last_filled_byte);
    ++last_filled_byte;
    if (error)
        return 6;

    // данные
    for (int i = 0; i < device_object_data[object_][command_]; ++i) {
        error = field_byte.set_value(data[i], packet, last_filled_byte);
        ++last_filled_byte;

        if (error)
            return 7;
    }

    return 0;
}

uint8_t Packet_Device::get_packet_data(const LoRa_packet& packet, uint8_t *obj, uint8_t *num, uint8_t *com, uint8_t *data, uint8_t *len) {
    last_read_byte = PACKET_HEADER_SIZE;

    // объект
    if (field_bit[0].get_value(&setting_) != 1) {
        object_ = field_byte.get_value(packet, last_read_byte);
        ++last_read_byte;
    }
    else {
        object_ = *obj;
    }
    if (!(object_ < DEVICE_OBJECT)) {
        object_ = 0xFF;
        return 2;
    }
    *obj = object_;

    // индивидуальный номер
    if (field_bit[1].get_value(&setting_) != 0) {
        *num = field_byte.get_value(packet, last_read_byte);
        ++last_read_byte;
    }

    // комманда
    command_ = field_byte.get_value(packet, last_read_byte);
    ++last_read_byte;
    if (!(command_ < DEVICE_COMMAND[object_])) {
        command_ = 0xFF;
        return 3;
    }
    *com = command_;

    // данные
    if(len != nullptr)
        *len = 0;
    for (int i = 0; i < device_object_data[object_][command_]; ++i) {
        data[i] = field_byte.get_value(packet, last_read_byte);
        if(len != nullptr)
            ++*len;
        ++last_read_byte;
    }

    return 0;
}



    
uint8_t Packet_Device::get_size_by_data(const uint8_t *obj, const uint8_t *com, uint8_t &size_data) {
    last_read_byte = PACKET_HEADER_SIZE;
    size_data = 0;

    // объект
    if (!(*obj < DEVICE_OBJECT)) {
        object_ = 0xFF;
        return 1;
    }
    object_ = *obj;
    if (field_bit[0].get_value(&setting_) != 1) {
        ++last_read_byte;
    }

    // индивидуальный номер
    if (field_bit[1].get_value(&setting_) != 0) {
        ++last_read_byte;
    }

    // комманда
    if (!(*com < DEVICE_COMMAND[object_])) {
        command_ = 0xFF;
        return 2;
    }
    command_ = *com;
    ++last_read_byte;

    // данные
    last_read_byte += device_object_data[object_][command_];
    size_data += device_object_data[object_][command_];

    return 0;
}

uint8_t Packet_Device::get_size_by_packet(const LoRa_packet& packet, uint8_t *obj, uint8_t &size_data) {
    size_data = 0;
    last_read_byte = PACKET_HEADER_SIZE;

    // объект
    if (field_bit[0].get_value(&setting_) != 1) {
        object_ = field_byte.get_value(packet, last_read_byte);
        ++last_read_byte;
    }
    else {
        object_ = *obj;
    }
    if (!(object_ < DEVICE_OBJECT)) {
        object_ = 0xFF;
        return 2;
    }

    // индивидуальный номер
    if (field_bit[1].get_value(&setting_) != 0) {
        ++last_read_byte;
    }

    // комманда
    command_ = field_byte.get_value(packet, last_read_byte);
    ++last_read_byte;
    if (!(command_ < DEVICE_COMMAND[object_])) {
        command_ = 0xFF;
        return 3;
    }

    // данные
    size_data += device_object_data[object_][command_];
    last_read_byte += device_object_data[object_][command_];

    return 0;
}




//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- ----- ----- -----  Packet_System  ----- ----- ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----


bool Packet_System::set_setting(uint8_t setting) {
    // if по типам
    return true; // не реализованы
}



uint8_t Packet_System::set_packet_data(LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len) {
    bool error = false;
    last_filled_byte = PACKET_HEADER_SIZE;


    // комманда
    if (!(*com < SYSTEM_COMMAND_DATA)) {
        command_ = 0xFF;
        return 2;
    }
    command_ = *com;
    error = field_byte.set_value(command_, packet, last_filled_byte);
    ++last_filled_byte;
    if (error)
        return 3;

    // данные
    int _len;
    if (system_com_data[command_] != 0xFF) {
        _len = system_com_data[command_];
    }
    else {
        switch (command_)
        {
        case 0x00:
            // запись ID до длины поля с данными
            for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i) {
                error = field_byte.set_value(data[i], packet, last_filled_byte);
                ++last_filled_byte;
                if (error)
                    return 4;
            }
            data = data + AMT_BYTES_SYSTEM_ID;

            _len = *len;
            error = field_byte.set_value(_len, packet, last_filled_byte);
            data = data + 1;
            ++last_filled_byte;
            if (error)
                return 5;

            error = field_byte.set_value(data[0], packet, last_filled_byte);
            ++last_filled_byte;
            if (error)
                return 6;
            data = data + 1;

            break;
        default:
            return 7;
        }
    }

    for (int i = 0; i < _len; ++i) {
        error = field_byte.set_value(data[i], packet, last_filled_byte);
        ++last_filled_byte;
        if (error)
            return 8;
    }

    return 0;
}

uint8_t Packet_System::get_packet_data(const LoRa_packet& packet, uint8_t *com, uint8_t *data, uint8_t *len) {
    last_read_byte = PACKET_HEADER_SIZE;

    // комманда
    command_ = field_byte.get_value(packet, last_read_byte);
    ++last_read_byte;
    if (!(command_ < SYSTEM_COMMAND_DATA)) {
        command_ = 0xFF;
        return 2;
    }
    *com = command_;


    // данные
    int _len;
    if (system_com_data[command_] != 0xFF) {
        _len = system_com_data[command_];
        *len = _len;
    }
    else {
        switch (command_)
        {
        case 0x00:
            // получение ID до длины поля с данными
            for (int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i) {
                data[i] = field_byte.get_value(packet, last_read_byte);
                ++last_read_byte;
            }
            data = data + AMT_BYTES_SYSTEM_ID;

            // len
            data[0] = field_byte.get_value(packet, last_read_byte);
            ++last_read_byte;
            *len = _len = data[0];
            data = data + 1;
            // type
            data[0] = field_byte.get_value(packet, last_read_byte);
            ++last_read_byte;
            data = data + 1;

            break;
        default:
            return 4;
        }
    }

    for (int i = 0; i < _len; ++i) {
        data[i] = field_byte.get_value(packet, last_read_byte);
        ++last_read_byte;
    }
    return 0;
}


uint8_t Packet_System::get_size_by_data(const uint8_t *com, const uint8_t *len, uint8_t &size_data) {
    last_read_byte = PACKET_HEADER_SIZE;
    size_data = 0;

    // комманда
    if (!(*com < SYSTEM_COMMAND_DATA)) {
        command_ = 0xFF;
        return 1;
    }
    command_ = *com;
    ++last_read_byte;
    // данные
    int _len;
    if (system_com_data[command_] != 0xFF) {
        _len = system_com_data[command_];
    }
    else {
        switch (command_)
        {
        case 0x00:
            last_read_byte += AMT_BYTES_SYSTEM_ID + 2; // смещение на ID
            size_data += AMT_BYTES_SYSTEM_ID + 2;
            ++last_read_byte;
            _len = *len;
            break;
        default:
            return 5;
        }
    }
    size_data += _len;
    last_read_byte += _len;
    return 0;
}
uint8_t Packet_System::get_size_by_packet(const LoRa_packet& packet, uint8_t &size_data) {
    last_read_byte = PACKET_HEADER_SIZE;
    // комманда
    command_ = field_byte.get_value(packet, last_read_byte);
    ++last_read_byte;
    if (!(command_ < SYSTEM_COMMAND_DATA)) {
        command_ = 0xFF;
        return 2;
    }
    // данные
    if (system_com_data[command_] != 0xFF) {
        size_data = system_com_data[command_];
    }
    else {
        switch (command_)
        {
        case 0x00:
            last_read_byte += AMT_BYTES_SYSTEM_ID; // смещение на ID
            size_data = AMT_BYTES_SYSTEM_ID; // смещение на ID
            size_data += field_byte.get_value(packet, last_read_byte);
            last_read_byte += 2; // смещение на length и type
            size_data += 2; // смещение на length и type
            ++last_read_byte;
            break;
        default:
            return 4;
        }
    }
    last_read_byte += size_data;
    return 0;
}



//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- ----- ----- ----- Err ----- ----- ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

