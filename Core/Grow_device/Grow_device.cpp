#include <Grow_device.h>

extern const uint16_t LORA_ADDRESS_BRANCH;

uint8_t id_mas_devices[COUNT_TYPE_DEVICE];

Grow_device::Grow_device(uint8_t amt_component, enum Type_device* type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(int i = 0; i < amt_component; ++i)
        component_.push_back(Grow_device_component(type_device[i], (id_mas_devices[type_device[i]]++)));

    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x02; // 0000.0010 - бит индивидуального номера, в случае наличия повторов
        }
}
Grow_device::Grow_device(uint8_t amt_component, uint8_t* type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(int i = 0; i < amt_component; ++i)
        component_.push_back(Grow_device_component((enum Type_device)(type_device[i]), (id_mas_devices[type_device[i]]++)));

    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x02;
        }
}

Grow_device::Grow_device(const std::vector<enum Type_device>& type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(unsigned int i = 0; i < type_device.size(); ++i)
        component_.push_back(Grow_device_component(type_device[i], (id_mas_devices[type_device[i]]++)));

    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x02;
        }
}

// --- Поля класса-платы ---

void Grow_device::set_system_id(std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id) {
    system_id_ = system_id;
}
std::array<uint8_t, AMT_BYTES_SYSTEM_ID> Grow_device::get_system_id() const {
    return system_id_;
}

void Grow_device::set_active(uint8_t active) {
    if(active < 3)
        active_ = active;
}

uint8_t Grow_device::get_active() const {
    return active_;
}







// установка режима
void Grow_device::set_sleep_state(bool state) {
	sleep_state = state;
}
// получение режима
bool Grow_device::get_sleep_state() const {
	return sleep_state;
}






bool Grow_device::get_change_value() const {
    return change_value_;
}
void Grow_device::clear_change_value() {
    change_value_ = false;
}

bool Grow_device::set_address(uint16_t address) {
    if((address >= (1 << LORA_ADDRESS_BRANCH)) || (address == 0))
        return true;
    address_ = address;
    return false;
}
uint16_t Grow_device::get_address() const {
    return address_;
}

#if !defined( ESP32 )
bool Grow_device::set_address_control_module(LoRa_address address) {
	if(address.branch != 0)
		return true;
	address_control_module_ = address;
	return false;
}
LoRa_address Grow_device::get_address_control_module() const {
	return address_control_module_;
}
#endif

void Grow_device::set_setting(uint8_t setting) {
    setting_ = setting;
}
uint8_t Grow_device::get_setting() const {
    return setting_;
}

// --- Обработка времени ---

void Grow_device::set_period(unsigned long period) {
    period_ = period;
}
unsigned long Grow_device::get_period() const {
    return period_;
}

bool Grow_device::check_time(unsigned long time) {
    end_time_ = time;
    if ((end_time_ - read_time_) > period_)
        readout_signal_ = true;
    return readout_signal_;
}
void Grow_device::update() {
    read_time_ = end_time_;
}
bool Grow_device::read_signal(bool clear) {
    if (!clear)
        return readout_signal_;
    clear = readout_signal_;
    readout_signal_ = false;
    return clear;
}

// --- Поля компонентов ---

bool Grow_device::get_type(uint8_t num, enum Type_device &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_type();
    return false;
}
bool Grow_device::get_type(uint8_t num, uint8_t &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_type();
    return false;
}
std::vector<enum Type_device> Grow_device::get_type() const {
    std::vector<enum Type_device> type_device;
    for(int i = 0; i < get_count_component(); ++i)
        type_device.push_back(component_[i].get_type());
    return type_device;
}
bool Grow_device::get_id(uint8_t num, uint8_t &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_id();
    return false;
}
std::vector<uint8_t> Grow_device::get_id() const {
    std::vector<uint8_t> id;
    for(int i = 0; i < get_count_component(); ++i)
        id.push_back(component_[i].get_id());
    return id;
}

bool Grow_device::set_value(uint8_t num, uint16_t value) {
    if(get_count_component() <= num)
        return true;
    component_[num].set_value(value);
    change_value_ = true;
    return false;
}
bool Grow_device::get_value(uint8_t num, uint16_t &result) {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_value();
    return false;
}
bool Grow_device::set_value(const std::vector<uint16_t>& value) {
    if(get_count_component() != value.size())
        return true;
    for(int i = 0; i < get_count_component(); ++i)
        component_[i].set_value(value[i]);
    change_value_ = true;
    return false;
}
std::vector<uint16_t> Grow_device::get_value() {
    std::vector<uint16_t> value;
    for(int i = 0; i < get_count_component(); ++i)
        value.push_back(component_[i].get_value());
    return value;
}

// Установить флаг необходимости синхронизации времени
void Grow_device::set_rtc_sync(bool flag) {
    rtc_sync = flag;
}
// Получить флаг необходимости синхронизации времени
bool Grow_device::get_rtc_sync() const {
    return rtc_sync;
}

// --- Информации о компонентах ---

uint8_t Grow_device::get_count_component() const {
    return component_.size();
}

Grow_device_component Grow_device::get_component(uint8_t num) const {
    return component_[num];
}
std::vector<Grow_device_component> Grow_device::get_component() const {
    return component_;
}

// --- Внешняя связь ---

bool Grow_device::filter(Grow_device &device) {
    if(component_.size() != device.component_.size())
        return false;
    for(unsigned int i = 0; i < component_.size(); ++i) {
        if(!component_[i].filter(device.component_[i]))
            return false;
    }
    return true;
}



#ifdef NOT_USE

#include <Grow_device.h>

extern const uint16_t LORA_ADDRESS_BRANCH;

uint8_t id_mas_devices[COUNT_TYPE_DEVICE];

Grow_device::Grow_device(uint8_t amt_component, enum Type_device* type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(int i = 0; i < amt_component; ++i)
        component_.push_back(Grow_device_component(type_device[i], (id_mas_devices[type_device[i]]++)));
    
    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x04; // 0000.0100 - бит индивидуального номера, в случае наличия повторов
        }
}
Grow_device::Grow_device(uint8_t amt_component, uint8_t* type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(int i = 0; i < amt_component; ++i)
        component_.push_back(Grow_device_component((enum Type_device)(type_device[i]), (id_mas_devices[type_device[i]]++)));
    
    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x04;
        }
}

Grow_device::Grow_device(const std::vector<enum Type_device>& type_device) {
    for(int i = 0; i < AMT_BYTES_SYSTEM_ID; ++i)
        system_id_[i] = 0;
    address_ = 0xFFFF;
    setting_ = 0;
    active_ = false;
    change_value_ = false;
    for(int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        id_mas_devices[i] = 0;
    for(unsigned int i = 0; i < type_device.size(); ++i)
        component_.push_back(Grow_device_component(type_device[i], (id_mas_devices[type_device[i]]++)));
    
    for (int i = 0; i < COUNT_TYPE_DEVICE; ++i)
        if(id_mas_devices[i] > 1) {
            setting_ = 0x04;
        }
}

// --- Поля класса-платы ---

void Grow_device::set_system_id(std::array<uint8_t, AMT_BYTES_SYSTEM_ID> system_id) {
    system_id_ = system_id;
}
std::array<uint8_t, AMT_BYTES_SYSTEM_ID> Grow_device::get_system_id() const {
    return system_id_;
}

void Grow_device::set_active(uint8_t active) {
    if(active < 3)
        active_ = active;
}

uint8_t Grow_device::get_active() const {
    return active_;
}

bool Grow_device::get_change_value() const {
    return change_value_;
}
void Grow_device::clear_change_value() {
    change_value_ = false;
}

bool Grow_device::set_address(uint16_t address) {
    if((address >= (1 << LORA_ADDRESS_BRANCH)) || (address == 0))
        return true;
    address_ = address;
    return false;
}
uint16_t Grow_device::get_address() const {
    return address_;
}

#if !defined( ESP32 )
bool Grow_device::set_address_control_module(LoRa_address address) {
	if(address.branch != 0)
		return true;
	address_control_module_ = address;
	return false;
}
LoRa_address Grow_device::get_address_control_module() const {
	return address_control_module_;
}
#endif

void Grow_device::set_setting(uint8_t setting) {
    setting_ = setting;
}
uint8_t Grow_device::get_setting() const {
    return setting_;
}

// --- Обработка времени ---

void Grow_device::set_period(unsigned long period) {
    period_ = period;
}
unsigned long Grow_device::get_period() const {
    return period_;
}

bool Grow_device::check_time(unsigned long time) {
    end_time_ = time;
    if ((end_time_ - read_time_) > period_)
        readout_signal_ = true;
    return readout_signal_;
}
void Grow_device::update() {
    read_time_ = end_time_; 
}
bool Grow_device::read_signal(bool clear) {
    if (!clear)
        return readout_signal_;
    clear = readout_signal_;
    readout_signal_ = false;
    return clear;
}

// --- Поля компонентов ---

bool Grow_device::get_type(uint8_t num, enum Type_device &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_type();
    return false;
}
bool Grow_device::get_type(uint8_t num, uint8_t &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_type();
    return false;
}
std::vector<enum Type_device> Grow_device::get_type() const {
    std::vector<enum Type_device> type_device;
    for(int i = 0; i < get_count_component(); ++i)
        type_device.push_back(component_[i].get_type());
    return type_device;
}
bool Grow_device::get_id(uint8_t num, uint8_t &result) const {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_id();
    return false;
}
std::vector<uint8_t> Grow_device::get_id() const {
    std::vector<uint8_t> id;
    for(int i = 0; i < get_count_component(); ++i)
        id.push_back(component_[i].get_id());
    return id;
}

bool Grow_device::set_value(uint8_t num, float value) {
    if(get_count_component() <= num)
        return true;
    component_[num].set_value(value);
    change_value_ = true;
    return false;
}
bool Grow_device::get_value(uint8_t num, float &result) {
    if(get_count_component() <= num)
        return true;
    result = component_[num].get_value();
    return false;
}
bool Grow_device::set_value(const std::vector<float>& value) {
    if(get_count_component() != value.size())
        return true;
    for(int i = 0; i < get_count_component(); ++i)
        component_[i].set_value(value[i]);
    change_value_ = true;
    return false;
}
std::vector<float> Grow_device::get_value() {
    std::vector<float> value;
    for(int i = 0; i < get_count_component(); ++i)
        value.push_back(component_[i].get_value());
    return value;
}
// --- Информации о компонентах ---

uint8_t Grow_device::get_count_component() const {
    return component_.size();
}

Grow_device_component Grow_device::get_component(uint8_t num) const {
    return component_[num];
}
std::vector<Grow_device_component> Grow_device::get_component() const {
    return component_;
} 

// --- Внешняя связь ---

bool Grow_device::filter(Grow_device &device) {
    if(component_.size() != device.component_.size())
        return false;
    for(unsigned int i = 0; i < component_.size(); ++i) {
        if(!component_[i].filter(device.component_[i]))
            return false;
    }
    return true;
}
#endif
