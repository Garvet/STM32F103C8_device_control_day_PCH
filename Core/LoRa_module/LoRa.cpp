#include "LoRa.h"

class LoRa_class LoRa;

// modes
#define MODE_SLEEP 0x00 // Спящий режим
#define MODE_STDBY 0x01 // Режим ожидания
#define MODE_FSTX 0x02  // Синтез частот TX
#define MODE_TX 0x03    // Передача пакета
#define MODE_FSRX 0x04  // Синтез частот RX
#define MODE_RX_CONTINUOUS 0x05 // Непрерывное получение
#define MODE_RX_SINGLE 0x06 // Единичное получение
#define MODE_CAD 0x07 // Обнаружение активности канала

// PaDac
#define RF_PADAC_20DBM_ON  0x07
#define RF_PADAC_20DBM_OFF 0x04

// DetectionThreshold
#define DT_SF6 0x0C
#define DT_SF7_12 0x0A
// DetectionOptimize
#define DO_SF6 0x05
#define DO_SF7_12 0x03
// Frequency
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08

#define LoRa_DIO0_Pin
#define LoRa_DIO1_Pin

LoRa_class::LoRa_class(/* args */) {
	init(nullptr);
}
LoRa_class::LoRa_class(SPI_HandleTypeDef *spi) {
	init(spi);
}
LoRa_class::~LoRa_class() {
    if(_spi != nullptr)
        delete _spi;
}

void LoRa_class::init(SPI_HandleTypeDef *spi) {
	_reset_pin = LORA_RESET_Pin;
	_reset_port = LORA_RESET_GPIO_Port;
	_nss_pin = LORA_NSS_Pin;
	_nss_port = LORA_NSS_GPIO_Port;
#ifdef LoRa_DIO0_Pin
	_dio0_pin = LORA_DIO0_Pin;
	_dio0_port = LORA_DIO0_GPIO_Port;
#else
	_dio0_pin = 0;
	_dio0_port = 0;
#endif
#ifdef LoRa_DIO1_Pin
	_dio1_pin = LORA_DIO1_Pin;
	_dio1_port = LORA_DIO1_GPIO_Port;
#else
	_dio1_pin = 0;
	_dio1_port = 0;
#endif
#ifdef LoRa_DIO3_Pin
	_dio3_pin = LORA_DIO3_Pin;
	_dio3_port = LORA_DIO3_GPIO_Port;
#else
	_dio3_pin = 0;
	_dio3_port = 0;
#endif
	_spi = spi;
	_frequency = 0;
	_FifoTxBaseAddr = 0;
	_packet_length = 0;
	_reg_field.init(spi, _nss_port, _nss_pin);
}

uint8_t LoRa_class::field_set(Address_field field, uint32_t value, bool write) {
    uint8_t result;
    result = _reg_field.set_field_value(field, value);
    if(write) {
        result = _reg_field.register_write(field, true, true);
    }
    return result;
}
uint8_t LoRa_class::field_set(Address_field* fields, uint32_t* values, uint8_t amt, bool write) {
    uint8_t result;
    result = _reg_field.set_field_value(fields, values, amt);
    if(write) {
        result = _reg_field.register_write(fields, amt, true, true);
    }
    return result;
}

uint8_t LoRa_class::field_get(Address_field field, uint32_t* value, bool read) {
    return _reg_field.get_field_value(field, value, read);
}
uint8_t LoRa_class::field_get(Address_field fields[], uint32_t* values, uint8_t amt, bool read) {
    return _reg_field.get_field_value(fields, values, amt, read);
}


uint8_t LoRa_class::begin(uint64_t frequency, bool paboost, uint8_t signal_power, uint8_t SF, uint64_t SBW, uint8_t sync_word) {
    uint32_t result = 0;
    uint8_t amt;
    // настройка выходов
    HAL_GPIO_WritePin(_reset_port, _reset_pin, GPIO_PIN_SET);
    // запуск модуля
    HAL_GPIO_WritePin(_reset_port, _reset_pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(_reset_port, _reset_pin, GPIO_PIN_SET);
    HAL_Delay(50);
    // проверка версии LoRa-модуля
    amt = field_get(Version, &result);
    if ((result != 0x12) || (amt != 1)) return 2;
    // переход в режим сна/настройки
    if(mode_sleep() != 1) return 3;
    // установка частоты работы модуля
    if(set_frequency(frequency) != 3) return 4;
    // установка адресов памяти TX и RX
    if(field_set(FifoRxBaseAddr, 0x00) != 1) return 5;
    if(field_set(FifoTxBaseAddr, _FifoTxBaseAddr) != 1) return 6;
    // настройка LNA
    if(field_set(LnaBoostHf, 0x03) != 1) return 7;
    // установка автоматического AGC
    if(field_set(LowDataRateOptimize, 0) != 1) return 8;
    if(field_set(AgcAutoOn, 1) != 1) return 9;
    // установкасилы сигнала на 14 дБ
    if(set_TX_power(signal_power, paboost) != 2) return 10;
    // установка силы коэффициента распространения SF
    if(set_spreading_factor(SF) != 3) return 11;
    // установка пропускной способности
    if(set_signal_bandwidth(SBW) != 1) return 12;
    // установка кодового слова 0x4A,  0x34 - LoRaWan
    if(field_set(SyncWord, sync_word) != 1) return 13;
    // включение проверки ошибки пакета
    if(crc_enable() != 1) return 14;
    // переход в режим ожидания
    if(mode_STDBY() != 1) return 15;
    return 0;
}

void LoRa_class::end() {
    // переход в режим сна
    mode_sleep();
}


uint8_t LoRa_class::set_mode(uint8_t mode) {
    Address_field fields[3] = {LongRangeMode, LowFrequencyModeOn, Mode};
    uint32_t values[3] = {1, 0, mode};
    return field_set(fields, values, 3);
}
// Режим сна/настройки
uint8_t LoRa_class::mode_sleep() {
    return set_mode(MODE_SLEEP);
}
// Режим ожидания
uint8_t LoRa_class::mode_STDBY() {
    return set_mode(MODE_STDBY);
}
// Режим отправки
uint8_t LoRa_class::mode_TX(bool set_dio) {
    if (set_dio & (_dio0_pin != 0))
        field_set(Dio0Mapping, 1);
    return set_mode(MODE_TX);
}
uint8_t LoRa_class::mode_FSTX() {
    return set_mode(MODE_FSTX);
}

uint8_t LoRa_class::mode_FSRX() {
    return set_mode(MODE_FSRX);
}

// Режим непрерывного приёма
uint8_t LoRa_class::mode_RX_continuous(bool set_dio) {
    if (set_dio & ((_dio0_pin != 0) || (_dio1_pin != 0))) {
        field_set(Dio0Mapping, 0, false);
        field_set(Dio1Mapping, 0);
    }
    return set_mode(MODE_RX_CONTINUOUS);
}
// Режим единичного приёма
uint8_t LoRa_class::mode_RX_single(bool set_dio) {
    if (set_dio & ((_dio0_pin != 0) || (_dio1_pin != 0))) {
        field_set(Dio0Mapping, 0, false);
        field_set(Dio1Mapping, 0);
    }
    return set_mode(MODE_RX_SINGLE);
}
// Режим проверки сети
uint8_t LoRa_class::mode_CAD(bool set_dio) {
    if (set_dio & ((_dio0_pin != 0) || (_dio1_pin != 0))) {
        field_set(Dio0Mapping, 2, false);
        field_set(Dio1Mapping, 2);
    }
    return set_mode(MODE_CAD);
}


// Установка силы отправляемого пакета
uint8_t LoRa_class::set_TX_power(uint8_t power, bool paboost, uint8_t max_power) {
    Address_field fields[4] = {PaDac, PaSelect, MaxPower, OutputPower};
    uint32_t pa_dac, pa_select;
    int16_t power_adjustment, min_power_value, max_power_value;
    if (max_power < 0x01)
        max_power = 0x01;
    else if (max_power > 0x07)
        max_power = 0x07;
    _reg_field.register_read(fields, 4);
    // Изменение бита PABOOST
    if (paboost) {
        pa_select = 1;
        min_power_value = 2;
        max_power_value = 20;
        if (power > 17)
            power_adjustment = -5;
        else
            power_adjustment = -2;
    }
    else {
        pa_select = 0;
        min_power_value = -1;
        max_power_value = 14;
        power_adjustment = 1;
    }
    // Проверка выхода силы сигнала за диапазон
    if (power < min_power_value)
        power = min_power_value;
    if (power > max_power_value)
        power = max_power_value;
    // Корректировка параметра
    power += power_adjustment;
    // Настройка флага высокого сигнала
    if (power > 17)
        pa_dac = RF_PADAC_20DBM_ON;
    else
        pa_dac = RF_PADAC_20DBM_OFF;
    // Передача настроек
    uint32_t values[4] = {pa_dac, pa_select, max_power, power};
    return field_set(fields, values, 4);
}

// Установка частоты радиосигнала
uint8_t LoRa_class::set_frequency(uint64_t frequency) {
    _frequency = frequency;
    uint32_t frf = ((uint64_t)frequency << 19) / 32000000;
    return field_set(Frf, frf);
}

// Установка силы коэффициента распространения SF
uint8_t LoRa_class::set_spreading_factor(uint8_t SF) {
    uint32_t detection_optimize, detection_threshold;
    if (SF < 6)
        SF = 6;
    else if (SF > 12)
        SF = 12;
    if (SF == 6) {
        detection_optimize = DO_SF6;
        detection_threshold = DT_SF6;
    }
    else {
        detection_optimize = DO_SF7_12;
        detection_threshold = DT_SF7_12;
    }
    Address_field fields[3] = {DetectionOptimize, DetectionThreshold, SpreadingFactor};
    uint32_t values[3] = {detection_optimize, detection_threshold, SF};
    return field_set(fields, values, 3);
}

// Установка пропускной способности
uint8_t LoRa_class::set_signal_bandwidth(uint64_t sbw) {
    uint8_t bw;
    if (sbw <= 7.8E3)        bw = 0;
    else if (sbw <= 10.4E3)  bw = 1;
    else if (sbw <= 15.6E3)  bw = 2;
    else if (sbw <= 20.8E3)  bw = 3;
    else if (sbw <= 31.25E3) bw = 4;
    else if (sbw <= 41.7E3)  bw = 5;
    else if (sbw <= 62.5E3)  bw = 6;
    else if (sbw <= 125E3)   bw = 7;
    else if (sbw <= 250E3)   bw = 8;
    else                     bw = 9;
    return field_set(Bw, bw);
}

// Установка длины преамбулы
uint8_t LoRa_class::set_preamble_length(unsigned int length) {
    return field_set(PreambleLength, length);
}

// Установка кодового слова
uint8_t LoRa_class::set_sync_word(uint8_t SW) {
    return field_set(SyncWord, SW);
}

// Включение CRC на LoRa-модуле
uint8_t LoRa_class::crc_enable() {
    return field_set(RxPayloadCrcOn, 1);
}
// Выключение CRC на LoRa-модуле
uint8_t LoRa_class::crc_disable() {
    return field_set(RxPayloadCrcOn, 0);
}
volatile uint32_t work_read_flag[4] = {0, 0, 0, 0};
// Приём пакета
class LoRa_packet LoRa_class::receiver_packet(uint8_t count, ulong wait, bool rssi, bool snr) {
    class LoRa_packet send_packet;
    Address_field fields[3] = {RxTimeout, RxDone, PayloadCrcError};
    Address_field flags[3] = {RxDone, ValidHeader, PayloadCrcError};
    if(count <= 1) {
        if(count == 1)
            mode_RX_single();
        volatile uint8_t rx_done, rx_timeout, crc_err;
        uint8_t amt;
        bool signal = false;
        ulong time, start_time, read_time;
        volatile int pin_done, pin_timeout, pin_crc_err;
        uint32_t values[3] = {0, 0, 0};
        rx_done = rx_timeout = crc_err = 0;
        if(wait == 0)
            time = 0;
        else
            time = HAL_GetTick();
        for(start_time = time, read_time = time; (wait == 0) || (HAL_GetTick() - time < wait);) {
            // Считывание каждые 10 мс.
            if((wait == 0) || (HAL_GetTick() - read_time > 10U)) {
                // Если работают DIO выходы, то при HIGH хотя бы на одном из них, пускаем signal
            	if ((_dio0_pin != 0) && (_dio1_pin != 0)) {
                	pin_done = HAL_GPIO_ReadPin(_dio0_port, _dio0_pin);
                	pin_timeout = HAL_GPIO_ReadPin(_dio1_port, _dio1_pin);
                    if (_dio3_pin != 0) {
                    	pin_crc_err = HAL_GPIO_ReadPin(_dio3_port, _dio3_pin);
                    }
                    else {
                        pin_crc_err = GPIO_PIN_RESET;
                    }
                    if ((pin_done == GPIO_PIN_SET) || (pin_timeout == GPIO_PIN_SET) || (pin_crc_err == GPIO_PIN_SET)) {
                        signal = true;
                    }
            	}
                // Если неработают DIO выходы, был signal или превышено время ожидания
                if (((count == 0) && (wait == 0)) || (_dio0_pin == 0) || (_dio1_pin == 0) || (HAL_GetTick() - start_time > 2000) || signal) {
                    amt = field_get(fields, values, 3, true);
                    if(amt == 3) {
                        rx_timeout = values[0];
                        rx_done = values[1];
                        crc_err = values[2];
                    }
                    if(signal) {
                    	if(rx_timeout + rx_done + crc_err != 0) {
                    		work_read_flag[0]++; // +
                    	}
                    	else {
                    		work_read_flag[1]++; // -
                    	}
                    }
                    else {
                    	if(rx_timeout + rx_done + crc_err != 0) {
                    		work_read_flag[2]++; // -
                    	}
                    	else {
                    		work_read_flag[3]++; // +
                    	}
                    }
                }
                if(rx_done != 0) {
                    break;
                }
                if(rx_timeout != 0) {
                    _reg_field.clear_flags(RxTimeout);
                    mode_RX_single(false);
                    rx_done = rx_timeout = crc_err = 0;
                    signal = false;
                    if(wait != 0)
                        start_time = HAL_GetTick();
                }
                if(wait == 0) {
                    break;
                }
            }
        }
        if((rx_done > 0) /*&& (crc_err == 0)*/) {
            _reg_field.clear_flags(RxDone);
            _reg_field.clear_flags(RxTimeout);
            send_packet = read_packet_data(crc_err, rssi, snr);
        }
        else {
            field_get(fields, values, 3, true);
            _reg_field.clear_flags(flags, 3);
            // send_packet = LoRa_packet();
        }
        if(wait != 0)
            mode_sleep();
    }
    else {
        for(int i = 0; i < count; ++i) {

        }
    }
    return send_packet;
}

// Содержание последнего принятого пакета
class LoRa_packet LoRa_class::read_packet_data(bool crc_err, bool f_rssi, bool f_snr) {
    uint32_t length, adr;
    uint8_t rssi;//, *data;
    if (f_rssi)
        rssi = packet_rssi();
    else
        rssi = 0;

    field_get(FifoRxBytesNb, &length, true);
    field_get(FifoRxCurrentAddr, &adr, true);
    field_set(FifoAddrPtr, adr);

    class LoRa_packet send_packet(nullptr, 0, crc_err, rssi);
    uint32_t data32 = 0;
    for(unsigned int i = 0; i < length; ++i) {
        field_get(Fifo, &data32, true);
        send_packet.add_packet_data(data32);
        // send_packet.add_packet_data((uint8_t)(data32 & 0xFF));
        // data[i] = data32 & 0xFF;
    }
    // data = new uint8_t[length];
    // uint32_t data32 = 0;
    // for(int i = 0; i < length; ++i) {
    //     field_get(Fifo, &data32, true);
    //     data[i] = data32 & 0xFF;
    // }
    // class LoRa_packet send_packet(data, length, crc_err, rssi);
    // delete[] data;
    return send_packet;
}

// RSSI последнего принятого пакета
uint8_t LoRa_class::packet_rssi() {
    uint32_t rssi = 0;
    field_get(PacketRssi, &rssi, true);
    if (_frequency < 868E6)
        rssi -= 164;
    else
        rssi -= 157;
    return rssi;
}
// SNR последнего принятого пакета
float LoRa_class::packet_snr() {
    float snr = 0;
    field_get(PacketSnr, (uint32_t*)&snr, true);
    return (snr * 0.25);
}
// Отправка пакета
bool LoRa_class::sender_packet(uint8_t* packet, uint8_t len, uint32_t wait) {
    packet_begin();
    if (packet_write(packet, len))
        return true;
    if (packet_end(wait))
        return true;
    return false;
}
bool LoRa_class::sender_packet(std::vector<uint8_t> packet, uint32_t wait) {
    packet_begin();
    if (packet_write(packet))
        return true;
    if (packet_end(wait))
        return true;
    return false;
}
// Объявление пакета
bool LoRa_class::packet_begin() {
    mode_STDBY();
    field_set(FifoAddrPtr, _FifoTxBaseAddr);
    _packet_length = 0;
    return true;
}
// Отправка данных в пакет buffer, size=None? (len)
bool LoRa_class::packet_write(uint8_t* packet, uint8_t len) {
    if (len + _packet_length > 255)
        return true;
    _packet_length += len;
    for(int i = 0; i < len; ++i)
        field_set(Fifo, packet[i]);
    field_set(PayloadLength, _packet_length);
    return false;
}
bool LoRa_class::packet_write(std::vector<uint8_t> packet) {
    if (packet.size() + _packet_length > 255)
        return true;
    _packet_length += packet.size();
    for(int i = 0; i < (int)packet.size(); ++i)
        field_set(Fifo, packet[i]);
    field_set(PayloadLength, _packet_length);
    return false;
}

// Отправка пакета
bool LoRa_class::packet_end(uint32_t wait, bool sleep) {
    mode_TX();
    bool result = false;
    if(wait > 0) {
        uint32_t tx_done = 0;
        bool signal = false;
        uint32_t time, start_time, read_time;
        int pin_done;
        tx_done = 0;
        for(time = HAL_GetTick(), start_time = time, read_time = time; HAL_GetTick() - time < wait;) {
            // Считывание каждые 10 мс.
            if(HAL_GetTick() - read_time > 10) {
                // Если работают DIO выходы, то при HIGH хотя бы на одном из них, пускаем signal
                if (_dio0_pin != 0) {
                	pin_done = HAL_GPIO_ReadPin(_dio0_port, _dio0_pin);
                    if (pin_done == GPIO_PIN_SET) {
                        signal = true;
                    }
                }
                // Если неработают DIO выходы, был signal или превышено время ожидания
                if ((_dio0_pin == 0) || (HAL_GetTick() - start_time > 2000) || signal) {
                    field_get(TxDone, &tx_done, true);
                }
                if(tx_done != 0) {
                    break;
                }
            }
        }
        if(sleep && (tx_done != 0)) {
            mode_sleep();
        }
    }
    return result;
}
