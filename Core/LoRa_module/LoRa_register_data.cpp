#include "LoRa_register_data.h"

#if defined (ECONOMICAL_VERSION)

//   ----- ----- ----- - - - - - - - - - ----- ----- -----
// ----- ----- ----- Список регистров LoRa ----- ----- -----
//   ----- ----- ----- - - - - - - - - - ----- ----- -----
const uint8_t
REG_FIFO = 0x00,
// --- Общие регистры настроек ---
REG_OP_MODE = 0x01,
REG_FRF_MSB = 0x06,
REG_FRF_MID = 0x07,
REG_FRF_LSB = 0x08,
// --- Регистры для блоков RF ---
REG_PA_CONFIG = 0x09,
                                                                                                        // REG_PA_RAMP = 0x0A,
                                                                                                        // REG_LR_OCP = 0X0B,
REG_LNA = 0x0C,
// --- Страница регистров LoRa ---
REG_FIFO_ADDR_PTR = 0x0D,
REG_FIFO_TX_BASE_ADDR = 0x0E,
REG_FIFO_RX_BASE_ADDR = 0x0F,
REG_FIFO_RX_CURRENT_ADDR = 0x10,
                                                                                                        // REG_IRQ_FLAGS_MASK = 0x11,
REG_IRQ_FLAGS = 0x12,
REG_RX_NB_BYTES = 0x13,
                                                                                                        // REG_RX_HEADER_CNT_VALUE_MSB = 0x14,
                                                                                                        // REG_RX_HEADER_CNT_VALUE_LSB = 0x15,
                                                                                                        // REG_RX_PACKET_CNT_VALUE_MSB = 0x16,
                                                                                                        // REG_RX_PACKET_CNT_VALUE_LSB = 0x17,
                                                                                                        // REG_MODEM_STAT = 0x18,
REG_PKT_SNR_VALUE = 0x19,
REG_PKT_RSSI_VALUE = 0x1A,
                                                                                                        // REG_RSSI_VALUE = 0x1B,
                                                                                                        // REG_HOP_CHANNEL = 0x1C,
REG_MODEM_CONFIG_1 = 0x1D,
REG_MODEM_CONFIG_2 = 0x1E,
                                                                                                        // REG_SYMB_TIMEOUT_LSB = 0x1F,
REG_PREAMBLE_MSB = 0x20,
REG_PREAMBLE_LSB = 0x21,
REG_PAYLOAD_LENGTH = 0x22,
                                                                                                        // REG_MAX_PAYLOAD_LENGTH = 0x23,
                                                                                                        // REG_HOP_PERIOD = 0x24,
                                                                                                        // REG_FIFI_RX_BYTE_ADDR = 0x25,
REG_MODEM_CONFIG_3 = 0x26,
                                                                                                        // REG_PPM_CORRECTION = 0x27,
                                                                                                        // REG_FEI_MSB = 0x28,
                                                                                                        // REG_FEI_MID = 0x29,
                                                                                                        // REG_FEI_LSB = 0x2A,
                                                                                                        // REG_RSSI_WIDEBAND = 0x2C,
REG_DETECTION_OPTIMIZE = 0x31,
                                                                                                        // REG_INVERT_IQ = 0x33,
REG_DETECTION_THRESHOLD = 0x37,
REG_SYNC_WORD = 0x39,
// --- Регистры управления IO ---
REG_DIO_MAPPING_1 = 0x40,
REG_DIO_MAPPING_2 = 0x41,
// --- Регистр версий ---
REG_VERSION = 0x42,
// --- Дополнительные регистры ---
                                                                                                        // REG_PLL_HOP = 0x44,
                                                                                                        // REG_TCXO = 0x4B,
REG_PA_DAC = 0x4D//,
                                                                                                        // REG_FORMER_TEMP = 0x5B,
                                                                                                        // REG_BITRATE_FRAC = 0x5D,
                                                                                                        // REG_AGC_REF = 0x61,
                                                                                                        // REG_AGC_THRESH_1 = 0x62,
                                                                                                        // REG_AGC_THRESH_2 = 0x63,
                                                                                                        // REG_AGC_THRESH_3 = 0x64,
                                                                                                        // REG_PLL_HF = 0x70
;


//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
const Register
// --- --- --- Регистр очереди --- --- ---
// - REG_FIFO -
reg_fifo[1] = {Register(REG_FIFO, 8, 0)},

// --- --- --- Общие регистры настроек --- --- ---
// - REG_OP_MODE -
reg_long_range_mode[1] = {Register(REG_OP_MODE, 1, 7)},
                                                                                                        // reg_access_shared_reg[1] = {Register(REG_OP_MODE, 1, 6)},
reg_low_frequency_mode_on[1] = {Register(REG_OP_MODE, 1, 3)},
reg_mode[1] = {Register(REG_OP_MODE, 3, 0)},

// - REG_FRF_* -
reg_frf[3] = {Register(REG_FRF_LSB, 8, 0),
              Register(REG_FRF_MID, 8, 0),
              Register(REG_FRF_MSB, 8, 0)},

// --- --- --- Регистры для блоков RF --- --- ---
// - REG_PA_CONFIG -
reg_pa_select[1] = {Register(REG_PA_CONFIG, 1, 7)},
reg_max_power[1] = {Register(REG_PA_CONFIG, 3, 4)},
reg_output_power[1] = {Register(REG_PA_CONFIG, 4, 0)},
                                                                                                        // // - REG_PA_RAMP -
                                                                                                        // reg_pa_ramp[1] = {Register(REG_PA_RAMP, 4, 0)},
                                                                                                        // // - REG_LR_OCP -
                                                                                                        // reg_ocp_on[1] = {Register(REG_LR_OCP, 1, 5)},
                                                                                                        // reg_ocp_trim[1] = {Register(REG_LR_OCP, 5, 0)},
// - REG_LNA -
                                                                                                        // reg_lna_gain[1] = {Register(REG_LNA, 3, 5)},
                                                                                                        // reg_lna_boost_lf[1] = {Register(REG_LNA, 2, 3)},
reg_lna_boost_hf[1] = {Register(REG_LNA, 2, 0)},


// --- --- --- Страница регистров LoRa --- --- ---
// - REG_FIFO_ADDR_PTR -
reg_fifo_addr_ptr[1] = {Register(REG_FIFO_ADDR_PTR, 8, 0)},
// - REG_FIFO_TX_BASE_ADDR -
reg_fifo_tx_base_addr[1] = {Register(REG_FIFO_TX_BASE_ADDR, 8, 0)},
// - REG_FIFO_RX_BASE_ADDR -
reg_fifo_rx_base_addr[1] = {Register(REG_FIFO_RX_BASE_ADDR, 8, 0)},
// - REG_FIFO_RX_CURRENT_ADDR -
reg_fifo_rx_current_addr[1] = {Register(REG_FIFO_RX_CURRENT_ADDR, 8, 0)},
                                                                                                        // // - REG_IRQ_FLAGS_MASK -
                                                                                                        // reg_rx_timeout_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 7)},
                                                                                                        // reg_rx_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 6)},
                                                                                                        // reg_payload_crc_error_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 5)},
                                                                                                        // reg_valid_header_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 4)},
                                                                                                        // reg_tx_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 3)},
                                                                                                        // reg_cad_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 2)},
                                                                                                        // reg_fhss_change_channel_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 1)},
                                                                                                        // reg_cad_detected_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 0)},
// - REG_IRQ_FLAGS -
reg_rx_timeout[1] = {Register(REG_IRQ_FLAGS, 1, 7)},
reg_rx_done[1] = {Register(REG_IRQ_FLAGS, 1, 6)},
reg_payload_crc_error[1] = {Register(REG_IRQ_FLAGS, 1, 5)},
reg_valid_header[1] = {Register(REG_IRQ_FLAGS, 1, 4)},
reg_tx_done[1] = {Register(REG_IRQ_FLAGS, 1, 3)},
reg_cad_done[1] = {Register(REG_IRQ_FLAGS, 1, 2)},
reg_fhss_change_channel[1] = {Register(REG_IRQ_FLAGS, 1, 1)},
reg_cad_detected[1] = {Register(REG_IRQ_FLAGS, 1, 0)},
// - REG_RX_NB_BYTES -
reg_fifo_rx_bytes_nb[1] = {Register(REG_RX_NB_BYTES, 8, 0)},
                                                                                                        // // - REG_RX_HEADER_CNT_VALUE_* -
                                                                                                        // reg_valid_header_cnt[2] = {Register(REG_RX_HEADER_CNT_VALUE_LSB, 8, 0),
                                                                                                        //                            Register(REG_RX_HEADER_CNT_VALUE_MSB, 8, 0)},
                                                                                                        // // - REG_RX_PACKET_CNT_VALUE_* -
                                                                                                        // reg_valid_packet_cnt[2] = {Register(REG_RX_PACKET_CNT_VALUE_LSB, 8, 0),
                                                                                                        //                            Register(REG_RX_PACKET_CNT_VALUE_MSB, 8, 0)},
                                                                                                        // // - REG_MODEM_STAT -
                                                                                                        // reg_rx_coding_rate[1] = {Register(REG_MODEM_STAT, 3, 5)},
                                                                                                        // reg_modem_status[1] = {Register(REG_MODEM_STAT, 5, 0)},
// - REG_PKT_SNR_VALUE -
reg_packet_snr[1] = {Register(REG_PKT_SNR_VALUE, 8, 0)},
// - REG_PKT_RSSI_VALUE -
reg_packet_rssi[1] = {Register(REG_PKT_RSSI_VALUE, 8, 0)},
                                                                                                        // // - REG_RSSI_VALUE -
                                                                                                        // reg_rssi[1] = {Register(REG_RSSI_VALUE, 8, 0)},
                                                                                                        // // - REG_HOP_CHANNEL -
                                                                                                        // reg_pll_timeout[1] = {Register(REG_HOP_CHANNEL, 1, 7)},
                                                                                                        // reg_crc_on_payload[1] = {Register(REG_HOP_CHANNEL, 1, 6)},
                                                                                                        // reg_fhss_present_channel[1] = {Register(REG_HOP_CHANNEL, 6, 0)},
// - REG_MODEM_CONFIG_1 -
reg_bw[1] = {Register(REG_MODEM_CONFIG_1, 4, 4)},
                                                                                                        // reg_coding_rate[1] = {Register(REG_MODEM_CONFIG_1, 3, 1)},
                                                                                                        // reg_implicit_header_mode_on[1] = {Register(REG_MODEM_CONFIG_1, 1, 0)},
// - REG_MODEM_CONFIG_2 -
reg_spreading_factor[1] = {Register(REG_MODEM_CONFIG_2, 4, 4)},
                                                                                                        // reg_tx_continuous_mode[1] = {Register(REG_MODEM_CONFIG_2, 1, 3)},
reg_rx_payload_crc_on[1] = {Register(REG_MODEM_CONFIG_2, 1, 2)},
                                                                                                        // // - REG_SYMB_TIMEOUT_* -
                                                                                                        // reg_symb_timeout[2] = {Register(REG_SYMB_TIMEOUT_LSB, 8, 0),
                                                                                                        //                        Register(REG_MODEM_CONFIG_2, 2, 0)},
// - REG_PREAMBLE_* -
reg_preamble_length[2] = {Register(REG_PREAMBLE_LSB, 8, 0),
                          Register(REG_PREAMBLE_MSB, 8, 0)},
// - REG_PAYLOAD_LENGTH -
reg_payload_length[1] = {Register(REG_PAYLOAD_LENGTH, 8, 0)},
                                                                                                        // // - REG_MAX_PAYLOAD_LENGTH -
                                                                                                        // reg_payload_max_length[1] = {Register(REG_MAX_PAYLOAD_LENGTH, 8, 0)},
                                                                                                        // // - REG_HOP_PERIOD -
                                                                                                        // reg_freq_hopping_period[1] = {Register(REG_HOP_PERIOD, 8, 0)},
                                                                                                        // // - REG_FIFI_RX_BYTE_ADDR -
                                                                                                        // reg_fifo_rx_byte_addr_ptr[1] = {Register(REG_FIFI_RX_BYTE_ADDR, 8, 0)},
// - REG_MODEM_CONFIG_3 -
reg_low_data_rate_optimize[1] = {Register(REG_MODEM_CONFIG_3, 1, 3)},
reg_agc_auto_on[1] = {Register(REG_MODEM_CONFIG_3, 1, 2)},
                                                                                                        // // - REG_PPM_CORRECTION -
                                                                                                        // reg_ppm_correction[1] = {Register(REG_PPM_CORRECTION, 8, 0)},
                                                                                                        // // - REG_FEI_* -
                                                                                                        // reg_freq_error[3] = {Register(REG_FEI_LSB, 8, 0),
                                                                                                        //                      Register(REG_FEI_MID, 8, 0),
                                                                                                        //                      Register(REG_FEI_MSB, 4, 0)},
                                                                                                        // // - REG_RSSI_WIDEBAND -
                                                                                                        // reg_rssi_wideband[1] = {Register(REG_RSSI_WIDEBAND, 8, 0)},
// - REG_DETECTION_OPTIMIZE -
reg_detection_optimize[1] = {Register(REG_DETECTION_OPTIMIZE, 3, 0)},
                                                                                                        // // - REG_INVERT_IQ -
                                                                                                        // reg_invert_iq[1] = {Register(REG_INVERT_IQ, 1, 6)},
// - REG_DETECTION_THRESHOLD -
reg_detection_threshold[1] = {Register(REG_DETECTION_THRESHOLD, 8, 0)},
// - REG_SYNC_WORD -
reg_sync_word[1] = {Register(REG_SYNC_WORD, 8, 0)},  // , reserved_value=[0x34])

// --- --- --- Регистры управления IO --- --- ---
// - REG_DIO_MAPPING_1 -
reg_dio0_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 6)},
reg_dio1_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 4)},
                                                                                                        // reg_dio2_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 2)},
                                                                                                        // reg_dio3_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 0)},
// - REG_DIO_MAPPING_2 -
                                                                                                        // reg_dio4_mapping[1] = {Register(REG_DIO_MAPPING_2, 2, 6)},
                                                                                                        // reg_dio5_mapping[1] = {Register(REG_DIO_MAPPING_2, 2, 4)},
reg_map_preamble_detect[1] = {Register(REG_DIO_MAPPING_2, 1, 0)},

// --- --- --- Регистр версий --- --- ---
// - REG_VERSION -
reg_version[1] = {Register(REG_VERSION, 8, 0)},

// --- --- --- Дополнительные регистры --- --- ---
                                                                                                        // // - REG_PLL_HOP -
                                                                                                        // reg_fast_hop_on[1] = {Register(REG_PLL_HOP, 1, 7)},
                                                                                                        // // - REG_TCXO -
                                                                                                        // reg_tcxo_input_on[1] = {Register(REG_TCXO, 1, 4)},
// - REG_PA_DAC -
reg_pa_dac[1] = {Register(REG_PA_DAC, 3, 0)}//,
                                                                                                        // // - REG_FORMER_TEMP -
                                                                                                        // reg_former_temp[1] = {Register(REG_FORMER_TEMP, 8, 0)},
                                                                                                        // // - REG_BITRATE_FRAC -
                                                                                                        // reg_bit_rate_frac[1] = {Register(REG_BITRATE_FRAC, 4, 0)},
                                                                                                        // // - REG_AGC_REF -
                                                                                                        // reg_agc_reference_level[1] = {Register(REG_AGC_REF, 6, 0)},
                                                                                                        // // - REG_AGC_THRESH_1 -
                                                                                                        // reg_agc_step1[1] = {Register(REG_AGC_THRESH_1, 5, 0)},
                                                                                                        // // - REG_AGC_THRESH_2 -
                                                                                                        // reg_agc_step2[1] = {Register(REG_AGC_THRESH_2, 4, 4)},
                                                                                                        // reg_agc_step3[1] = {Register(REG_AGC_THRESH_2, 4, 0)},
                                                                                                        // // - REG_AGC_THRESH_3 -
                                                                                                        // reg_agc_step4[1] = {Register(REG_AGC_THRESH_3, 4, 4)},
                                                                                                        // reg_agc_step5[1] = {Register(REG_AGC_THRESH_3, 4, 0)},
                                                                                                        // // - REG_PLL_HF -
                                                                                                        // reg_pll_bandwidth[1] = {Register(REG_PLL_HF, 2, 6)}
;

//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- -----  зарезервированные значения  ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
const uint32_t
// --- --- --- Регистры для блоков RF --- --- ---
// - REG_LNA -
reserv_lna_boost_hf[2] = {0x01, 0x02},
// --- --- --- Страница регистров LoRa --- --- ---
// - REG_DETECTION_OPTIMIZE -
reserv_detection_optimize[1] = {0x04},
// - REG_DETECTION_THRESHOLD -
reserv_detection_threshold[1] = {0x0B},
// - REG_SYNC_WORD -
// reg_sync_word[1] = {0x34},
// --- --- --- Дополнительные регистры --- --- ---
// - REG_PA_DAC -
reserv_pa_dac[2] = {0x05, 0x06};


// (Register *registers, uint8_t reg_count=1, bool reg_revers=false, char mode='w', uint32_t min_value=0,
//                   uint32_t (-1), uint32_t *reserved_value=nullptr, uint32_t reserv_count=0);


//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Поля и регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
const Address_field
// --- --- --- Регистр очереди --- --- ---
// - REG_FIFO -
Fifo(reg_fifo, 1, false, 'w', 0x00, 0xFF),

// --- --- --- Общие регистры настроек --- --- ---
// - REG_OP_MODE -
LongRangeMode(reg_long_range_mode, 1, false, 'w', 0x00, 0x01),
                                                                                                        // AccessSharedReg(reg_access_shared_reg, 1, false, 'w', 0x00, 0x01),
LowFrequencyModeOn(reg_low_frequency_mode_on, 1, false, 'w', 0x00, 0x01),
Mode(reg_mode, 1, false, 'w', 0x00, 0x07),
// - REG_FRF_* -
Frf(reg_frf, 3, false, 'w', 0x000000, 0xFFFFFF),

// --- --- --- Регистры для блоков RF --- --- ---
// - REG_PA_CONFIG -
PaSelect(reg_pa_select, 1, false, 'w', 0x00, 0x01),
MaxPower(reg_max_power, 1, false, 'w', 0x00, 0x07),
OutputPower(reg_output_power, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // // - REG_PA_RAMP -
                                                                                                        // PaRamp(reg_pa_ramp, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // // - REG_LR_OCP -
                                                                                                        // OcpOn(reg_ocp_on, 1, false, 'w', 0x00, 0x01),
                                                                                                        // OcpTrim(reg_ocp_trim, 1, false, 'w', 0x00, 0x1F),
// - REG_LNA -
                                                                                                        // LnaGain(reg_lna_gain, 1, false, 'w', 0x01, 0x06),
                                                                                                        // LnaBoostLf(reg_lna_boost_lf, 1, false, 'w', 0x00, 0x00),
LnaBoostHf(reg_lna_boost_hf, 1, false, 'w', 0x00, 0x03, reserv_lna_boost_hf, 2),

// --- --- --- Страница регистров LoRa --- --- ---
// - REG_FIFO_ADDR_PTR -
FifoAddrPtr(reg_fifo_addr_ptr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_TX_BASE_ADDR -
FifoTxBaseAddr(reg_fifo_tx_base_addr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_RX_BASE_ADDR -
FifoRxBaseAddr(reg_fifo_rx_base_addr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_RX_CURRENT_ADDR -
FifoRxCurrentAddr(reg_fifo_rx_current_addr, 1, false, 'r', 0x00, 0xFF),
                                                                                                        // // - REG_IRQ_FLAGS_MASK -
                                                                                                        // RxTimeoutMask(reg_rx_timeout_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // RxDoneMask(reg_rx_done_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // PayloadCrcErrorMask(reg_payload_crc_error_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // ValidHeaderMask(reg_valid_header_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // TxDoneMask(reg_tx_done_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // CadDoneMask(reg_cad_done_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // FhssChangeChannelMask(reg_fhss_change_channel_mask, 1, false, 'w', 0x00, 0x01),
                                                                                                        // CadDetectedMask(reg_cad_detected_mask, 1, false, 'w', 0x00, 0x01),
// - REG_IRQ_FLAGS -
RxTimeout(reg_rx_timeout, 1, false, 'c', 0x00, 0x01),
RxDone(reg_rx_done, 1, false, 'c', 0x00, 0x01),
PayloadCrcError(reg_payload_crc_error, 1, false, 'c', 0x00, 0x01),
ValidHeader(reg_valid_header, 1, false, 'c', 0x00, 0x01),
TxDone(reg_tx_done, 1, false, 'c', 0x00, 0x01),
CadDone(reg_cad_done, 1, false, 'c', 0x00, 0x01),
FhssChangeChannel(reg_fhss_change_channel, 1, false, 'c', 0x00, 0x01),
CadDetected(reg_cad_detected, 1, false, 'c', 0x00, 0x01),
// - REG_RX_NB_BYTES -
FifoRxBytesNb(reg_fifo_rx_bytes_nb, 1, false, 'r', 0x00, 0xFF),
                                                                                                        // // - REG_RX_HEADER_CNT_VALUE_* -
                                                                                                        // ValidHeaderCnt(reg_valid_header_cnt, 2, false, 'r', 0x0000, 0xFFFF),
                                                                                                        // // - REG_RX_PACKET_CNT_VALUE_* -
                                                                                                        // ValidPacketCnt(reg_valid_packet_cnt, 2, false, 'c', 0x0000, 0xFFFF),
                                                                                                        // // - REG_MODEM_STAT -
                                                                                                        // RxCodingRate(reg_rx_coding_rate, 1, false, 'r', 0x00, 0x07),
                                                                                                        // ModemStatus(reg_modem_status, 1, false, 'r', 0x00, 0x1F),
// - REG_PKT_SNR_VALUE -
PacketSnr(reg_packet_snr, 1, false, 'r', 0x00, 0xFF),
// - REG_PKT_RSSI_VALUE -
PacketRssi(reg_packet_rssi, 1, false, 'r', 0x00, 0xFF),
                                                                                                        // // - REG_RSSI_VALUE -
                                                                                                        // Rssi(reg_rssi, 1, false, 'r', 0x00, 0xFF),
                                                                                                        // // - REG_HOP_CHANNEL -
                                                                                                        // PllTimeout(reg_pll_timeout, 1, false, 'r', 0x00, 0x01),
                                                                                                        // CrcOnPayload(reg_crc_on_payload, 1, false, 'r', 0x00, 0x01),
                                                                                                        // FhssPresentChannel(reg_fhss_present_channel, 1, false, 'r', 0x00, 0x3F),
// - REG_MODEM_CONFIG_1 -
Bw(reg_bw, 1, false, 'w', 0x00, 0x09),
                                                                                                        // CodingRate(reg_coding_rate, 1, false, 'w', 0x01, 0x04),
                                                                                                        // ImplicitHeaderModeOn(reg_implicit_header_mode_on, 1, false, 'w', 0x00, 0x01),
// - REG_MODEM_CONFIG_2 -
SpreadingFactor(reg_spreading_factor, 1, false, 'w', 0x06, 0x0C),
                                                                                                        // TxContinuousMode(reg_tx_continuous_mode, 1, false, 'w', 0x00, 0x01),
RxPayloadCrcOn(reg_rx_payload_crc_on, 1, false, 'w', 0x00, 0x01),
                                                                                                        // // - REG_SYMB_TIMEOUT_* -
                                                                                                        // SymbTimeout(reg_symb_timeout, 2, false, 'w', 0x0000, 0x03FF),
// - REG_PREAMBLE_* -
PreambleLength(reg_preamble_length, 2, false, 'w', 0x0000, 0xFFFF),
// - REG_PAYLOAD_LENGTH -
PayloadLength(reg_payload_length, 1, false, 'w', 0x01, 0xFF),
                                                                                                        // // - REG_MAX_PAYLOAD_LENGTH -
                                                                                                        // PayloadMaxLength(reg_payload_max_length, 1, false, 'w', 0x00, 0xFF),
                                                                                                        // // - REG_HOP_PERIOD -
                                                                                                        // FreqHoppingPeriod(reg_freq_hopping_period, 1, false, 'w', 0x00, 0xFF),
                                                                                                        // // - REG_FIFI_RX_BYTE_ADDR -
                                                                                                        // FifoRxByteAddrPtr(reg_fifo_rx_byte_addr_ptr, 1, false, 'r', 0x00, 0xFF),
// - REG_MODEM_CONFIG_3 -
LowDataRateOptimize(reg_low_data_rate_optimize, 1, false, 'w', 0x00, 0x01),
AgcAutoOn(reg_agc_auto_on, 1, false, 'w', 0x00, 0x01),
                                                                                                        // // - REG_PPM_CORRECTION -
                                                                                                        // PpmCorrection(reg_ppm_correction, 1, false, 'w', 0x00, 0xFF),
                                                                                                        // // - REG_FEI_* -
                                                                                                        // FreqError(reg_freq_error, 3, false, 'r', 0x000000, 0x0FFFFF),
                                                                                                        // // - REG_RSSI_WIDEBAND -
                                                                                                        // RssiWideband(reg_rssi_wideband, 1, false, 'r', 0x00, 0xFF),
// - REG_DETECTION_OPTIMIZE -
DetectionOptimize(reg_detection_optimize, 1, false, 'w', 0x03, 0x05, reserv_detection_optimize, 1),
                                                                                                        // // - REG_INVERT_IQ -
                                                                                                        // InvertIQ(reg_invert_iq, 1, false, 'w', 0x00, 0x01),
// - REG_DETECTION_THRESHOLD -
DetectionThreshold(reg_detection_threshold, 1, false, 'w', 0x0A, 0x0C, reserv_detection_threshold, 1),
// - REG_SYNC_WORD -
SyncWord(reg_sync_word, 1, false, 'w', 0x00, 0xFF),  // , reserv_sync_word, 1),

// --- --- --- Регистры управления IO --- --- ---
// - REG_DIO_MAPPING_1 -
Dio0Mapping(reg_dio0_mapping, 1, false, 'w', 0x00, 0x03),
Dio1Mapping(reg_dio1_mapping, 1, false, 'w', 0x00, 0x03),
                                                                                                        // Dio2Mapping(reg_dio2_mapping, 1, false, 'w', 0x00, 0x03),
                                                                                                        // Dio3Mapping(reg_dio3_mapping, 1, false, 'w', 0x00, 0x03),
// - REG_DIO_MAPPING_2 -
                                                                                                        // Dio4Mapping(reg_dio4_mapping, 1, false, 'w', 0x00, 0x03),
                                                                                                        // Dio5Mapping(reg_dio5_mapping, 1, false, 'w', 0x00, 0x03),
MapPreambleDetect(reg_map_preamble_detect, 1, false, 'w', 0x00, 0x01),

// --- --- --- Регистр версий --- --- ---
// - REG_VERSION -
Version(reg_version, 1, false, 'r', 0x00, 0xFF),

// --- --- --- Дополнительные регистры --- --- ---
                                                                                                        // // - REG_PLL_HOP -
                                                                                                        // FastHopOn(reg_fast_hop_on, 1, false, 'w', 0x00, 0x01),
                                                                                                        // // - REG_TCXO -
                                                                                                        // TcxoInputOn(reg_tcxo_input_on, 1, false, 'w', 0x00, 0x01),
// - REG_PA_DAC -
PaDac(reg_pa_dac, 1, false, 'w', 0x04, 0x07, reserv_pa_dac, 2) //,
                                                                                                        // // - REG_FORMER_TEMP -
                                                                                                        // FormerTemp(reg_former_temp, 1, false, 'r', 0x00, 0xFF),
                                                                                                        // // - REG_BITRATE_FRAC -
                                                                                                        // BitRateFrac(reg_bit_rate_frac, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // // - REG_AGC_REF -
                                                                                                        // AgcReferenceLevel(reg_agc_reference_level, 1, false, 'w', 0x00, 0x3F),
                                                                                                        // // - REG_AGC_THRESH_1 -
                                                                                                        // AgcStep1(reg_agc_step1, 1, false, 'w', 0x00, 0x1F),
                                                                                                        // // - REG_AGC_THRESH_2 -
                                                                                                        // AgcStep2(reg_agc_step2, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // AgcStep3(reg_agc_step3, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // // - REG_AGC_THRESH_3 -
                                                                                                        // AgcStep4(reg_agc_step4, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // AgcStep5(reg_agc_step5, 1, false, 'w', 0x00, 0x0F),
                                                                                                        // // - REG_PLL_HF -
                                                                                                        // PllBandwidth(reg_pll_bandwidth, 1, false, 'w', 0x00, 0x03)
;



// Данные о всех регистрах/полях

const int AMT_ALL_REGISTER = 28, AMT_ALL_FIELD = 36;
// const int AMT_ALL_REGISTER = 57, AMT_ALL_FIELD = 86;

const uint8_t ALL_REGISTER[28] = { // ALL_REGISTER[57]
    REG_FIFO, REG_OP_MODE, REG_FRF_MSB, REG_FRF_MID, REG_FRF_LSB, REG_PA_CONFIG,
    // REG_PA_RAMP, REG_LR_OCP,
    REG_LNA, REG_FIFO_ADDR_PTR, REG_FIFO_TX_BASE_ADDR, REG_FIFO_RX_BASE_ADDR, REG_FIFO_RX_CURRENT_ADDR,
    // REG_IRQ_FLAGS_MASK,
    REG_IRQ_FLAGS, REG_RX_NB_BYTES,
    // REG_RX_HEADER_CNT_VALUE_MSB, REG_RX_HEADER_CNT_VALUE_LSB, REG_RX_PACKET_CNT_VALUE_MSB, REG_RX_PACKET_CNT_VALUE_LSB, REG_MODEM_STAT,
    REG_PKT_SNR_VALUE, REG_PKT_RSSI_VALUE,
    // REG_RSSI_VALUE, REG_HOP_CHANNEL,
    REG_MODEM_CONFIG_1, REG_MODEM_CONFIG_2,
    // REG_SYMB_TIMEOUT_LSB,
    REG_PREAMBLE_MSB, REG_PREAMBLE_LSB, REG_PAYLOAD_LENGTH,
    // REG_MAX_PAYLOAD_LENGTH, REG_HOP_PERIOD, REG_FIFI_RX_BYTE_ADDR,
    REG_MODEM_CONFIG_3,
    // REG_PPM_CORRECTION, REG_FEI_MSB, REG_FEI_MID, REG_FEI_LSB, REG_RSSI_WIDEBAND,
    REG_DETECTION_OPTIMIZE,
    // REG_INVERT_IQ,
    REG_DETECTION_THRESHOLD, REG_SYNC_WORD, REG_DIO_MAPPING_1, REG_DIO_MAPPING_2, REG_VERSION,
    // REG_PLL_HOP, REG_TCXO,
    REG_PA_DAC//,
    // REG_FORMER_TEMP, REG_BITRATE_FRAC, REG_AGC_REF, REG_AGC_THRESH_1, REG_AGC_THRESH_2, REG_AGC_THRESH_3, REG_PLL_HF
};

const Address_field ALL_FIELD[38] = { // ALL_FIELD[86]
    Fifo, LongRangeMode,
    // AccessSharedReg,
    LowFrequencyModeOn, Mode, Frf, PaSelect, MaxPower, OutputPower,
    // PaRamp, OcpOn, OcpTrim, LnaGain, LnaBoostLf,
    LnaBoostHf, FifoAddrPtr, FifoTxBaseAddr, FifoRxBaseAddr, FifoRxCurrentAddr,
    // RxTimeoutMask, RxDoneMask, PayloadCrcErrorMask, ValidHeaderMask, TxDoneMask, CadDoneMask, FhssChangeChannelMask, CadDetectedMask,
    RxTimeout, RxDone, PayloadCrcError, ValidHeader, TxDone, CadDone, FhssChangeChannel, CadDetected, FifoRxBytesNb,
    // ValidHeaderCnt, ValidPacketCnt, RxCodingRate, ModemStatus,
    PacketSnr, PacketRssi,
    // Rssi, PllTimeout, CrcOnPayload, FhssPresentChannel,
    Bw,
    // CodingRate, ImplicitHeaderModeOn,
    SpreadingFactor,
    // TxContinuousMode,
    RxPayloadCrcOn,
    // SymbTimeout,
    PreambleLength, PayloadLength,
    // PayloadMaxLength, FreqHoppingPeriod, FifoRxByteAddrPtr,
    LowDataRateOptimize, AgcAutoOn,
    // PpmCorrection, FreqError, RssiWideband,
    DetectionOptimize,
    // InvertIQ,
    DetectionThreshold, SyncWord, Dio0Mapping, Dio1Mapping,
    // Dio2Mapping, Dio3Mapping, Dio4Mapping, Dio5Mapping,
    Version,
    // FastHopOn, TcxoInputOn,
    PaDac
    // FormerTemp, BitRateFrac, AgcReferenceLevel, AgcStep1, AgcStep2, AgcStep3, AgcStep4, AgcStep5, PllBandwidth
};












#else

//   ----- ----- ----- - - - - - - - - - ----- ----- -----
// ----- ----- ----- Список регистров LoRa ----- ----- -----
//   ----- ----- ----- - - - - - - - - - ----- ----- -----
const uint8_t
REG_FIFO = 0x00,
// --- Общие регистры настроек ---
REG_OP_MODE = 0x01,
REG_FRF_MSB = 0x06,
REG_FRF_MID = 0x07,
REG_FRF_LSB = 0x08,
// --- Регистры для блоков RF ---
REG_PA_CONFIG = 0x09,
REG_PA_RAMP = 0x0A,
REG_LR_OCP = 0X0B,
REG_LNA = 0x0C,
// --- Страница регистров LoRa ---
REG_FIFO_ADDR_PTR = 0x0D,
REG_FIFO_TX_BASE_ADDR = 0x0E,
REG_FIFO_RX_BASE_ADDR = 0x0F,
REG_FIFO_RX_CURRENT_ADDR = 0x10,
REG_IRQ_FLAGS_MASK = 0x11,
REG_IRQ_FLAGS = 0x12,
REG_RX_NB_BYTES = 0x13,
REG_RX_HEADER_CNT_VALUE_MSB = 0x14,
REG_RX_HEADER_CNT_VALUE_LSB = 0x15,
REG_RX_PACKET_CNT_VALUE_MSB = 0x16,
REG_RX_PACKET_CNT_VALUE_LSB = 0x17,
REG_MODEM_STAT = 0x18,
REG_PKT_SNR_VALUE = 0x19,
REG_PKT_RSSI_VALUE = 0x1A,
REG_RSSI_VALUE = 0x1B,
REG_HOP_CHANNEL = 0x1C,
REG_MODEM_CONFIG_1 = 0x1D,
REG_MODEM_CONFIG_2 = 0x1E,
REG_SYMB_TIMEOUT_LSB = 0x1F,
REG_PREAMBLE_MSB = 0x20,
REG_PREAMBLE_LSB = 0x21,
REG_PAYLOAD_LENGTH = 0x22,
REG_MAX_PAYLOAD_LENGTH = 0x23,
REG_HOP_PERIOD = 0x24,
REG_FIFI_RX_BYTE_ADDR = 0x25,
REG_MODEM_CONFIG_3 = 0x26,
REG_PPM_CORRECTION = 0x27,
REG_FEI_MSB = 0x28,
REG_FEI_MID = 0x29,
REG_FEI_LSB = 0x2A,
REG_RSSI_WIDEBAND = 0x2C,
REG_DETECTION_OPTIMIZE = 0x31,
REG_INVERT_IQ = 0x33,
REG_DETECTION_THRESHOLD = 0x37,
REG_SYNC_WORD = 0x39,
// --- Регистры управления IO ---
REG_DIO_MAPPING_1 = 0x40,
REG_DIO_MAPPING_2 = 0x41,
// --- Регистр версий ---
REG_VERSION = 0x42,
// --- Дополнительные регистры ---
REG_PLL_HOP = 0x44,
REG_TCXO = 0x4B,
REG_PA_DAC = 0x4D,
REG_FORMER_TEMP = 0x5B,
REG_BITRATE_FRAC = 0x5D,
REG_AGC_REF = 0x61,
REG_AGC_THRESH_1 = 0x62,
REG_AGC_THRESH_2 = 0x63,
REG_AGC_THRESH_3 = 0x64,
REG_PLL_HF = 0x70;


//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
Register
// --- --- --- Регистр очереди --- --- ---
// - REG_FIFO -
reg_fifo[1] = {Register(REG_FIFO, 8, 0)},

// --- --- --- Общие регистры настроек --- --- ---
// - REG_OP_MODE -
reg_long_range_mode[1] = {Register(REG_OP_MODE, 1, 7)},
reg_access_shared_reg[1] = {Register(REG_OP_MODE, 1, 6)},
reg_low_frequency_mode_on[1] = {Register(REG_OP_MODE, 1, 3)},
reg_mode[1] = {Register(REG_OP_MODE, 3, 0)},

// - REG_FRF_* -
reg_frf[3] = {Register(REG_FRF_LSB, 8, 0),
              Register(REG_FRF_MID, 8, 0),
              Register(REG_FRF_MSB, 8, 0)},

// --- --- --- Регистры для блоков RF --- --- ---
// - REG_PA_CONFIG -
reg_pa_select[1] = {Register(REG_PA_CONFIG, 1, 7)},
reg_max_power[1] = {Register(REG_PA_CONFIG, 3, 4)},
reg_output_power[1] = {Register(REG_PA_CONFIG, 4, 0)},
// - REG_PA_RAMP -
reg_pa_ramp[1] = {Register(REG_PA_RAMP, 4, 0)},
// - REG_LR_OCP -
reg_ocp_on[1] = {Register(REG_LR_OCP, 1, 5)},
reg_ocp_trim[1] = {Register(REG_LR_OCP, 5, 0)},
// - REG_LNA -
reg_lna_gain[1] = {Register(REG_LNA, 3, 5)},
reg_lna_boost_lf[1] = {Register(REG_LNA, 2, 3)},
reg_lna_boost_hf[1] = {Register(REG_LNA, 2, 0)},


// --- --- --- Страница регистров LoRa --- --- ---
// - REG_FIFO_ADDR_PTR -
reg_fifo_addr_ptr[1] = {Register(REG_FIFO_ADDR_PTR, 8, 0)},
// - REG_FIFO_TX_BASE_ADDR -
reg_fifo_tx_base_addr[1] = {Register(REG_FIFO_TX_BASE_ADDR, 8, 0)},
// - REG_FIFO_RX_BASE_ADDR -
reg_fifo_rx_base_addr[1] = {Register(REG_FIFO_RX_BASE_ADDR, 8, 0)},
// - REG_FIFO_RX_CURRENT_ADDR -
reg_fifo_rx_current_addr[1] = {Register(REG_FIFO_RX_CURRENT_ADDR, 8, 0)},
// - REG_IRQ_FLAGS_MASK -
reg_rx_timeout_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 7)},
reg_rx_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 6)},
reg_payload_crc_error_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 5)},
reg_valid_header_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 4)},
reg_tx_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 3)},
reg_cad_done_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 2)},
reg_fhss_change_channel_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 1)},
reg_cad_detected_mask[1] = {Register(REG_IRQ_FLAGS_MASK, 1, 0)},
// - REG_IRQ_FLAGS -
reg_rx_timeout[1] = {Register(REG_IRQ_FLAGS, 1, 7)},
reg_rx_done[1] = {Register(REG_IRQ_FLAGS, 1, 6)},
reg_payload_crc_error[1] = {Register(REG_IRQ_FLAGS, 1, 5)},
reg_valid_header[1] = {Register(REG_IRQ_FLAGS, 1, 4)},
reg_tx_done[1] = {Register(REG_IRQ_FLAGS, 1, 3)},
reg_cad_done[1] = {Register(REG_IRQ_FLAGS, 1, 2)},
reg_fhss_change_channel[1] = {Register(REG_IRQ_FLAGS, 1, 1)},
reg_cad_detected[1] = {Register(REG_IRQ_FLAGS, 1, 0)},
// - REG_RX_NB_BYTES -
reg_fifo_rx_bytes_nb[1] = {Register(REG_RX_NB_BYTES, 8, 0)},
// - REG_RX_HEADER_CNT_VALUE_* -
reg_valid_header_cnt[2] = {Register(REG_RX_HEADER_CNT_VALUE_LSB, 8, 0),
                           Register(REG_RX_HEADER_CNT_VALUE_MSB, 8, 0)},
// - REG_RX_PACKET_CNT_VALUE_* -
reg_valid_packet_cnt[2] = {Register(REG_RX_PACKET_CNT_VALUE_LSB, 8, 0),
                           Register(REG_RX_PACKET_CNT_VALUE_MSB, 8, 0)},
// - REG_MODEM_STAT -
reg_rx_coding_rate[1] = {Register(REG_MODEM_STAT, 3, 5)},
reg_modem_status[1] = {Register(REG_MODEM_STAT, 5, 0)},
// - REG_PKT_SNR_VALUE -
reg_packet_snr[1] = {Register(REG_PKT_SNR_VALUE, 8, 0)},
// - REG_PKT_RSSI_VALUE -
reg_packet_rssi[1] = {Register(REG_PKT_RSSI_VALUE, 8, 0)},
// - REG_RSSI_VALUE -
reg_rssi[1] = {Register(REG_RSSI_VALUE, 8, 0)},
// - REG_HOP_CHANNEL -
reg_pll_timeout[1] = {Register(REG_HOP_CHANNEL, 1, 7)},
reg_crc_on_payload[1] = {Register(REG_HOP_CHANNEL, 1, 6)},
reg_fhss_present_channel[1] = {Register(REG_HOP_CHANNEL, 6, 0)},
// - REG_MODEM_CONFIG_1 -
reg_bw[1] = {Register(REG_MODEM_CONFIG_1, 4, 4)},
reg_coding_rate[1] = {Register(REG_MODEM_CONFIG_1, 3, 1)},
reg_implicit_header_mode_on[1] = {Register(REG_MODEM_CONFIG_1, 1, 0)},
// - REG_MODEM_CONFIG_2 -
reg_spreading_factor[1] = {Register(REG_MODEM_CONFIG_2, 4, 4)},
reg_tx_continuous_mode[1] = {Register(REG_MODEM_CONFIG_2, 1, 3)},
reg_rx_payload_crc_on[1] = {Register(REG_MODEM_CONFIG_2, 1, 2)},
// - REG_SYMB_TIMEOUT_* -
reg_symb_timeout[2] = {Register(REG_SYMB_TIMEOUT_LSB, 8, 0),
                       Register(REG_MODEM_CONFIG_2, 2, 0)},
// - REG_PREAMBLE_* -
reg_preamble_length[2] = {Register(REG_PREAMBLE_LSB, 8, 0),
                          Register(REG_PREAMBLE_MSB, 8, 0)},
// - REG_PAYLOAD_LENGTH -
reg_payload_length[1] = {Register(REG_PAYLOAD_LENGTH, 8, 0)},
// - REG_MAX_PAYLOAD_LENGTH -
reg_payload_max_length[1] = {Register(REG_MAX_PAYLOAD_LENGTH, 8, 0)},
// - REG_HOP_PERIOD -
reg_freq_hopping_period[1] = {Register(REG_HOP_PERIOD, 8, 0)},
// - REG_FIFI_RX_BYTE_ADDR -
reg_fifo_rx_byte_addr_ptr[1] = {Register(REG_FIFI_RX_BYTE_ADDR, 8, 0)},
// - REG_MODEM_CONFIG_3 -
reg_low_data_rate_optimize[1] = {Register(REG_MODEM_CONFIG_3, 1, 3)},
reg_agc_auto_on[1] = {Register(REG_MODEM_CONFIG_3, 1, 2)},
// - REG_PPM_CORRECTION -
reg_ppm_correction[1] = {Register(REG_PPM_CORRECTION, 8, 0)},
// - REG_FEI_* -
reg_freq_error[3] = {Register(REG_FEI_LSB, 8, 0),
                     Register(REG_FEI_MID, 8, 0),
                     Register(REG_FEI_MSB, 4, 0)},
// - REG_RSSI_WIDEBAND -
reg_rssi_wideband[1] = {Register(REG_RSSI_WIDEBAND, 8, 0)},
// - REG_DETECTION_OPTIMIZE -
reg_detection_optimize[1] = {Register(REG_DETECTION_OPTIMIZE, 3, 0)},
// - REG_INVERT_IQ -
reg_invert_iq[1] = {Register(REG_INVERT_IQ, 1, 6)},
// - REG_DETECTION_THRESHOLD -
reg_detection_threshold[1] = {Register(REG_DETECTION_THRESHOLD, 8, 0)},
// - REG_SYNC_WORD -
reg_sync_word[1] = {Register(REG_SYNC_WORD, 8, 0)},  // , reserved_value=[0x34])

// --- --- --- Регистры управления IO --- --- ---
// - REG_DIO_MAPPING_1 -
reg_dio0_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 6)},
reg_dio1_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 4)},
reg_dio2_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 2)},
reg_dio3_mapping[1] = {Register(REG_DIO_MAPPING_1, 2, 0)},
// - REG_DIO_MAPPING_2 -
reg_dio4_mapping[1] = {Register(REG_DIO_MAPPING_2, 2, 6)},
reg_dio5_mapping[1] = {Register(REG_DIO_MAPPING_2, 2, 4)},
reg_map_preamble_detect[1] = {Register(REG_DIO_MAPPING_2, 1, 0)},

// --- --- --- Регистр версий --- --- ---
// - REG_VERSION -
reg_version[1] = {Register(REG_VERSION, 8, 0)},

// --- --- --- Дополнительные регистры --- --- ---
// - REG_PLL_HOP -
reg_fast_hop_on[1] = {Register(REG_PLL_HOP, 1, 7)},
// - REG_TCXO -
reg_tcxo_input_on[1] = {Register(REG_TCXO, 1, 4)},
// - REG_PA_DAC -
reg_pa_dac[1] = {Register(REG_PA_DAC, 3, 0)},
// - REG_FORMER_TEMP -
reg_former_temp[1] = {Register(REG_FORMER_TEMP, 8, 0)},
// - REG_BITRATE_FRAC -
reg_bit_rate_frac[1] = {Register(REG_BITRATE_FRAC, 4, 0)},
// - REG_AGC_REF -
reg_agc_reference_level[1] = {Register(REG_AGC_REF, 6, 0)},
// - REG_AGC_THRESH_1 -
reg_agc_step1[1] = {Register(REG_AGC_THRESH_1, 5, 0)},
// - REG_AGC_THRESH_2 -
reg_agc_step2[1] = {Register(REG_AGC_THRESH_2, 4, 4)},
reg_agc_step3[1] = {Register(REG_AGC_THRESH_2, 4, 0)},
// - REG_AGC_THRESH_3 -
reg_agc_step4[1] = {Register(REG_AGC_THRESH_3, 4, 4)},
reg_agc_step5[1] = {Register(REG_AGC_THRESH_3, 4, 0)},
// - REG_PLL_HF -
reg_pll_bandwidth[1] = {Register(REG_PLL_HF, 2, 6)};

//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
// ----- ----- -----  зарезервированные значения  ----- ----- -----
//   ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
uint32_t
// --- --- --- Регистры для блоков RF --- --- ---
// - REG_LNA -
reserv_lna_boost_hf[2] = {0x01, 0x02},
// --- --- --- Страница регистров LoRa --- --- ---
// - REG_DETECTION_OPTIMIZE -
reserv_detection_optimize[1] = {0x04},
// - REG_DETECTION_THRESHOLD -
reserv_detection_threshold[1] = {0x0B},
// - REG_SYNC_WORD -
// reg_sync_word[1] = {0x34},
// --- --- --- Дополнительные регистры --- --- ---
// - REG_PA_DAC -
reserv_pa_dac[2] = {0x05, 0x06};


// (Register *registers, uint8_t reg_count=1, bool reg_revers=false, char mode='w', uint32_t min_value=0,
//                   uint32_t (-1), uint32_t *reserved_value=nullptr, uint32_t reserv_count=0);


//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Поля и регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
Address_field
// --- --- --- Регистр очереди --- --- ---
// - REG_FIFO -
Fifo(reg_fifo, 1, false, 'w', 0x00, 0xFF),

// --- --- --- Общие регистры настроек --- --- ---
// - REG_OP_MODE -
LongRangeMode(reg_long_range_mode, 1, false, 'w', 0x00, 0x01),
AccessSharedReg(reg_access_shared_reg, 1, false, 'w', 0x00, 0x01),
LowFrequencyModeOn(reg_low_frequency_mode_on, 1, false, 'w', 0x00, 0x01),
Mode(reg_mode, 1, false, 'w', 0x00, 0x07),
// - REG_FRF_* -
Frf(reg_frf, 3, false, 'w', 0x000000, 0xFFFFFF),

// --- --- --- Регистры для блоков RF --- --- ---
// - REG_PA_CONFIG -
PaSelect(reg_pa_select, 1, false, 'w', 0x00, 0x01),
MaxPower(reg_max_power, 1, false, 'w', 0x00, 0x07),
OutputPower(reg_output_power, 1, false, 'w', 0x00, 0x0F),
// - REG_PA_RAMP -
PaRamp(reg_pa_ramp, 1, false, 'w', 0x00, 0x0F),
// - REG_LR_OCP -
OcpOn(reg_ocp_on, 1, false, 'w', 0x00, 0x01),
OcpTrim(reg_ocp_trim, 1, false, 'w', 0x00, 0x1F),
// - REG_LNA -
LnaGain(reg_lna_gain, 1, false, 'w', 0x01, 0x06),
LnaBoostLf(reg_lna_boost_lf, 1, false, 'w', 0x00, 0x00),
LnaBoostHf(reg_lna_boost_hf, 1, false, 'w', 0x00, 0x03, reserv_lna_boost_hf, 2),

// --- --- --- Страница регистров LoRa --- --- ---
// - REG_FIFO_ADDR_PTR -
FifoAddrPtr(reg_fifo_addr_ptr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_TX_BASE_ADDR -
FifoTxBaseAddr(reg_fifo_tx_base_addr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_RX_BASE_ADDR -
FifoRxBaseAddr(reg_fifo_rx_base_addr, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFO_RX_CURRENT_ADDR -
FifoRxCurrentAddr(reg_fifo_rx_current_addr, 1, false, 'r', 0x00, 0xFF),
// - REG_IRQ_FLAGS_MASK -
RxTimeoutMask(reg_rx_timeout_mask, 1, false, 'w', 0x00, 0x01),
RxDoneMask(reg_rx_done_mask, 1, false, 'w', 0x00, 0x01),
PayloadCrcErrorMask(reg_payload_crc_error_mask, 1, false, 'w', 0x00, 0x01),
ValidHeaderMask(reg_valid_header_mask, 1, false, 'w', 0x00, 0x01),
TxDoneMask(reg_tx_done_mask, 1, false, 'w', 0x00, 0x01),
CadDoneMask(reg_cad_done_mask, 1, false, 'w', 0x00, 0x01),
FhssChangeChannelMask(reg_fhss_change_channel_mask, 1, false, 'w', 0x00, 0x01),
CadDetectedMask(reg_cad_detected_mask, 1, false, 'w', 0x00, 0x01),
// - REG_IRQ_FLAGS -
RxTimeout(reg_rx_timeout, 1, false, 'c', 0x00, 0x01),
RxDone(reg_rx_done, 1, false, 'c', 0x00, 0x01),
PayloadCrcError(reg_payload_crc_error, 1, false, 'c', 0x00, 0x01),
ValidHeader(reg_valid_header, 1, false, 'c', 0x00, 0x01),
TxDone(reg_tx_done, 1, false, 'c', 0x00, 0x01),
CadDone(reg_cad_done, 1, false, 'c', 0x00, 0x01),
FhssChangeChannel(reg_fhss_change_channel, 1, false, 'c', 0x00, 0x01),
CadDetected(reg_cad_detected, 1, false, 'c', 0x00, 0x01),
// - REG_RX_NB_BYTES -
FifoRxBytesNb(reg_fifo_rx_bytes_nb, 1, false, 'r', 0x00, 0xFF),
// - REG_RX_HEADER_CNT_VALUE_* -
ValidHeaderCnt(reg_valid_header_cnt, 2, false, 'r', 0x0000, 0xFFFF),
// - REG_RX_PACKET_CNT_VALUE_* -
ValidPacketCnt(reg_valid_packet_cnt, 2, false, 'c', 0x0000, 0xFFFF),
// - REG_MODEM_STAT -
RxCodingRate(reg_rx_coding_rate, 1, false, 'r', 0x00, 0x07),
ModemStatus(reg_modem_status, 1, false, 'r', 0x00, 0x1F),
// - REG_PKT_SNR_VALUE -
PacketSnr(reg_packet_snr, 1, false, 'r', 0x00, 0xFF),
// - REG_PKT_RSSI_VALUE -
PacketRssi(reg_packet_rssi, 1, false, 'r', 0x00, 0xFF),
// - REG_RSSI_VALUE -
Rssi(reg_rssi, 1, false, 'r', 0x00, 0xFF),
// - REG_HOP_CHANNEL -
PllTimeout(reg_pll_timeout, 1, false, 'r', 0x00, 0x01),
CrcOnPayload(reg_crc_on_payload, 1, false, 'r', 0x00, 0x01),
FhssPresentChannel(reg_fhss_present_channel, 1, false, 'r', 0x00, 0x3F),
// - REG_MODEM_CONFIG_1 -
Bw(reg_bw, 1, false, 'w', 0x00, 0x09),
CodingRate(reg_coding_rate, 1, false, 'w', 0x01, 0x04),
ImplicitHeaderModeOn(reg_implicit_header_mode_on, 1, false, 'w', 0x00, 0x01),
// - REG_MODEM_CONFIG_2 -
SpreadingFactor(reg_spreading_factor, 1, false, 'w', 0x06, 0x0C),
TxContinuousMode(reg_tx_continuous_mode, 1, false, 'w', 0x00, 0x01),
RxPayloadCrcOn(reg_rx_payload_crc_on, 1, false, 'w', 0x00, 0x01),
// - REG_SYMB_TIMEOUT_* -
SymbTimeout(reg_symb_timeout, 2, false, 'w', 0x0000, 0x03FF),
// - REG_PREAMBLE_* -
PreambleLength(reg_preamble_length, 2, false, 'w', 0x0000, 0xFFFF),
// - REG_PAYLOAD_LENGTH -
PayloadLength(reg_payload_length, 1, false, 'w', 0x01, 0xFF),
// - REG_MAX_PAYLOAD_LENGTH -
PayloadMaxLength(reg_payload_max_length, 1, false, 'w', 0x00, 0xFF),
// - REG_HOP_PERIOD -
FreqHoppingPeriod(reg_freq_hopping_period, 1, false, 'w', 0x00, 0xFF),
// - REG_FIFI_RX_BYTE_ADDR -
FifoRxByteAddrPtr(reg_fifo_rx_byte_addr_ptr, 1, false, 'r', 0x00, 0xFF),
// - REG_MODEM_CONFIG_3 -
LowDataRateOptimize(reg_low_data_rate_optimize, 1, false, 'w', 0x00, 0x01),
AgcAutoOn(reg_agc_auto_on, 1, false, 'w', 0x00, 0x01),
// - REG_PPM_CORRECTION -
PpmCorrection(reg_ppm_correction, 1, false, 'w', 0x00, 0xFF),
// - REG_FEI_* -
FreqError(reg_freq_error, 3, false, 'r', 0x000000, 0x0FFFFF),
// - REG_RSSI_WIDEBAND -
RssiWideband(reg_rssi_wideband, 1, false, 'r', 0x00, 0xFF),
// - REG_DETECTION_OPTIMIZE -
DetectionOptimize(reg_detection_optimize, 1, false, 'w', 0x03, 0x05, reserv_detection_optimize, 1),
// - REG_INVERT_IQ -
InvertIQ(reg_invert_iq, 1, false, 'w', 0x00, 0x01),
// - REG_DETECTION_THRESHOLD -
DetectionThreshold(reg_detection_threshold, 1, false, 'w', 0x0A, 0x0C, reserv_detection_threshold, 1),
// - REG_SYNC_WORD -
SyncWord(reg_sync_word, 1, false, 'w', 0x00, 0xFF),  // , reserv_sync_word, 1),

// --- --- --- Регистры управления IO --- --- ---
// - REG_DIO_MAPPING_1 -
Dio0Mapping(reg_dio0_mapping, 1, false, 'w', 0x00, 0x03),
Dio1Mapping(reg_dio1_mapping, 1, false, 'w', 0x00, 0x03),
Dio2Mapping(reg_dio2_mapping, 1, false, 'w', 0x00, 0x03),
Dio3Mapping(reg_dio3_mapping, 1, false, 'w', 0x00, 0x03),
// - REG_DIO_MAPPING_2 -
Dio4Mapping(reg_dio4_mapping, 1, false, 'w', 0x00, 0x03),
Dio5Mapping(reg_dio5_mapping, 1, false, 'w', 0x00, 0x03),
MapPreambleDetect(reg_map_preamble_detect, 1, false, 'w', 0x00, 0x01),

// --- --- --- Регистр версий --- --- ---
// - REG_VERSION -
Version(reg_version, 1, false, 'r', 0x00, 0xFF),

// --- --- --- Дополнительные регистры --- --- ---
// - REG_PLL_HOP -
FastHopOn(reg_fast_hop_on, 1, false, 'w', 0x00, 0x01),
// - REG_TCXO -
TcxoInputOn(reg_tcxo_input_on, 1, false, 'w', 0x00, 0x01),
// - REG_PA_DAC -
PaDac(reg_pa_dac, 1, false, 'w', 0x04, 0x07, reserv_pa_dac, 2),
// - REG_FORMER_TEMP -
FormerTemp(reg_former_temp, 1, false, 'r', 0x00, 0xFF),
// - REG_BITRATE_FRAC -
BitRateFrac(reg_bit_rate_frac, 1, false, 'w', 0x00, 0x0F),
// - REG_AGC_REF -
AgcReferenceLevel(reg_agc_reference_level, 1, false, 'w', 0x00, 0x3F),
// - REG_AGC_THRESH_1 -
AgcStep1(reg_agc_step1, 1, false, 'w', 0x00, 0x1F),
// - REG_AGC_THRESH_2 -
AgcStep2(reg_agc_step2, 1, false, 'w', 0x00, 0x0F),
AgcStep3(reg_agc_step3, 1, false, 'w', 0x00, 0x0F),
// - REG_AGC_THRESH_3 -
AgcStep4(reg_agc_step4, 1, false, 'w', 0x00, 0x0F),
AgcStep5(reg_agc_step5, 1, false, 'w', 0x00, 0x0F),
// - REG_PLL_HF -
PllBandwidth(reg_pll_bandwidth, 1, false, 'w', 0x00, 0x03);



// Данные о всех регистрах/полях

const int AMT_ALL_REGISTER = 57, AMT_ALL_FIELD = 86;

const uint8_t ALL_REGISTER[57] = {
    REG_FIFO, REG_OP_MODE, REG_FRF_MSB, REG_FRF_MID, REG_FRF_LSB, REG_PA_CONFIG,
    REG_PA_RAMP, REG_LR_OCP, REG_LNA, REG_FIFO_ADDR_PTR, REG_FIFO_TX_BASE_ADDR,
    REG_FIFO_RX_BASE_ADDR, REG_FIFO_RX_CURRENT_ADDR, REG_IRQ_FLAGS_MASK,
    REG_IRQ_FLAGS, REG_RX_NB_BYTES, REG_RX_HEADER_CNT_VALUE_MSB,
    REG_RX_HEADER_CNT_VALUE_LSB, REG_RX_PACKET_CNT_VALUE_MSB,
    REG_RX_PACKET_CNT_VALUE_LSB, REG_MODEM_STAT, REG_PKT_SNR_VALUE,
    REG_PKT_RSSI_VALUE, REG_RSSI_VALUE, REG_HOP_CHANNEL, REG_MODEM_CONFIG_1,
    REG_MODEM_CONFIG_2, REG_SYMB_TIMEOUT_LSB, REG_PREAMBLE_MSB, REG_PREAMBLE_LSB,
    REG_PAYLOAD_LENGTH, REG_MAX_PAYLOAD_LENGTH, REG_HOP_PERIOD,
    REG_FIFI_RX_BYTE_ADDR, REG_MODEM_CONFIG_3, REG_PPM_CORRECTION, REG_FEI_MSB,
    REG_FEI_MID, REG_FEI_LSB, REG_RSSI_WIDEBAND, REG_DETECTION_OPTIMIZE,
    REG_INVERT_IQ, REG_DETECTION_THRESHOLD, REG_SYNC_WORD, REG_DIO_MAPPING_1,
    REG_DIO_MAPPING_2, REG_VERSION, REG_PLL_HOP, REG_TCXO, REG_PA_DAC,
    REG_FORMER_TEMP, REG_BITRATE_FRAC, REG_AGC_REF, REG_AGC_THRESH_1,
    REG_AGC_THRESH_2, REG_AGC_THRESH_3, REG_PLL_HF
};

Address_field ALL_FIELD[86] = {
    Fifo, LongRangeMode, AccessSharedReg, LowFrequencyModeOn, Mode, Frf, PaSelect,
    MaxPower, OutputPower, PaRamp, OcpOn, OcpTrim, LnaGain, LnaBoostLf, LnaBoostHf,
    FifoAddrPtr, FifoTxBaseAddr, FifoRxBaseAddr, FifoRxCurrentAddr, RxTimeoutMask,
    RxDoneMask, PayloadCrcErrorMask, ValidHeaderMask, TxDoneMask, CadDoneMask,
    FhssChangeChannelMask, CadDetectedMask, RxTimeout, RxDone, PayloadCrcError,
    ValidHeader, TxDone, CadDone, FhssChangeChannel, CadDetected, FifoRxBytesNb,
    ValidHeaderCnt, ValidPacketCnt, RxCodingRate, ModemStatus, PacketSnr, PacketRssi,
    Rssi, PllTimeout, CrcOnPayload, FhssPresentChannel, Bw, CodingRate,
    ImplicitHeaderModeOn, SpreadingFactor, TxContinuousMode, RxPayloadCrcOn,
    SymbTimeout, PreambleLength, PayloadLength, PayloadMaxLength, FreqHoppingPeriod,
    FifoRxByteAddrPtr, LowDataRateOptimize, AgcAutoOn, PpmCorrection, FreqError,
    RssiWideband, DetectionOptimize, InvertIQ, DetectionThreshold, SyncWord,
    Dio0Mapping, Dio1Mapping, Dio2Mapping, Dio3Mapping, Dio4Mapping, Dio5Mapping,
    Version, FastHopOn, TcxoInputOn, PaDac, FormerTemp, BitRateFrac, AgcReferenceLevel,
    AgcStep1, AgcStep2, AgcStep3, AgcStep4, AgcStep5, PllBandwidth
};



#endif
