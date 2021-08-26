#ifndef __LORA_REGISTER_DATA_H__
#define __LORA_REGISTER_DATA_H__

#if defined( ESP32 )
#include <Arduino.h>
#endif
#include <Address_field.h>

#define ECONOMICAL_VERSION
#if defined (ECONOMICAL_VERSION)

//   ----- ----- ----- - - - - - - - - - ----- ----- -----
// ----- ----- ----- Список регистров LoRa ----- ----- -----
//   ----- ----- ----- - - - - - - - - - ----- ----- -----

extern const uint8_t
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
;

//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Поля и регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
extern const Address_field
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
;


// Данные о всех регистрах/полях

extern const int AMT_ALL_REGISTER, AMT_ALL_FIELD;

extern const uint8_t ALL_REGISTER[];

extern const Address_field ALL_FIELD[];




#else

//   ----- ----- ----- - - - - - - - - - ----- ----- -----
// ----- ----- ----- Список регистров LoRa ----- ----- -----
//   ----- ----- ----- - - - - - - - - - ----- ----- -----
extern const uint8_t
REG_FIFO,
// --- Общие регистры настроек ---
REG_OP_MODE,
REG_FRF_MSB,
REG_FRF_MID,
REG_FRF_LSB,
// --- Регистры для блоков RF ---
REG_PA_CONFIG,
REG_PA_RAMP,
REG_LR_OCP,
REG_LNA,
// --- Страница регистров LoRa ---
REG_FIFO_ADDR_PTR,
REG_FIFO_TX_BASE_ADDR,
REG_FIFO_RX_BASE_ADDR,
REG_FIFO_RX_CURRENT_ADDR,
REG_IRQ_FLAGS_MASK,
REG_IRQ_FLAGS,
REG_RX_NB_BYTES,
REG_RX_HEADER_CNT_VALUE_MSB,
REG_RX_HEADER_CNT_VALUE_LSB,
REG_RX_PACKET_CNT_VALUE_MSB,
REG_RX_PACKET_CNT_VALUE_LSB,
REG_MODEM_STAT,
REG_PKT_SNR_VALUE,
REG_PKT_RSSI_VALUE,
REG_RSSI_VALUE,
REG_HOP_CHANNEL,
REG_MODEM_CONFIG_1,
REG_MODEM_CONFIG_2,
REG_SYMB_TIMEOUT_LSB,
REG_PREAMBLE_MSB,
REG_PREAMBLE_LSB,
REG_PAYLOAD_LENGTH,
REG_MAX_PAYLOAD_LENGTH,
REG_HOP_PERIOD,
REG_FIFI_RX_BYTE_ADDR,
REG_MODEM_CONFIG_3,
REG_PPM_CORRECTION,
REG_FEI_MSB,
REG_FEI_MID,
REG_FEI_LSB,
REG_RSSI_WIDEBAND,
REG_DETECTION_OPTIMIZE,
REG_INVERT_IQ,
REG_DETECTION_THRESHOLD,
REG_SYNC_WORD,
// --- Регистры управления IO ---
REG_DIO_MAPPING_1,
REG_DIO_MAPPING_2,
// --- Регистр версий ---
REG_VERSION,
// --- Дополнительные регистры ---
REG_PLL_HOP,
REG_TCXO,
REG_PA_DAC,
REG_FORMER_TEMP,
REG_BITRATE_FRAC,
REG_AGC_REF,
REG_AGC_THRESH_1,
REG_AGC_THRESH_2,
REG_AGC_THRESH_3,
REG_PLL_HF;


//   ----- ----- ----- - - - - - - ----- ----- -----
// ----- ----- ----- Поля и регистры ----- ----- -----
//   ----- ----- ----- - - - - - - ----- ----- -----
extern Address_field
// --- --- --- Регистр очереди --- --- ---
// - REG_FIFO -
Fifo,

// --- --- --- Общие регистры настроек --- --- ---
// - REG_OP_MODE -
LongRangeMode,
AccessSharedReg,
LowFrequencyModeOn,
Mode,
// - REG_FRF_* -
Frf,

// --- --- --- Регистры для блоков RF --- --- ---
// - REG_PA_CONFIG -
PaSelect,
MaxPower,
OutputPower,
// - REG_PA_RAMP -
PaRamp,
// - REG_LR_OCP -
OcpOn,
OcpTrim,
// - REG_LNA -
LnaGain,
LnaBoostLf,
LnaBoostHf,

// --- --- --- Страница регистров LoRa --- --- ---
// - REG_FIFO_ADDR_PTR -
FifoAddrPtr,
// - REG_FIFO_TX_BASE_ADDR -
FifoTxBaseAddr,
// - REG_FIFO_RX_BASE_ADDR -
FifoRxBaseAddr,
// - REG_FIFO_RX_CURRENT_ADDR -
FifoRxCurrentAddr,
// - REG_IRQ_FLAGS_MASK -
RxTimeoutMask,
RxDoneMask,
PayloadCrcErrorMask,
ValidHeaderMask,
TxDoneMask,
CadDoneMask,
FhssChangeChannelMask,
CadDetectedMask,
// - REG_IRQ_FLAGS -
RxTimeout,
RxDone,
PayloadCrcError,
ValidHeader,
TxDone,
CadDone,
FhssChangeChannel,
CadDetected,
// - REG_RX_NB_BYTES -
FifoRxBytesNb,
// - REG_RX_HEADER_CNT_VALUE_* -
ValidHeaderCnt,
// - REG_RX_PACKET_CNT_VALUE_* -
ValidPacketCnt,
// - REG_MODEM_STAT -
RxCodingRate,
ModemStatus,
// - REG_PKT_SNR_VALUE -
PacketSnr,
// - REG_PKT_RSSI_VALUE -
PacketRssi,
// - REG_RSSI_VALUE -
Rssi,
// - REG_HOP_CHANNEL -
PllTimeout,
CrcOnPayload,
FhssPresentChannel,
// - REG_MODEM_CONFIG_1 -
Bw,
CodingRate,
ImplicitHeaderModeOn,
// - REG_MODEM_CONFIG_2 -
SpreadingFactor,
TxContinuousMode,
RxPayloadCrcOn,
// - REG_SYMB_TIMEOUT_* -
SymbTimeout,
// - REG_PREAMBLE_* -
PreambleLength,
// - REG_PAYLOAD_LENGTH -
PayloadLength,
// - REG_MAX_PAYLOAD_LENGTH -
PayloadMaxLength,
// - REG_HOP_PERIOD -
FreqHoppingPeriod,
// - REG_FIFI_RX_BYTE_ADDR -
FifoRxByteAddrPtr,
// - REG_MODEM_CONFIG_3 -
LowDataRateOptimize,
AgcAutoOn,
// - REG_PPM_CORRECTION -
PpmCorrection,
// - REG_FEI_* -
FreqError,
// - REG_RSSI_WIDEBAND -
RssiWideband,
// - REG_DETECTION_OPTIMIZE -
DetectionOptimize,
// - REG_INVERT_IQ -
InvertIQ,
// - REG_DETECTION_THRESHOLD -
DetectionThreshold,
// - REG_SYNC_WORD -
SyncWord,  // , reserv_sync_word, 1),

// --- --- --- Регистры управления IO --- --- ---
// - REG_DIO_MAPPING_1 -
Dio0Mapping,
Dio1Mapping,
Dio2Mapping,
Dio3Mapping,
// - REG_DIO_MAPPING_2 -
Dio4Mapping,
Dio5Mapping,
MapPreambleDetect,

// --- --- --- Регистр версий --- --- ---
// - REG_VERSION -
Version,

// --- --- --- Дополнительные регистры --- --- ---
// - REG_PLL_HOP -
FastHopOn,
// - REG_TCXO -
TcxoInputOn,
// - REG_PA_DAC -
PaDac,
// - REG_FORMER_TEMP -
FormerTemp,
// - REG_BITRATE_FRAC -
BitRateFrac,
// - REG_AGC_REF -
AgcReferenceLevel,
// - REG_AGC_THRESH_1 -
AgcStep1,
// - REG_AGC_THRESH_2 -
AgcStep2,
AgcStep3,
// - REG_AGC_THRESH_3 -
AgcStep4,
AgcStep5,
// - REG_PLL_HF -
PllBandwidth;



// Данные о всех регистрах/полях

extern const int AMT_ALL_REGISTER, AMT_ALL_FIELD;

extern const uint8_t ALL_REGISTER[];

extern Address_field ALL_FIELD[];


#endif

#endif // __LORA_REGISTER_DATA_H__
