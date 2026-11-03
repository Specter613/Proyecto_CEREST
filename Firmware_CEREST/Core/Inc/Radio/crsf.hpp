#ifndef INC_RADIO_CRSF_HPP_
#define INC_RADIO_CRSF_HPP_

#include <cstdint>
#include <cstring>
#include "stm32f4xx_hal.h"
#include "Proceso/uart_dispatch.hpp"

#define CRSF_MAX_CHANNELS 16
#define CRSF_FRAME_SIZE_MAX 64

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14

typedef enum {
    CRSF_SIGNAL_LOST = 0,
    CRSF_SIGNAL_OK = 1
} CRSF_Status_t;

typedef struct {
    uint16_t ch[CRSF_MAX_CHANNELS];
    CRSF_Status_t status;
    uint8_t activeChannels;   // <-- nuevo: cuántos canales están reportando datos válidos

    // --- Campos de Telemetría de Enlace ---
    int16_t rssi1;       // RSSI Antena 1 en dBm (ej. -85)
    int16_t rssi2;       // RSSI Antena 2 en dBm
    uint8_t linkQuality; // Link Quality (0 a 100%)
    int8_t  snr;         // Relación Señal/Ruido en dB
} CRSF_Radio;

class CRSF
{
public:
    explicit CRSF(UART_HandleTypeDef *huart) : huart_(huart) {}

    void Init(CRSF_Radio *device);
    void Process();
    uint8_t NewFrameAvailable();

    const CRSF_Radio* GetDevice() const { return radio_; }

private:
    static constexpr uint8_t kSizeBuff = 8;

    static void RxFullHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxHalfHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxErrorHandlerTrampoline(UART_HandleTypeDef *huart);

    void RxHalfHandler();
    void RxFullHandler();
    void RxErrorHandler();

    void ParseByte(uint8_t byte);
    void DecodeFrame(CRSF_Radio *device);
    uint8_t Crc8(uint8_t *data, uint8_t length);

    // Cuenta cuántos canales reportan un valor distinto del "reposo"
    // (992, centro de la escala 11-bit de CRSF) — sirve como indicador
    // simple de cuántos canales el transmisor está de verdad usando.
    void UpdateActiveChannelCount(CRSF_Radio *device);

    UART_HandleTypeDef *huart_;
    CRSF_Radio *radio_ = nullptr;

    uint8_t frame_[CRSF_FRAME_SIZE_MAX] = {};
    uint8_t frameIndex_ = 0;
    uint8_t frameLength_ = 0;
    uint8_t crsfData_[CRSF_FRAME_SIZE_MAX] = {};
    uint8_t rxBufDMA_[kSizeBuff] = {};
    volatile uint8_t newFrame_ = 0;

    static CRSF *activeInstance;
};



#endif /* INC_RADIO_CRSF_HPP_ */