#include "Radio/crsf.hpp"

CRSF *CRSF::activeInstance = nullptr;

uint8_t CRSF::Crc8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;

    for(uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void CRSF::ParseByte(uint8_t byte)
{
    if(frameIndex_ == 0)
    {
        if(byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_RADIO_TRANSMITTER)
        {
            frame_[frameIndex_++] = byte;
        }
    }
    else
    {
        if(byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_RADIO_TRANSMITTER)
        {
            frameIndex_ = 0;
            frame_[frameIndex_++] = byte;
            return;
        }

        if(frameIndex_ == 1)
        {
            if(byte < 2 || byte > CRSF_FRAME_SIZE_MAX)
            {
                frameIndex_ = 0;
                return;
            }
            frameLength_ = byte;
            frame_[frameIndex_++] = byte;
        }
        else if(frameIndex_ < (frameLength_ + 2))
        {
            frame_[frameIndex_++] = byte;
        }
        else
        {
            frameIndex_ = 0;
        }
    }

    if(frameIndex_ == (frameLength_ + 2))
    {
        uint8_t crc = Crc8(&frame_[2], frameLength_ - 1);

        if(crc == frame_[frameLength_ + 1])
        {
            memcpy(crsfData_, frame_, frameLength_ + 2);
            uint8_t frameType = frame_[2];
            // Aceptamos tanto canales RC (0x16) como Estadísticas de Enlace (0x14)
            if(frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || frameType == CRSF_FRAMETYPE_LINK_STATISTICS)
            {
                radio_->status = CRSF_SIGNAL_OK;
                newFrame_ = 1;
            }
        }
        else
        {
            radio_->status = CRSF_SIGNAL_LOST;
        }
        frameIndex_ = 0;
    }
}

void CRSF::DecodeFrame(CRSF_Radio *device)
{
	uint8_t frameType = crsfData_[2];

	// 1. Tramas de Canales RC (0x16)
    if(frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        device->ch[0]  = ((crsfData_[3]  | crsfData_[4]  << 8) & 0x07FF);
        device->ch[1]  = ((crsfData_[4]  >> 3 | crsfData_[5]  << 5) & 0x07FF);
        device->ch[2]  = ((crsfData_[5]  >> 6 | crsfData_[6]  << 2 | crsfData_[7]  << 10) & 0x07FF);
        device->ch[3]  = ((crsfData_[7]  >> 1 | crsfData_[8]  << 7) & 0x07FF);
        device->ch[4]  = ((crsfData_[8]  >> 4 | crsfData_[9]  << 4) & 0x07FF);
        device->ch[5]  = ((crsfData_[9]  >> 7 | crsfData_[10] << 1 | crsfData_[11] << 9) & 0x07FF);
        device->ch[6]  = ((crsfData_[11] >> 2 | crsfData_[12] << 6) & 0x07FF);
        device->ch[7]  = ((crsfData_[12] >> 5 | crsfData_[13] << 3) & 0x07FF);

        device->ch[8]  = ((crsfData_[14] | crsfData_[15] << 8) & 0x07FF);
        device->ch[9]  = ((crsfData_[15] >> 3 | crsfData_[16] << 5) & 0x07FF);
        device->ch[10] = ((crsfData_[16] >> 6 | crsfData_[17] << 2 | crsfData_[18] << 10) & 0x07FF);
        device->ch[11] = ((crsfData_[18] >> 1 | crsfData_[19] << 7) & 0x07FF);
        device->ch[12] = ((crsfData_[19] >> 4 | crsfData_[20] << 4) & 0x07FF);
        device->ch[13] = ((crsfData_[20] >> 7 | crsfData_[21] << 1 | crsfData_[22] << 9) & 0x07FF);
        device->ch[14] = ((crsfData_[22] >> 2 | crsfData_[23] << 6) & 0x07FF);
        device->ch[15] = ((crsfData_[23] >> 5 | crsfData_[24] << 3) & 0x07FF);

        UpdateActiveChannelCount(device);
    }
    // 2. Tramas de Estadísticas de Enlace / RSSI (0x14)
    else if(frameType == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
    	// En la especificación de CRSF, los RSSI vienen como enteros positivos que representan -dBm
    	device->rssi1       = -static_cast<int16_t>(crsfData_[3]); // ej. 85 -> -85 dBm
    	device->rssi2       = -static_cast<int16_t>(crsfData_[4]);
    	device->linkQuality = crsfData_[5];                        // 0 a 100%
    	device->snr         = static_cast<int8_t>(crsfData_[6]);   // Relación señal/ruido
    }
}

void CRSF::UpdateActiveChannelCount(CRSF_Radio *device)
{
    // 992 es el valor de "centro/reposo" en la escala de 11 bits de CRSF
    // (equivalente a 1500us en PWM). Un canal que nunca se aleja de ese
    // valor probablemente no está siendo movido por el transmisor —
    // esto da una estimación práctica de cuántos canales están en uso.
    constexpr uint16_t kCenterValue = 992;
    constexpr uint16_t kTolerance = 5;

    uint8_t count = 0;
    for(uint8_t i = 0; i < CRSF_MAX_CHANNELS; i++)
    {
        uint16_t v = device->ch[i];
        if(v < (kCenterValue - kTolerance) || v > (kCenterValue + kTolerance))
        {
            count++;
        }
    }
    device->activeChannels = count;
}

void CRSF::Process()
{
    if(!newFrame_) return;

    newFrame_ = 0;
    DecodeFrame(radio_);
}

void CRSF::Init(CRSF_Radio *device)
{
    radio_ = device;
    activeInstance = this;

    for(int i = 0; i < CRSF_MAX_CHANNELS; i++)
    {
        radio_->ch[i] = 0;
    }
    radio_->status = CRSF_SIGNAL_LOST;
    radio_->activeChannels = 0;

    UartDispatch::Registrar(huart_, RxFullHandlerTrampoline);
    UartDispatch::RegistrarMitad(huart_, RxHalfHandlerTrampoline);
    UartDispatch::RegistrarError(huart_, RxErrorHandlerTrampoline);

    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}

void CRSF::RxHalfHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr)
    {
        activeInstance->RxHalfHandler();
    }
}

void CRSF::RxFullHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr)
    {
        activeInstance->RxFullHandler();
    }
}

void CRSF::RxHalfHandler()
{
    for(int i = 0; i < kSizeBuff / 2; i++)
    {
        ParseByte(rxBufDMA_[i]);
    }
}

void CRSF::RxFullHandler()
{
    for(int i = kSizeBuff / 2; i < kSizeBuff; i++)
    {
        ParseByte(rxBufDMA_[i]);
    }
}

uint8_t CRSF::NewFrameAvailable()
{
    if(newFrame_)
    {
        newFrame_ = 0;
        return 1;
    }
    return 0;
}
void CRSF::RxErrorHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr)
    {
        activeInstance->RxErrorHandler();
    }
}

void CRSF::RxErrorHandler()
{
    // Limpiamos los flags de error de la UART
    __HAL_UART_CLEAR_PEFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);

    // Abortamos y re-armamos la recepción DMA
    HAL_UART_AbortReceive(huart_);
    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}