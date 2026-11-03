/*
 * gnss.cpp
 *
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */

#include "Sensores/gnss.hpp"
#include <cstring>

extern "C" void Error_Handler(void);

static const uint8_t kConfigUBX[]   = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBA,0x52};
static const uint8_t kSetRateTo5hz[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};
static const uint8_t kSetGNSS[]     = {0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x20,0x04,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,0xDF,0xFB};

static const uint8_t kGetDeviceID[]      = {0xB5,0x62,0x27,0x03,0x00,0x00,0x2A,0xA5};
static const uint8_t kGetNavigatorData[] = {0xB5,0x62,0x01,0x21,0x00,0x00,0x22,0x67};
static const uint8_t kGetPOSLLH[]        = {0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0x0A};
static const uint8_t kGetPVT[]           = {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19};
static const uint8_t kGetNAVSAT[]        = {0xB5,0x62,0x01,0x35,0x00,0x00,0x36,0xA3};

static const uint8_t kSetStationaryMode[] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x02,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80};
static const uint8_t kSetPortableMode[]   = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x00,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x3C};

Gnss::Gnss(UART_HandleTypeDef *huart) : huart_(huart) {}

void Gnss::Init()
{
    data_ = GnssData{};
    LoadConfig();
    HAL_Delay(300);

    HAL_UART_Abort_IT(huart_);
    HAL_UART_DeInit(huart_);
    huart_->Init.BaudRate = 115200;
    if(HAL_UART_Init(huart_) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(1000);
}

void Gnss::LoadConfig()
{
    HAL_UART_Transmit_DMA(huart_, kConfigUBX, sizeof(kConfigUBX));
    HAL_Delay(300);
    HAL_UART_Transmit_DMA(huart_, kSetRateTo5hz, sizeof(kSetRateTo5hz));
    HAL_Delay(300);
    HAL_UART_Transmit_DMA(huart_, kSetGNSS, sizeof(kSetGNSS));
    HAL_Delay(300);
}

void Gnss::SetMode(GnssMode mode)
{
    switch(mode)
    {
        case GnssMode::Portable:
            HAL_UART_Transmit_DMA(huart_, kSetPortableMode, sizeof(kSetPortableMode));
            break;
        case GnssMode::Stationary:
            HAL_UART_Transmit_DMA(huart_, kSetStationaryMode, sizeof(kSetStationaryMode));
            break;
        default:
            break;
    }
}

// Sin memset en ninguna de las dos — comportamiento consistente y sin
// condiciones de carrera con el abort/rearm de la UART.
void Gnss::RequestPVT()
{
    HAL_UART_AbortReceive_IT(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    lastRxStatus_ = HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
    lastTxStatus_ = HAL_UART_Transmit_DMA(huart_, kGetPVT, sizeof(kGetPVT));
}

void Gnss::RequestNAVSAT()
{
    HAL_UART_AbortReceive_IT(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
    HAL_UART_Transmit_DMA(huart_, kGetNAVSAT, sizeof(kGetNAVSAT));
}

void Gnss::RequestUniqID()
{
    HAL_UART_AbortReceive_IT(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
    HAL_UART_Transmit_DMA(huart_, kGetDeviceID, sizeof(kGetDeviceID));
}

void Gnss::RequestNavigatorData()
{
    HAL_UART_AbortReceive_IT(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
    HAL_UART_Transmit_DMA(huart_, kGetNavigatorData, sizeof(kGetNavigatorData));
}

void Gnss::RequestPOSLLH()
{
    HAL_UART_AbortReceive_IT(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
    HAL_UART_Transmit_DMA(huart_, kGetPOSLLH, sizeof(kGetPOSLLH));
}

void Gnss::Update()
{
    ParseBuffer();

    if(requestNavSat_)
        RequestNAVSAT();
    else
        RequestPVT();
    requestNavSat_ = !requestNavSat_;
}

void Gnss::ComputeChecksum(const uint8_t *buffer, uint16_t start, uint16_t len,
                            uint8_t &ckA, uint8_t &ckB)
{
    ckA = 0;
    ckB = 0;
    for(uint16_t i = start; i < start + len; i++)
    {
        ckA += buffer[i];
        ckB += ckA;
    }
}

void Gnss::ParseBuffer()
{
    for(int var = 0; var <= kBufferSize - 8; ++var)
    {
        if(uartBuffer_[var] == 0xB5 && uartBuffer_[var + 1] == 0x62)
        {
            uint8_t msgClass = uartBuffer_[var + 2];
            uint8_t msgId    = uartBuffer_[var + 3];
            uint16_t payloadLen = uartBuffer_[var + 4] | (uartBuffer_[var + 5] << 8);

            // Validación de límites: si el mensaje reportado no cabe
            // en el buffer, descártalo sin leer memoria fuera de rango.
            if(var + 6 + payloadLen + 1 >= kBufferSize)
            {
                continue;
            }

            uint8_t ckA, ckB;
            ComputeChecksum(uartBuffer_, var + 2, 4 + payloadLen, ckA, ckB);

            uint8_t recvCkA = uartBuffer_[var + 6 + payloadLen];
            uint8_t recvCkB = uartBuffer_[var + 6 + payloadLen + 1];

            if(ckA != recvCkA || ckB != recvCkB) continue;

            if(msgClass == 0x27 && msgId == 0x03)       ParseUniqID();
            else if(msgClass == 0x01 && msgId == 0x21)  ParseNavigatorData();
            else if(msgClass == 0x01 && msgId == 0x07)  ParsePVT();
            else if(msgClass == 0x01 && msgId == 0x02)  ParsePOSLLH();
            else if(msgClass == 0x01 && msgId == 0x35)  ParseNAVSAT();
        }
    }
}

void Gnss::ParseUniqID()
{
    memcpy(data_.uniqueID, &uartBuffer_[10], sizeof(data_.uniqueID));
}

void Gnss::ParsePVT()
{
    data_.year = static_cast<uint16_t>(uartBuffer_[10] | (uartBuffer_[11] << 8));
    data_.month = uartBuffer_[12];
    data_.day   = uartBuffer_[13];
    data_.hour  = uartBuffer_[14];
    data_.min   = uartBuffer_[15];
    data_.sec   = uartBuffer_[16];
    data_.fixType = uartBuffer_[26];
    data_.numSV   = uartBuffer_[29];

    memcpy(&data_.lon, &uartBuffer_[30], sizeof(int32_t));
    data_.fLon = static_cast<float>(data_.lon) / 10000000.0f;

    memcpy(&data_.lat, &uartBuffer_[34], sizeof(int32_t));
    data_.fLat = static_cast<float>(data_.lat) / 10000000.0f;

    memcpy(&data_.height, &uartBuffer_[38], sizeof(int32_t));
    memcpy(&data_.hMSL,   &uartBuffer_[42], sizeof(int32_t));
    memcpy(&data_.hAcc,   &uartBuffer_[46], sizeof(uint32_t));
    memcpy(&data_.vAcc,   &uartBuffer_[50], sizeof(uint32_t));
    memcpy(&data_.gSpeed, &uartBuffer_[66], sizeof(int32_t));
    memcpy(&data_.headMot,&uartBuffer_[70], sizeof(int32_t));
}

void Gnss::ParseNavigatorData()
{
    data_.year = static_cast<uint16_t>(uartBuffer_[18] | (uartBuffer_[19] << 8));
    data_.month = uartBuffer_[20];
    data_.day   = uartBuffer_[21];
    data_.hour  = uartBuffer_[22];
    data_.min   = uartBuffer_[23];
    data_.sec   = uartBuffer_[24];
}

void Gnss::ParseNAVSAT()
{
    // numSvs vive en el byte 11 (después de header/class/id/len/iTOW/version)
    data_.satCount = uartBuffer_[11];
}

void Gnss::ParsePOSLLH()
{
    memcpy(&data_.lon, &uartBuffer_[10], sizeof(int32_t));
    data_.fLon = static_cast<float>(data_.lon) / 10000000.0f;

    memcpy(&data_.lat, &uartBuffer_[14], sizeof(int32_t));
    data_.fLat = static_cast<float>(data_.lat) / 10000000.0f;

    memcpy(&data_.height, &uartBuffer_[18], sizeof(int32_t));
    memcpy(&data_.hMSL,   &uartBuffer_[22], sizeof(int32_t));
    memcpy(&data_.hAcc,   &uartBuffer_[26], sizeof(uint32_t));
    memcpy(&data_.vAcc,   &uartBuffer_[30], sizeof(uint32_t));
}



// ///Codigo experimental 

// /*
//  * gnss.cpp
//  *
//  *  Created on: 28 jul 2026
//  *      Author: specter0163
//  */

// #include "Sensores/gnss.hpp"
// #include <cstring>

// extern "C" void Error_Handler(void);

// // CFG-PRT: UART1 a 115200, entrada UBX+NMEA, salida solo UBX
// static const uint8_t kConfigUBX[]   = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBA,0x52};
// // CFG-RATE: 200 ms (5 Hz)
// static const uint8_t kSetRateTo5hz[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};
// // CFG-GNSS: GPS + SBAS + Galileo + GLONASS
// static const uint8_t kSetGNSS[]     = {0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x20,0x04,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,0xDF,0xFB};

// static const uint8_t kGetDeviceID[]      = {0xB5,0x62,0x27,0x03,0x00,0x00,0x2A,0xA5};
// static const uint8_t kGetNavigatorData[] = {0xB5,0x62,0x01,0x21,0x00,0x00,0x22,0x67};
// static const uint8_t kGetPOSLLH[]        = {0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0x0A};
// static const uint8_t kGetPVT[]           = {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19};
// static const uint8_t kGetNAVSAT[]        = {0xB5,0x62,0x01,0x35,0x00,0x00,0x36,0xA3};

// static const uint8_t kSetStationaryMode[] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x02,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80};
// static const uint8_t kSetPortableMode[]   = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x00,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x3C};

// Gnss::Gnss(UART_HandleTypeDef *huart) : huart_(huart) {}

// void Gnss::Init()
// {
//     data_ = GnssData{};
//     LoadConfig();
// }

// void Gnss::LoadConfig()
// {
//     // 1) A la velocidad actual (la de fábrica del GPS): cambiar su UART a 115200 y protocolo UBX
//     HAL_UART_Transmit(huart_, kConfigUBX, sizeof(kConfigUBX), 100);
//     HAL_Delay(100);   // el GPS aplica el cambio de velocidad

//     // 2) Ahora el micro también a 115200
//     HAL_UART_Abort_IT(huart_);
//     HAL_UART_DeInit(huart_);
//     huart_->Init.BaudRate = 115200;
//     if (HAL_UART_Init(huart_) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     HAL_Delay(100);

//     // 3) El resto de la configuración, YA a 115200 (antes se mandaba a la velocidad vieja y se perdía)
//     HAL_UART_Transmit(huart_, kSetRateTo5hz, sizeof(kSetRateTo5hz), 100);
//     HAL_Delay(100);
//     HAL_UART_Transmit(huart_, kSetGNSS, sizeof(kSetGNSS), 100);
//     HAL_Delay(500);   // el cambio de constelaciones reinicia el motor GNSS del módulo
// }

// void Gnss::SetMode(GnssMode mode)
// {
//     switch(mode)
//     {
//         case GnssMode::Portable:
//             HAL_UART_Transmit_DMA(huart_, kSetPortableMode, sizeof(kSetPortableMode));
//             break;
//         case GnssMode::Stationary:
//             HAL_UART_Transmit_DMA(huart_, kSetStationaryMode, sizeof(kSetStationaryMode));
//             break;
//         default:
//             break;
//     }
// }

// // Sin memset en ninguna de las dos — comportamiento consistente y sin
// // condiciones de carrera con el abort/rearm de la UART.
// void Gnss::RequestPVT()
// {
//     HAL_UART_AbortReceive_IT(huart_);
//     __HAL_UART_CLEAR_OREFLAG(huart_);
//     __HAL_UART_CLEAR_FEFLAG(huart_);
//     lastRxStatus_ = HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
//     lastTxStatus_ = HAL_UART_Transmit_DMA(huart_, kGetPVT, sizeof(kGetPVT));
// }

// void Gnss::RequestNAVSAT()
// {
//     HAL_UART_AbortReceive_IT(huart_);
//     __HAL_UART_CLEAR_OREFLAG(huart_);
//     __HAL_UART_CLEAR_FEFLAG(huart_);
//     HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
//     HAL_UART_Transmit_DMA(huart_, kGetNAVSAT, sizeof(kGetNAVSAT));
// }

// void Gnss::RequestUniqID()
// {
//     HAL_UART_AbortReceive_IT(huart_);
//     __HAL_UART_CLEAR_OREFLAG(huart_);
//     __HAL_UART_CLEAR_FEFLAG(huart_);
//     HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
//     HAL_UART_Transmit_DMA(huart_, kGetDeviceID, sizeof(kGetDeviceID));
// }

// void Gnss::RequestNavigatorData()
// {
//     HAL_UART_AbortReceive_IT(huart_);
//     __HAL_UART_CLEAR_OREFLAG(huart_);
//     __HAL_UART_CLEAR_FEFLAG(huart_);
//     HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
//     HAL_UART_Transmit_DMA(huart_, kGetNavigatorData, sizeof(kGetNavigatorData));
// }

// void Gnss::RequestPOSLLH()
// {
//     HAL_UART_AbortReceive_IT(huart_);
//     __HAL_UART_CLEAR_OREFLAG(huart_);
//     __HAL_UART_CLEAR_FEFLAG(huart_);
//     HAL_UARTEx_ReceiveToIdle_IT(huart_, uartBuffer_, sizeof(uartBuffer_));
//     HAL_UART_Transmit_DMA(huart_, kGetPOSLLH, sizeof(kGetPOSLLH));
// }

// void Gnss::Update()
// {
//     ParseBuffer();

//     if(requestNavSat_)
//         RequestNAVSAT();
//     else
//         RequestPVT();
//     requestNavSat_ = !requestNavSat_;
// }

// void Gnss::ComputeChecksum(const uint8_t *buffer, uint16_t start, uint16_t len,
//                             uint8_t &ckA, uint8_t &ckB)
// {
//     ckA = 0;
//     ckB = 0;
//     for(uint16_t i = start; i < start + len; i++)
//     {
//         ckA += buffer[i];
//         ckB += ckA;
//     }
// }

// void Gnss::ParseBuffer()
// {
//     for(int var = 0; var <= kBufferSize - 8; ++var)
//     {
//         if(uartBuffer_[var] == 0xB5 && uartBuffer_[var + 1] == 0x62)
//         {
//             uint8_t msgClass = uartBuffer_[var + 2];
//             uint8_t msgId    = uartBuffer_[var + 3];
//             uint16_t payloadLen = uartBuffer_[var + 4] | (uartBuffer_[var + 5] << 8);

//             // Validación de límites: si el mensaje reportado no cabe
//             // en el buffer, descártalo sin leer memoria fuera de rango.
//             if(var + 6 + payloadLen + 1 >= kBufferSize)
//             {
//                 continue;
//             }

//             uint8_t ckA, ckB;
//             ComputeChecksum(uartBuffer_, var + 2, 4 + payloadLen, ckA, ckB);

//             uint8_t recvCkA = uartBuffer_[var + 6 + payloadLen];
//             uint8_t recvCkB = uartBuffer_[var + 6 + payloadLen + 1];

//             if(ckA != recvCkA || ckB != recvCkB) continue;

//             // Las funciones Parse* leen con índices desde el inicio del buffer:
//             // si el mensaje no empieza en 0, se mueve al inicio antes de parsearlo.
//             if(var > 0)
//             {
//                 memmove(uartBuffer_, &uartBuffer_[var], 6 + payloadLen + 2);
//             }

//             if(msgClass == 0x27 && msgId == 0x03)       ParseUniqID();
//             else if(msgClass == 0x01 && msgId == 0x21)  ParseNavigatorData();
//             else if(msgClass == 0x01 && msgId == 0x07)  ParsePVT();
//             else if(msgClass == 0x01 && msgId == 0x02)  ParsePOSLLH();
//             else if(msgClass == 0x01 && msgId == 0x35)  ParseNAVSAT();

//             break;   // una respuesta por petición: ya se procesó
//         }
//     }
// }

// void Gnss::ParseUniqID()
// {
//     memcpy(data_.uniqueID, &uartBuffer_[10], sizeof(data_.uniqueID));
// }

// void Gnss::ParsePVT()
// {
//     data_.year = static_cast<uint16_t>(uartBuffer_[10] | (uartBuffer_[11] << 8));
//     data_.month = uartBuffer_[12];
//     data_.day   = uartBuffer_[13];
//     data_.hour  = uartBuffer_[14];
//     data_.min   = uartBuffer_[15];
//     data_.sec   = uartBuffer_[16];
//     data_.fixType = uartBuffer_[26];
//     data_.numSV   = uartBuffer_[29];

//     memcpy(&data_.lon, &uartBuffer_[30], sizeof(int32_t));
//     data_.fLon = static_cast<float>(data_.lon) / 10000000.0f;

//     memcpy(&data_.lat, &uartBuffer_[34], sizeof(int32_t));
//     data_.fLat = static_cast<float>(data_.lat) / 10000000.0f;

//     memcpy(&data_.height, &uartBuffer_[38], sizeof(int32_t));
//     memcpy(&data_.hMSL,   &uartBuffer_[42], sizeof(int32_t));
//     memcpy(&data_.hAcc,   &uartBuffer_[46], sizeof(uint32_t));
//     memcpy(&data_.vAcc,   &uartBuffer_[50], sizeof(uint32_t));
//     memcpy(&data_.gSpeed, &uartBuffer_[66], sizeof(int32_t));
//     memcpy(&data_.headMot,&uartBuffer_[70], sizeof(int32_t));
// }

// void Gnss::ParseNavigatorData()
// {
//     data_.year = static_cast<uint16_t>(uartBuffer_[18] | (uartBuffer_[19] << 8));
//     data_.month = uartBuffer_[20];
//     data_.day   = uartBuffer_[21];
//     data_.hour  = uartBuffer_[22];
//     data_.min   = uartBuffer_[23];
//     data_.sec   = uartBuffer_[24];
// }

// void Gnss::ParseNAVSAT()
// {
//     // numSvs vive en el byte 11 (después de header/class/id/len/iTOW/version)
//     data_.satCount = uartBuffer_[11];
// }

// void Gnss::ParsePOSLLH()
// {
//     memcpy(&data_.lon, &uartBuffer_[10], sizeof(int32_t));
//     data_.fLon = static_cast<float>(data_.lon) / 10000000.0f;

//     memcpy(&data_.lat, &uartBuffer_[14], sizeof(int32_t));
//     data_.fLat = static_cast<float>(data_.lat) / 10000000.0f;

//     memcpy(&data_.height, &uartBuffer_[18], sizeof(int32_t));
//     memcpy(&data_.hMSL,   &uartBuffer_[22], sizeof(int32_t));
//     memcpy(&data_.hAcc,   &uartBuffer_[26], sizeof(uint32_t));
//     memcpy(&data_.vAcc,   &uartBuffer_[30], sizeof(uint32_t));
// }