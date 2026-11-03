/*
 * mtf02.cpp
 *
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */

/*
 * mtf02.cpp
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */
#include "Sensores/mtf02.hpp"

MTF01 *MTF01::activeInstance = nullptr;

void MTF01::Init()
{
    activeInstance = this;

    UartDispatch::Registrar(huart_, RxFullHandlerTrampoline);
    UartDispatch::RegistrarMitad(huart_, RxHalfHandlerTrampoline);
    UartDispatch::RegistrarError(huart_, RxErrorHandlerTrampoline);

    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}

void MTF01::RxFullHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr) activeInstance->RxFullHandler();
}

void MTF01::RxHalfHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr) activeInstance->RxHalfHandler();
}

void MTF01::RxErrorHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance != nullptr) activeInstance->RxErrorHandler();
}

void MTF01::RxFullHandler()
{
    for(int i = kSizeBuff / 2; i < kSizeBuff; i++) MTF01_ProcessByte(rxBufDMA_[i]);
}

void MTF01::RxHalfHandler()
{
    for(int i = 0; i < kSizeBuff / 2; i++) MTF01_ProcessByte(rxBufDMA_[i]);
}

// --- La pieza que faltaba por completo: recuperación ante error ---
void MTF01::RxErrorHandler()
{
    __HAL_UART_CLEAR_PEFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);

    HAL_UART_AbortReceive(huart_);
    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);   // vuelve a armar — sin esto, el sensor muere para siempre ante el primer glitch
}

bool MTF01::micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t checksum = 0;

    checksum += msg->stx;
    checksum += msg->dev_id;
    checksum += msg->sys_id;
    checksum += msg->msg_id;
    checksum += msg->seq;
    checksum += msg->payload_len;

    for(uint8_t i = 0; i < msg->payload_len; i++) checksum += msg->payload[i];
    return checksum == msg->checksum;
}

bool MTF01::micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch(msg->status)
    {
        case 0:
            if(data == MICOLINK_MSG_HEAD){ 
                msg->stx = data;
                msg->payload_cnt = 0; // Reinicia el contador payload al iniciar la trama
                msg->status = 1; }
            break;
        case 1: msg->dev_id = data; msg->status++; break;
        case 2: msg->sys_id = data; msg->status++; break;
        case 3: msg->msg_id = data; msg->status++; break;
        case 4: msg->seq = data; msg->status++; break;
        case 5:
            msg->payload_len = data;
            if(msg->payload_len == 0) msg->status += 2;
            else if(msg->payload_len > MICOLINK_MAX_PAYLOAD_LEN) { msg->status = 0; msg->payload_cnt = 0; }
            else msg->status++;
            break;
        case 6:
            msg->payload[msg->payload_cnt++] = data;
            if(msg->payload_cnt == msg->payload_len) { msg->payload_cnt = 0; msg->status++; }
            break;
        case 7:
            msg->checksum = data;
            msg->status = 0;
            if(micolink_check_sum(msg)) return true;
            break;
        default:
            msg->status = 0; msg->payload_cnt = 0;
            break;
    }
    return false;
}

void MTF01::MTF01_ProcessByte(uint8_t byte)
{
    if(micolink_parse_char(&msg, byte))
    {
        if(msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR)
        {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, msg.payload, sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t));

            float dist_m = static_cast<float>(payload.distance) / 1000.0f; // Convertir a metros

            mtf.distance = dist_m;
            mtf.flowX = (static_cast<float>(payload.flow_vel_x) * dist_m) / 100.0f; // Convertir a m/s
            mtf.flowY = (static_cast<float>(-payload.flow_vel_y) * dist_m) / 100.0f; // Convertir a m/s
            //mtf.flowX = payload.flow_vel_x;
            //mtf.flowY = -payload.flow_vel_y;
            mtf.quality = payload.flow_quality;
            mtf.status = payload.distance_status;
            nuevo_ = true; 
        }
    }
}

void MTF01::MTF01_GetData(MTF01_Data *data)
{
    __disable_irq();
    memcpy(data, &mtf, sizeof(MTF01_Data));
    __enable_irq();
}

void MTF01::ActualizarPosicion(float dt_s)
{
    float vx_ms = mtf.flowY;
    float vy_ms = mtf.flowX;

    if (mtf.quality > 50 && mtf.status == 1)
    {
        posicion_mtf.x += vx_ms * dt_s;
        posicion_mtf.y += vy_ms * dt_s;

        // Leaky Integrator (Filtro de fuga)
        posicion_mtf.x *= 0.998f;
        posicion_mtf.y *= 0.998f;
    }
    else
    {
        // Redução gradual caso o sensor perca leitura válida
        posicion_mtf.x *= 0.95f;
        posicion_mtf.y *= 0.95f;
    }
}

