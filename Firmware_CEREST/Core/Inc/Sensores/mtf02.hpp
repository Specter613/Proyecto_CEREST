/*
 * mtf02.hpp
 *
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */
/*
 * mtf02.hpp
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */
#ifndef INC_SENSORES_MTF02_HPP_
#define INC_SENSORES_MTF02_HPP_
#include <cstdint>
#include <cstring>
#include "stm32f4xx_hal.h"
#include "Proceso/uart_dispatch.hpp"

#define MICOLINK_MSG_HEAD            0xEF // STX
#define MICOLINK_MAX_PAYLOAD_LEN     255
#define MICOLINK_MAX_LEN             (MICOLINK_MAX_PAYLOAD_LEN + 7)
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51

typedef struct
{
    uint8_t stx;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t payload_len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum; 
    //Adicional 
    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

#pragma pack(push,1)
typedef struct
{
    uint32_t system_time;
    uint32_t distance; // Valor minimo de 1, 0  para invalido
    uint8_t distance_strength; //0 - 255
    uint8_t distance_precision; //0 - 255 cuanto menor sea el valor mayor sera la precision
    uint8_t distance_status; // 1 valido, 0 invalido
    uint8_t reserved1;
    int16_t flow_vel_x; // velocidad(cm/s) = flow_vel_x * distance(m)
    int16_t flow_vel_y; // velocidad(cm/s) = flow_vel_y * distance(m)
    uint8_t flow_quality;//  0 - 255 un valor mayor indica una mayor calidad de los datos de flujo
    uint8_t flow_status; // 1 valido, 0 invalido
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

typedef struct
{
    float distance;
    float flowX;
    float flowY;
    uint8_t quality;
    uint8_t status;
} MTF01_Data;

typedef struct
{
    float x;
    float y;
} PosicionMTF;

class MTF01
{
public:
    explicit MTF01(UART_HandleTypeDef *huart) : huart_(huart) {}
    void Init();

    bool micolink_check_sum(MICOLINK_MSG_t *msg);
    bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);
    void MTF01_ProcessByte(uint8_t byte);

    void ActualizarPosicion(float dt);
    bool ConsumirDatoNuevo() { bool n = nuevo_; nuevo_ = false; return n; }


    void MTF01_GetData(MTF01_Data *data);
    const MTF01_Data& GetData() const { return mtf; }
    const PosicionMTF& GetPosicion() const { return posicion_mtf; }
    void ResetPosicion() { posicion_mtf.x = 0.0f; posicion_mtf.y = 0.0f; }

private:
    static void RxFullHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxHalfHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxErrorHandlerTrampoline(UART_HandleTypeDef *huart);
    void RxFullHandler();
    void RxHalfHandler();
    void RxErrorHandler();   // <-- NUEVO: la pieza que faltaba
    volatile bool nuevo_ = false;

    static constexpr uint16_t kSizeBuff = 128;

    UART_HandleTypeDef *huart_;
    uint8_t rxBufDMA_[kSizeBuff] = {};
    MICOLINK_MSG_t msg = {};
    MTF01_Data mtf = {};
    PosicionMTF posicion_mtf = {};

    static MTF01 *activeInstance;
};
#endif /* INC_SENSORES_MTF02_HPP_ */
