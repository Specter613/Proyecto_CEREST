/*
 * uart_dispatch.hpp
 *
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */

#ifndef INC_PROCESO_UART_DISPATCH_HPP_
#define INC_PROCESO_UART_DISPATCH_HPP_

#include "stm32f4xx_hal.h"

typedef void (*ManejadorUart)(UART_HandleTypeDef *huart);

class UartDispatch
{
public:
    static void Registrar(UART_HandleTypeDef *huart, ManejadorUart manejador);
    static void RegistrarMitad(UART_HandleTypeDef *huart, ManejadorUart manejador);
    static void RegistrarError(UART_HandleTypeDef *huart, ManejadorUart manejador);
    static void RegistrarTx(UART_HandleTypeDef *huart, ManejadorUart manejador);

    static void ManejarRxCompleto(UART_HandleTypeDef *huart);
    static void ManejarRxMitadCompleto(UART_HandleTypeDef *huart);
    static void ManejarError(UART_HandleTypeDef *huart);
    static void ManejarTxCompleto(UART_HandleTypeDef *huart);

private:
    struct Registro { UART_HandleTypeDef *huart = nullptr; ManejadorUart manejador = nullptr; };
    static constexpr uint8_t kMaxManejadores = 4;

    static Registro rx_[kMaxManejadores];       static uint8_t contadorRx_;
    static Registro rxMitad_[kMaxManejadores];  static uint8_t contadorRxMitad_;
    static Registro error_[kMaxManejadores];    static uint8_t contadorError_;
    static Registro tx_[kMaxManejadores];       static uint8_t contadorTx_;
};

#endif /* INC_PROCESO_UART_DISPATCH_HPP_ */
