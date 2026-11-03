/*
 * SerialTx.hpp
 *
 *  Created on: 10 ago 2026
 *      Author: specter0163
 */

#ifndef INC_PROCESO_SERIALTX_HPP_
#define INC_PROCESO_SERIALTX_HPP_

#include "Proceso/ManejoBuffer.hpp"
#include "stm32f4xx_hal.h"

class SerialTx
{
public:
    SerialTx(UART_HandleTypeDef *huart, uint8_t *almacen, uint16_t capacidad)
        : huart_(huart), buffer_(almacen, capacidad) {}

    uint16_t Escribir(const uint8_t *datos, uint16_t len) { return buffer_.Escribir(datos, len); }
    uint16_t EspacioLibre() const { return buffer_.EspacioLibre(); }

    // Llamado UNA VEZ POR CICLO desde App_Loop() — mismo patrón que
    // _timer_tick() de ArduPilot. Toda la decisión de "mandar el
    // siguiente pedazo" vive aquí, en contexto normal, no en la ISR.
    void Bombear()
    {
        if (transmitiendo_) return;
        if (huart_->gState != HAL_UART_STATE_READY) return;

        uint16_t longitudBloque;
        const uint8_t *bloque = buffer_.VerBloqueContiguo(longitudBloque);
        if (longitudBloque == 0) return;

        longitudPendiente_ = longitudBloque;
        transmitiendo_ = true;

        if (HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(bloque), longitudBloque) != HAL_OK)
        {
            transmitiendo_ = false;
            longitudPendiente_ = 0;
        }
    }

    // Llamado SOLO desde la ISR de TX complete — deliberadamente mínimo,
    // sin ningún cálculo de "qué sigue". Eso lo decide Bombear() después.
    void AlCompletarTx()
    {
        buffer_.Consumir(longitudPendiente_);
        longitudPendiente_ = 0;
        transmitiendo_ = false;
    }

    UART_HandleTypeDef* Handle() const { return huart_; }

private:
    UART_HandleTypeDef *huart_;
    BufferCircular buffer_;
    volatile bool transmitiendo_ = false;
    uint16_t longitudPendiente_ = 0;
};




#endif /* INC_PROCESO_SERIALTX_HPP_ */
