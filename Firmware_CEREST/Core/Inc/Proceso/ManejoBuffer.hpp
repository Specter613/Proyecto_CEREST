/*
 * ManejoBuffer.hpp
 *
 *  Created on: 10 ago 2026
 *      Author: specter0163
 */

#ifndef INC_PROCESO_MANEJOBUFFER_HPP_
#define INC_PROCESO_MANEJOBUFFER_HPP_

#include <cstdint>

// Un solo productor (quien llama Escribir) / un solo consumidor (quien
// llama Consumir) — como cabeza_ y cola_ cada uno los toca SOLO un lado,
// no hace falta __disable_irq() en el caso normal.
class BufferCircular
{
public:
    BufferCircular(uint8_t *almacen, uint16_t capacidad)
        : buffer_(almacen), capacidad_(capacidad) {}

    uint16_t EspacioLibre() const
    {
        uint16_t usado = Disponible();
        return capacidad_ - usado - 1;
    }

    uint16_t Disponible() const
    {
        uint16_t h = cabeza_, t = cola_;
        return (h >= t) ? (h - t) : (capacidad_ - t + h);
    }

    // Regresa cuántos bytes REALMENTE entraron — nunca sobreescribe
    // datos viejos si no hay espacio.
    uint16_t Escribir(const uint8_t *datos, uint16_t len)
    {
        uint16_t espacio = EspacioLibre();
        uint16_t aEscribir = (len < espacio) ? len : espacio;
        for (uint16_t i = 0; i < aEscribir; i++)
        {
            buffer_[cabeza_] = datos[i];
            cabeza_ = (cabeza_ + 1) % capacidad_;
        }
        return aEscribir;
    }

    // Da el siguiente bloque contiguo listo para DMA, sin copiar.
    const uint8_t* VerBloqueContiguo(uint16_t &longitudSalida) const
    {
        uint16_t h = cabeza_, t = cola_;
        longitudSalida = (h >= t) ? (h - t) : (capacidad_ - t);
        return &buffer_[t];
    }

    void Consumir(uint16_t len) { cola_ = (cola_ + len) % capacidad_; }

private:
    uint8_t *buffer_;
    uint16_t capacidad_;
    volatile uint16_t cabeza_ = 0;
    volatile uint16_t cola_ = 0;
};



#endif /* INC_PROCESO_MANEJOBUFFER_HPP_ */
