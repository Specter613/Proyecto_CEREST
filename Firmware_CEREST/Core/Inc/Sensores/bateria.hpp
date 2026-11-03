/*
 * bateria.hpp — voltaje y corriente del módulo de potencia (tipo 3DR) por ADC
 */
#ifndef INC_SENSORES_BATERIA_HPP_
#define INC_SENSORES_BATERIA_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"

class Bateria {
public:
    Bateria(ADC_HandleTypeDef *hadc, uint32_t canalVoltaje, uint32_t canalCorriente)
        : hadc_(hadc), canalV_(canalVoltaje), canalI_(canalCorriente) {}

    /** Primera lectura: VDDA real, voltaje inicial, celdas y porcentaje de partida. */
    void Init();

    /** Lee el ADC, filtra e integra los mAh. dt en segundos. */
    void Update(float dt);

    float   Voltaje() const    { return voltaje_; }      // V
    float   Corriente() const  { return corriente_; }    // A
    float   mAh() const        { return mAh_; }          // mAh consumidos
    uint8_t Celdas() const     { return celdas_; }
    int8_t  Porcentaje() const { return porcentaje_; }   // 0..100, -1 = desconocido

    // ======================= AJUSTES =======================
    static constexpr float kMultVoltaje     = 10.82f;   // V batería por V en el pin (3DR estándar)
    static constexpr float kAmpPorVolt      = 17.0f;   // A por V en el pin (3DR estándar)
    static constexpr float kOffsetCorriente = 0.0f;    // V en el pin de corriente sin consumo
    static constexpr float kCapacidadMah    = 0.0f;    // mAh de tu batería (0 = % solo por voltaje)
    static constexpr float kTauVoltaje      = 0.5f;    // s: filtro del voltaje
    static constexpr float kTauCorriente    = 0.2f;    // s: filtro de la corriente

private:
    uint16_t LeerCanal(uint32_t canal);
    uint16_t LeerPromedio(uint32_t canal, uint8_t muestras);
    void     ActualizarVdda();
    static int8_t PorcentajePorVoltaje(float voltajeCelda);

    ADC_HandleTypeDef *hadc_;
    uint32_t canalV_, canalI_;

    float   vdda_       = 3.3f;
    float   voltaje_    = 0.0f;
    float   corriente_  = 0.0f;
    float   mAh_        = 0.0f;
    uint8_t celdas_     = 0;
    int8_t  porcentaje_ = -1;
    float   porcentajeInicial_ = -1.0f;
    uint32_t ultimoVdda_ = 0;
};

#endif /* INC_SENSORES_BATERIA_HPP_ */