/*
 * servo.hpp — servo SG90 por PWM (50 Hz) en un canal de timer
 */
#ifndef INC_ACTUADORES_SERVO_HPP_
#define INC_ACTUADORES_SERVO_HPP_

#include "stm32f4xx_hal.h"
#include <cstdint>

class Servo {
public:
    Servo(TIM_HandleTypeDef *htim, uint32_t canal) : htim_(htim), canal_(canal) {}

    /** Ajusta el timer a 1 tick = 1 µs y periodo de 20 ms (50 Hz), y arranca el PWM. */
    void Init();

    /** Mueve el servo al ángulo indicado (se limita a kGradosMin..kGradosMax). */
    void SetGrados(float grados);

    /** Convierte un canal RC (1000..2000 µs) al rango de ángulos y mueve el servo. */
    void SetDesdeCanal(uint16_t canalUs);

    // ======================= AJUSTES =======================
    static constexpr float kPulsoMinUs  = 500.0f;   // µs a 0°   (SG90)
    static constexpr float kPulsoMaxUs  = 2500.0f;  // µs a 180° (SG90)
    static constexpr float kGradosServo = 180.0f;   // recorrido total del servo
    static constexpr float kGradosMin   = 0.0f;     // rango que se usa con el mando
    static constexpr float kGradosMax   = 90.0f;

private:
    TIM_HandleTypeDef *htim_;
    uint32_t canal_;
};

#endif /* INC_ACTUADORES_SERVO_HPP_ */