/*
 * servo.cpp — servo SG90 por PWM (50 Hz) en un canal de timer
 */
#include "Actuadores/servo.hpp"

void Servo::Init()
{
    // Reloj del timer: TIM2 está en APB1; si APB1 está dividido, el timer va al doble
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t relojTimer = ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : 2U * pclk1;

    // 1 tick = 1 µs, periodo = 20000 µs (50 Hz)
    __HAL_TIM_SET_PRESCALER(htim_, (relojTimer / 1000000U) - 1U);
    __HAL_TIM_SET_AUTORELOAD(htim_, 20000U - 1U);
    htim_->Instance->EGR = TIM_EGR_UG;   // carga el prescaler y el periodo ya

    SetGrados(kGradosMin);
    HAL_TIM_PWM_Start(htim_, canal_);
}

void Servo::SetGrados(float grados)
{
    if (grados < kGradosMin) grados = kGradosMin;
    if (grados > kGradosMax) grados = kGradosMax;

    const float pulso = kPulsoMinUs + grados * (kPulsoMaxUs - kPulsoMinUs) / kGradosServo;
    __HAL_TIM_SET_COMPARE(htim_, canal_, static_cast<uint32_t>(pulso));
}

void Servo::SetDesdeCanal(uint16_t canalUs)
{
    float x = (static_cast<float>(canalUs) - 1000.0f) / 1000.0f;   // 0..1
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    SetGrados(kGradosMin + x * (kGradosMax - kGradosMin));
}