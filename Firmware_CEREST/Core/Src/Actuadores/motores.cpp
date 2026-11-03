#include "config.hpp"
#include "Actuadores/motores.hpp"
#include "usbd_cdc_if.h"

/** @brief Construye el controlador, asociándolo al timer que genera el PWM. */
MotorController::MotorController(TIM_HandleTypeDef *htim3)
    : htim3_(htim3) {}

/** @brief Arranca el PWM en los 4 canales y deja los motores en mínimo,
 *  desarmados — estado seguro garantizado justo después de energizar. */
void MotorController::Inicio()
{
    initStatus_[0] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_1);
    initStatus_[1] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_2);
    initStatus_[2] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_3);
    initStatus_[3] = HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_4);
	EstablecerPwm(htim3_, TIM_CHANNEL_1, kPwmMin);
	EstablecerPwm(htim3_, TIM_CHANNEL_2, kPwmMin);
	EstablecerPwm(htim3_, TIM_CHANNEL_3, kPwmMin);
	EstablecerPwm(htim3_, TIM_CHANNEL_4, kPwmMin);
    Desarmar();
}

/** @brief Escribe un pulso PWM en microsegundos a un canal del timer,
 *  limitado siempre al rango [kPwmMin, kPwmMax] antes de aplicarlo. */
void MotorController::EstablecerPwm(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pulse_us)
{
    if(pulse_us < kPwmMin) pulse_us = kPwmMin;
    if(pulse_us > kPwmMax) pulse_us = kPwmMax;
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

/** @brief Fuerza los 4 motores a pulso mínimo (desarmado) — estado
 *  seguro usado en cualquier condición de emergencia o desarmado normal. */
void MotorController::Desarmar()
{
    armed_ = false;
    pwm_[0] = pwm_[1] = pwm_[2] = pwm_[3] = kPwmDisarmed;
    EstablecerPwm(htim3_, TIM_CHANNEL_1, kPwmDisarmed);   
    EstablecerPwm(htim3_, TIM_CHANNEL_2, kPwmDisarmed);   
    EstablecerPwm(htim3_, TIM_CHANNEL_3, kPwmDisarmed);   
    EstablecerPwm(htim3_, TIM_CHANNEL_4, kPwmDisarmed);   
}

/**
Pasos para mezclar:
1) Mezcla solo roll y pitch
2) Calcula cuanto Yaw puede agregarse
4) Si las correcciones no caben ni con todo el rango se escalan
5)Ajusta Throttle para que el motor mas bajo quede >= kIdle y el mas Alto <= kPwmSeguridad
6) Escribe el PWM
 */
void MotorController::AplicarMezcla(float throttle_us, float roll_us, float pitch_us, float yaw_us)
{   
    const float idle = static_cast<float>(kIdle);
    const float tope = static_cast<float>(kPwmSeguridad);
    const float rango = tope - idle; // Espacio disponible para mezclar

    if(throttle_us < kUmbralArranque){
        uint16_t thr = static_cast<uint16_t>(Limitar(throttle_us, static_cast<float>(kPwmMin), tope)); 
        pwm_[0] = pwm_[1] = pwm_[2] = pwm_[3] = thr;
        return; 
    }

    // 1)
    float rPB0 = -roll_us - pitch_us; 
    float rPB4 = roll_us + pitch_us;
    float rPB5 = roll_us - pitch_us;
    float rPB1 = -roll_us + pitch_us;

    float rPBMax = Max4(rPB0, rPB4, rPB5, rPB1);
    float rPBMin = Min4(rPB0, rPB4, rPB5, rPB1);

    // 2)
    float disponible = (rango - (rPBMax - rPBMin)) * 0.5f; // Mitad del espacio restante para Yaw
    float yawPermitido = std::fmax(disponible, static_cast<float>(kYawMinimo)); // Al menos 40us de autoridad de Yaw
    float yaw = Limitar(yaw_us, -yawPermitido, yawPermitido);

    // 3)

    float mPB0 = rPB0 + yaw;
    float mPB4 = rPB4 + yaw;
    float mPB5 = rPB5 - yaw;
    float mPB1 = rPB1 - yaw;

    float rMax = Max4(mPB0, mPB4, mPB5, mPB1);
    float rMin = Min4(mPB0, mPB4, mPB5, mPB1);

    // 4) 
    float spread = rMax - rMin;
    if(spread > rango){
        float k = rango / spread;
        mPB0 *= k; mPB4 *= k; mPB5 *= k; mPB1 *= k;
        rMax *= k; rMin *= k;
    }

    // 5)
    float t = Limitar(throttle_us, idle -rMin, tope -rMax); 

    // 6)
    pwm_[0] = static_cast<uint16_t>(Limitar(t + mPB4, idle, tope));
    pwm_[1] = static_cast<uint16_t>(Limitar(t + mPB5, idle, tope));
    pwm_[2] = static_cast<uint16_t>(Limitar(t + mPB0, idle, tope));
    pwm_[3] = static_cast<uint16_t>(Limitar(t + mPB1, idle, tope));


    #if HABILITAR_DEBUG_PID
    static char msgFull[160];
    int lenFull = snprintf(msgFull, sizeof(msgFull),
        "FULL: in(t=%.0f r=%.0f p=%.0f y=%.0f) adj(t=%.0f y=%.0f) out(PB4=%u PB5=%u PB0=%u PB1=%u)\r\n",
        throttle_us, roll_us, pitch_us, yaw_us, t, yaw, pwm_[0], pwm_[1], pwm_[2], pwm_[3]);
    CDC_Transmit_FS((uint8_t*)msgFull, lenFull);
    #endif
}

/**
 * @brief Ahora TODOS los parámetros llegan en microsegundos —
 * throttle_us típicamente 1000-2000, roll/pitch/yaw_us son DELTAS
 * (pueden ser negativos), del mismo tamaño que la autoridad real del
 * PID configurada en ControlDeVuelo.
 */
void MotorController::Actualizar(bool armed, bool signalLost,
                                  float throttle_us, float roll_us, float pitch_us, float yaw_us)
{
    if(signalLost) { Desarmar(); return; }
    if(!armed)
    {
        wasArmed_ = false;
        Desarmar();
        return;
    }
    if(armed && !wasArmed_ && throttle_us > kPwmMin + 50.0f)
    {
        Desarmar();
        return;
    }
    wasArmed_ = true;
    armed_ = true;

    throttle_us = std::fmax(kPwmMin, std::fmin(kPwmMax, throttle_us));
    // roll/pitch/yaw_us NO se recortan aquí a ±1 nunca más — su límite
    // real ya lo impone ControlDeVuelo (ver más abajo), y el recorte
    // final ocurre UNA sola vez, dentro de AplicarMezcla.

    AplicarMezcla(throttle_us, roll_us, pitch_us, yaw_us);

    EstablecerPwm(htim3_, TIM_CHANNEL_1, pwm_[0]);
    EstablecerPwm(htim3_, TIM_CHANNEL_2, pwm_[1]);
    EstablecerPwm(htim3_, TIM_CHANNEL_3, pwm_[2]);
    EstablecerPwm(htim3_, TIM_CHANNEL_4, pwm_[3]);
}


/**
 * @brief Secuencia de calibración de rango del ESC: máximo (el ESC ya
 * energizado entra en modo programación) → espera → mínimo → espera.
 * Solo debe llamarse explícitamente, nunca en el arranque normal —
 * bloquea el sistema por ~6 segundos.
 */
void MotorController::CalibrarEsc()
{
	  HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(htim3_, TIM_CHANNEL_4);
	  EstablecerPwm(htim3_, TIM_CHANNEL_1, kPwmMax);
	  EstablecerPwm(htim3_, TIM_CHANNEL_2, kPwmMax);
	  EstablecerPwm(htim3_, TIM_CHANNEL_3, kPwmMax);
	  EstablecerPwm(htim3_, TIM_CHANNEL_4, kPwmMax);
	  HAL_Delay(3000);
	  EstablecerPwm(htim3_, TIM_CHANNEL_1, kPwmMin);
	  EstablecerPwm(htim3_, TIM_CHANNEL_2, kPwmMin);
	  EstablecerPwm(htim3_, TIM_CHANNEL_3, kPwmMin);
	  EstablecerPwm(htim3_, TIM_CHANNEL_4, kPwmMin);
	  HAL_Delay(3000);
    Desarmar();
}

/**
 * @brief Failsafe de inclinación extrema — si roll o pitch superan el
 * ángulo máximo permitido, desarma de inmediato. Protege contra pérdida
 * total de control o un choque violento contra el suelo/pared.
 */
bool MotorController::VerificarFailsafeInclinacion(float rollDeg, float pitchDeg)
{
    if(fabsf(rollDeg) > maxTiltAngle_ || fabsf(pitchDeg) > maxTiltAngle_)
    {
        Desarmar();
        return true;
    }
    return false;
}