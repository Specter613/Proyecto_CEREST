/*
 * motores.hpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#ifndef INC_ACTUADORES_MOTORES_HPP_
#define INC_ACTUADORES_MOTORES_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"
#include <cmath>

/**
 * @brief Controla los 4 motores del cuadricóptero vía PWM (TIM3), aplicando
 * la mezcla de throttle/roll/pitch/yaw y las protecciones de seguridad
 * (desarmado, pérdida de señal, inclinación excesiva).
 */
class MotorController{
public:
	/**
	 * @brief Construye el controlador, asociándolo al timer que genera el PWM.
	 * @param htim3 handle del timer configurado con los 4 canales PWM
	 */
	MotorController(TIM_HandleTypeDef *htim3);

	/**
	 * @brief Arranca el PWM en los 4 canales y deja los motores en mínimo,
	 * desarmados — estado seguro garantizado justo después de energizar.
	 */
	void Inicio();

	/**
	 * @brief Actualiza los 4 motores a partir de throttle/roll/pitch/yaw
	 * normalizados (-1.0 a 1.0 para roll/pitch/yaw, 0.0 a 1.0 para throttle)
	 * y el estado de armado/failsafe. Esta es la ÚNICA función que debe
	 * tocar el mezclador.
	 * @param armed        true si el piloto solicita armado
	 * @param signalLost   true si se perdió la señal de radio
	 * @param throttle_us     0.0 a 1.0
	 * @param roll_us         -1.0 a 1.0
	 * @param pitch_us       -1.0 a 1.0
	 * @param yaw_us          -1.0 a 1.0
	 */
	void Actualizar(bool armed, bool signalLost, float throttle, float roll, float pitch, float yaw);

	/**
	 * @brief Fuerza los 4 motores a pulso mínimo (desarmado) — estado
	 * seguro usado en cualquier condición de emergencia o desarmado normal.
	 */
	void Desarmar();

	/**
	 * @brief Secuencia de calibración de rango del ESC: máximo (el ESC ya
	 * energizado entra en modo programación) → espera → mínimo → espera.
	 * Solo debe llamarse explícitamente, nunca en el arranque normal —
	 * bloquea el sistema por ~10 segundos.
	 */
	void CalibrarEsc();

	/** @brief Devuelve el último valor de PWM (en microsegundos) aplicado al motor indicado (0-3). */
	uint16_t ObtenerPwm(uint8_t motorIndex) const { return pwm_[motorIndex]; }

	/** @brief Devuelve el estado (HAL_StatusTypeDef) que dejó el arranque de cada uno de los 4 canales PWM. */
	const uint8_t* ObtenerEstadoInicial() const { return initStatus_; }

	/** @brief Define el ángulo máximo de inclinación (grados) permitido antes de que el failsafe desarme. */
	void EstablecerMaximoInclinacion(float maxAngleDeg) { maxTiltAngle_ = maxAngleDeg; }

	/**
	 * @brief Failsafe de inclinación extrema — si roll o pitch superan el
	 * ángulo máximo permitido, desarma de inmediato.
	 * @return true si el failsafe se activó (y ya desarmó los motores)
	 */
	bool VerificarFailsafeInclinacion(float rollDeg, float pitchDeg);

private:
	/** @brief Escribe un pulso PWM (microsegundos) a un canal del timer, limitado siempre a [kPwmMin, kPwmMax]. */
	void EstablecerPwm(TIM_HandleTypeDef *htim, uint8_t channel, uint16_t pulse_us);

	/** @brief Calcula la mezcla de throttle/roll/pitch/yaw para cada uno de los 4 motores. */
	void AplicarMezcla(float throttle, float roll, float pitch, float yaw);

	static inline float Limitar(float val, float low, float high) { return val < low ? low : (val > high ? high : val); }
	static inline float Max4(float a, float b, float c, float d) { return std::fmax(std::fmax(a, b), std::fmax(c, d)); }
	static inline float Min4(float a, float b, float c, float d) { return std::fmin(std::fmin(a, b), std::fmin(c, d)); }

	static constexpr uint16_t kPwmMin = 1000;
	static constexpr uint16_t kPwmMax = 2000;
	static constexpr uint16_t kPwmDisarmed = 1000;

	// Par Mezclador
	static constexpr uint16_t kPwmSeguridad = 1999;
	static constexpr uint16_t kIdle = 1180; // Pulso mínimo de seguridad para que el motor no se apague mientras está armado
	static constexpr uint16_t kUmbralArranque = 1050; // Pulso mínimo para considerar que el motor está arrancando
	static constexpr uint16_t kYawMinimo = 40; // 40us


	TIM_HandleTypeDef *htim3_;
	uint16_t pwm_[4] = { kPwmDisarmed, kPwmDisarmed, kPwmDisarmed, kPwmDisarmed };
	bool armed_ = false;
	uint8_t initStatus_[4] = {0, 0, 0, 0};
	float maxTiltAngle_ = 60.0f;
	bool wasArmed_ = false;
};

#endif /* INC_ACTUADORES_MOTORES_HPP_ */
