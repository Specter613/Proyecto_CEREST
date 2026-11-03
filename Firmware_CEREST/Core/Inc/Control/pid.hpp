/*
 * pid.hpp
 *
 *  Created on: 8 ago 2026
 *      Author: specter0163
 *
 * LAZO EXTERIOR (ángulo)                    LAZO INTERIOR (velocidad)
 * ┌─────────────────────┐                   ┌──────────────────────┐
 * │ Mando RC (-30:30)    │                   │ Setpoint: sale del   │
 * │ Ángulo MPU6050  ──○──│──── PID Ángulo ──►│  PID Ángulo (°/s)    │
 * │ (real, negativo)     │      e(t)         │  Vel. MPU6050 ──○──  │
 * │                      │                   │  (real, negativo) e(t)│
 * └─────────────────────┘                   │ → PID Velocidad → µs │
 *                                            └──────────────────────┘
 */

#ifndef INC_CONTROL_PID_HPP_
#define INC_CONTROL_PID_HPP_

struct GananciasPID {
    // --- BUCLE EXTERIOR (ÁNGULO) ---
    float kp_angulo_roll;
    float ki_angulo_roll;
    float kd_angulo_roll;

    float kp_angulo_pitch;
    float ki_angulo_pitch;
    float kd_angulo_pitch;

    // --- BUCLE INTERIOR (VELOCIDAD ANGULAR) ---
    float kp_velocidad_roll;
    float ki_velocidad_roll;
    float kd_velocidad_roll;

    float kp_velocidad_pitch;
    float ki_velocidad_pitch;
    float kd_velocidad_pitch;

    float kp_velocidad_yaw;
    float ki_velocidad_yaw;
    float kd_velocidad_yaw;

    bool Guardar();

    bool Cargar();
};

class ControladorPID
{
public:
    /**
     * @brief Construye el controlador con los límites de su salida.
     * @param salidaMin    límite inferior de la salida ya sumada (P+I+D)
     * @param salidaMax    límite superior de la salida ya sumada (P+I+D)
     */
    ControladorPID(float salidaMin, float salidaMax);

    /**
     * @brief Ejecuta un ciclo completo del PID.
     * @param consigna valor deseado (setpoint)
     * @param medidaFiltron  valor real medido (ángulo o velocidad, según el bucle)
     * @param dt       tiempo transcurrido desde la última llamada, en segundos
     * @param kp,ki,kd ganancias — se leen en vivo, reflejan cambios por
     *                 MAVLink al instante, sin necesitar sincronización
     * @return salida ya limitada a [salidaMin, salidaMax]
     */
    float ActualizarPID(float consigna, float medida, float dt, float kp, float ki, float kd);

    /** @brief Reinicia el estado interno (integral, derivada, primera
     *  llamada) — llamar siempre al desarmar, para no arrastrar windup
     *  de un vuelo al siguiente. */
    void Reiniciar();
    void ReiniciarIntegral();

    /** @brief Último término proporcional calculado */
    float ObtenerTerminoP() const { return terminoP_; }
    /** @brief Último término integral calculado. */
    float ObtenerTerminoI() const { return terminoI_; }
    /** @brief Último término derivativo calculado. */
    float ObtenerTerminoD() const { return terminoD_; }

private:
    float CalcularP(float error, float kp);
    float CalcularD(float error, float errorAnterior, float dt, float kd);
    float CalcularI(float error, float errorAnterior, float dt, float ki, bool saturando);
    bool  Saturacion(float salidaSat, float error) const;

    float salidaMin_;
    float salidaMax_;


    float integral_ = 0.0f;
    float derivada_ = 0.0f;
    float errorAnterior_ = 0.0f;
    bool  primeraLlamada_ = true;

    float terminoP_ = 0.0f;
    float terminoI_ = 0.0f;
    float terminoD_ = 0.0f;
};

#endif /* INC_CONTROL_PID_HPP_ */