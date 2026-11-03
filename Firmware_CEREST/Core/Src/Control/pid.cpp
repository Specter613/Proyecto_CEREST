    /*
    * pid.cpp
    *
    *  Created on: 9 ago 2026
    *      Author: specter0163
    */
    #include "config.hpp"
    #include "Control/pid.hpp"
    #include "Proceso/sd.hpp"
    #include "usbd_cdc_if.h"
    #include <cmath>


    /** @brief Guarda directamente el objeto instanciado desde la RAM. */
    bool GananciasPID::Guardar()
    {
        return SDcard::Guardar(SDLayout::kPidFilename, SDLayout::kPidMagic, this, sizeof(GananciasPID));
    }

    /** @brief Carga usando un búfer temporal — evita corromper 'this' si
    *  la lectura falla a la mitad. Si no hay datos válidos, deja los
    *  valores por defecto ya inicializados en el struct. */
    bool GananciasPID::Cargar()
    {
        alignas(4) static GananciasPID tempBuffer;

        if (SDcard::Cargar(SDLayout::kPidFilename, SDLayout::kPidMagic, &tempBuffer, sizeof(GananciasPID)))
        {
            *this = tempBuffer;
            return true;
        }

        kp_angulo_roll = 5.5f;  ki_angulo_roll = 0.0f;  kd_angulo_roll = 0.f;
        kp_angulo_pitch = 5.5f; ki_angulo_pitch = 0.0f; kd_angulo_pitch = 0.f;

        // deja igual lo que ya tienes:
        kp_velocidad_roll = 1.75f;  ki_velocidad_roll = 0.35f;  kd_velocidad_roll = 0.001f;
        kp_velocidad_pitch = 1.75f; ki_velocidad_pitch = 0.45f; kd_velocidad_pitch = 0.001f;
        kp_velocidad_yaw = 1.3f;   ki_velocidad_yaw = 0.8f;   kd_velocidad_yaw = 0.0f;
        return false;
    }

    // ============================================================
    // ControladorPID — mismo patrón que AC_PID de ArduPilot
    // ============================================================
    ControladorPID::ControladorPID(float salidaMin, float salidaMax)
        : salidaMin_(salidaMin), salidaMax_(salidaMax) {}

    /** @brief Etapa proporcional — simplemente kp por el error. */
    float ControladorPID::CalcularP(float error, float kp)
    {
        return kp * error;
    }

    /**
    * @brief Etapa derivativa, calculada sobre la MEDICIÓN (no sobre el
    * error) — evita el "kick" (pico brusco de salida) cuando la consigna
    * cambia de golpe. Filtrada con paso bajo simple para no amplificar
    * el ruido del sensor.
    */
    float ControladorPID::CalcularD(float error, float errorAnterior, float dt, float kd)
    {
        constexpr float kCorteHz = 100.0f;                       // corte del filtro de la D
        float derivadaCruda = (error - errorAnterior) / dt;
        errorAnterior_ = error;

        const float alfa = dt / (dt + 1.0f / (2.0f * 3.14159265f * kCorteHz));
        derivada_ += alfa * (derivadaCruda - derivada_);        // derivada_ = miembro existente, ya filtrada
        return kd * derivada_;
    }

    /** @brief Determina si la salida sin limitar ya se saldría del rango
    *  permitido, Y en la misma dirección en que el error sigue empujando —
    *  condición usada por el anti-windup. */
    bool ControladorPID::Saturacion(float salidaSat, float error) const
    {
        bool saturaArriba = (salidaSat > salidaMax_) && (error > 0.0f);
        bool saturaAbajo  = (salidaSat < salidaMin_) && (error < 0.0f);
        return saturaArriba || saturaAbajo;
    }
    /**
    * @brief Etapa integral, con DOBLE seguridad anti-windup:
    * 1. Condicional: el integrador crece solo si la salida total NO está
    *    saturada, o si el error se opone a la dirección de la saturación.
    * 2. Límite absoluto: el integrador nunca puede crecer más de lo que
    *    por sí solo saturaría la salida — protege incluso en el caso donde
    *    el error nunca llega a saturar la salida TOTAL, pero el integrador
    *    igual se acumula lentamente durante mucho tiempo (el bug real que
    *    ya encontramos y corregimos: windup silencioso en banco).
    */
    float ControladorPID::CalcularI(float error, float errorAnterior, float dt, float ki, bool saturando)
    {
        if(!saturando)
        {
            integral_ += ((error + errorAnterior) * dt) / 2.0f;
            
            if(ki > 1e-6f)
            {
                float integralMax = std::fmax(std::fabs(salidaMax_), std::fabs(salidaMin_)) / ki;   
                if(integral_ > integralMax) integral_ = integralMax;
                if(integral_ < -integralMax) integral_ = -integralMax;
            }
        }
        errorAnterior_ = error;
        return ki * integral_;
    }

    float ControladorPID::ActualizarPID(float consigna, float medida, float dt, float kp, float ki, float kd)
    {
        if(dt <= 0.0f) return 0.0f;
        float error = consigna - medida;

        if(primeraLlamada_)
        {
            errorAnterior_ = error;
            primeraLlamada_ = false;
        }

        
        float errorAnterior = errorAnterior_;

        terminoP_ = CalcularP(error, kp);
        terminoD_ = CalcularD(error, errorAnterior, dt, kd);

        float salidaPrevia = terminoP_ + terminoI_ + terminoD_;
        bool saturando = Saturacion(salidaPrevia, error);

        terminoI_ = CalcularI(error, errorAnterior, dt, ki, saturando);

        float salidaPID = terminoP_ + terminoI_ + terminoD_;
        if(salidaPID > salidaMax_) salidaPID = salidaMax_;
        if(salidaPID < salidaMin_) salidaPID = salidaMin_;
        return salidaPID;
        
    }

    /** @brief Reinicia integral, derivada y bandera de primera llamada. */
    void ControladorPID::Reiniciar()
    {
        integral_ = 0.0f;
        derivada_ = 0.0f;
        errorAnterior_ = 0.0f;
        primeraLlamada_ = true;
        terminoP_ = terminoI_ = terminoD_ = 0.0f;
    }

    /** @brief Reinicia solo la integral: P y D siguen funcionando normal. */
    void ControladorPID::ReiniciarIntegral()
    {
        integral_ = 0.0f;
        terminoI_ = 0.0f;
    }