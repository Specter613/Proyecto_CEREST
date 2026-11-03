/*
 * buzzer.hpp — buzzer por GPIO (transistor NPN), melodías sin bloquear
 */
#ifndef INC_ACTUADORES_BUZZER_HPP_
#define INC_ACTUADORES_BUZZER_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"

// Melodía: duraciones en ms alternando encendido, apagado, encendido, ...
struct MelodiaBuzzer {
    const uint16_t *duraciones;
    uint8_t cantidad;
};

class Buzzer {
public:
    // Polaridad: true = suena con el pin en ALTO; false = suena con el pin en BAJO
    static constexpr bool kActivoEnAlto = false;

    Buzzer(GPIO_TypeDef *puerto, uint16_t pin) : puerto_(puerto), pin_(pin) {}

    /** Deja el buzzer en silencio (llamarlo lo antes posible en App_Init). */
    void Init();

    /** Empieza una melodía (reemplaza la que esté sonando). */
    void Reproducir(const MelodiaBuzzer &melodia);

    /** Avanza la melodía. Llamar periódicamente con el tiempo actual en ms. */
    void Update(uint32_t ahoraMs);

    bool Sonando() const { return melodia_ != nullptr; }

private:
    void Escribir(bool sonar);

    GPIO_TypeDef *puerto_;
    uint16_t pin_;

    const MelodiaBuzzer *melodia_ = nullptr;
    uint8_t  paso_        = 0;
    uint32_t inicioPaso_  = 0;
    bool     iniciar_     = false;
};

// --- Melodías disponibles ---
extern const MelodiaBuzzer kMelodiaInicioListo;   // dos pitidos cortos
extern const MelodiaBuzzer kMelodiaGpsListo;      // tres cortos y uno largo

#endif /* INC_ACTUADORES_BUZZER_HPP_ */