/*
 * buzzer.cpp — buzzer por GPIO (transistor NPN), melodías sin bloquear
 */
#include "Sensores/buzzer.hpp"

// --- Melodías: encendido, apagado, encendido, ... (ms) ---
static const uint16_t kInicio[] = {100, 100, 100};
static const uint16_t kGps[]    = {80, 80, 80, 80, 80, 80, 400};

const MelodiaBuzzer kMelodiaInicioListo = {kInicio, sizeof(kInicio) / sizeof(kInicio[0])};
const MelodiaBuzzer kMelodiaGpsListo    = {kGps,    sizeof(kGps)    / sizeof(kGps[0])};

void Buzzer::Escribir(bool sonar)
{
    // Con polaridad invertida, "sonar" corresponde al pin en bajo
    const bool nivelAlto = kActivoEnAlto ? sonar : !sonar;
    HAL_GPIO_WritePin(puerto_, pin_, nivelAlto ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Buzzer::Init()
{
    melodia_ = nullptr;
    Escribir(false);
}

void Buzzer::Reproducir(const MelodiaBuzzer &melodia)
{
    if (melodia.cantidad == 0) return;
    melodia_ = &melodia;
    paso_ = 0;
    iniciar_ = true;   // el primer paso empieza en el próximo Update
}

void Buzzer::Update(uint32_t ahoraMs)
{
    if (melodia_ == nullptr) return;

    if (iniciar_) {
        iniciar_ = false;
        inicioPaso_ = ahoraMs;
        Escribir(true);   // los pasos pares son "sonar"
        return;
    }

    if (ahoraMs - inicioPaso_ < melodia_->duraciones[paso_]) return;

    // Siguiente paso
    paso_++;
    if (paso_ >= melodia_->cantidad) {
        melodia_ = nullptr;
        Escribir(false);
        return;
    }
    inicioPaso_ = ahoraMs;
    Escribir((paso_ % 2) == 0);
}