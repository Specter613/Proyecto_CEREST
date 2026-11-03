/*
 * bateria.cpp — voltaje y corriente del módulo de potencia (tipo 3DR) por ADC
 */
#include "Sensores/bateria.hpp"
#include <cmath>

// Calibración de fábrica de VREFINT (STM32F4): valor del ADC medido con VDDA = 3.3 V
static const uint16_t *const kVrefintCal = reinterpret_cast<const uint16_t*>(0x1FFF7A2AU);

uint16_t Bateria::LeerCanal(uint32_t canal)
{
    ADC_ChannelConfTypeDef conf{};
    conf.Channel      = canal;
    conf.Rank         = 1;
    conf.SamplingTime = ADC_SAMPLETIME_480CYCLES;   // el más largo: estable con alta impedancia
    if (HAL_ADC_ConfigChannel(hadc_, &conf) != HAL_OK) return 0;

    uint16_t valor = 0;
    HAL_ADC_Start(hadc_);
    if (HAL_ADC_PollForConversion(hadc_, 2) == HAL_OK) {
        valor = static_cast<uint16_t>(HAL_ADC_GetValue(hadc_));
    }
    HAL_ADC_Stop(hadc_);
    return valor;
}

uint16_t Bateria::LeerPromedio(uint32_t canal, uint8_t muestras)
{
    uint32_t suma = 0;
    for (uint8_t i = 0; i < muestras; i++) suma += LeerCanal(canal);
    return static_cast<uint16_t>(suma / muestras);
}

void Bateria::ActualizarVdda()
{
    // VDDA real a partir de la referencia interna calibrada de fábrica
    const uint16_t raw = LeerPromedio(ADC_CHANNEL_VREFINT, 4);
    if (raw == 0) return;
    const float vdda = 3.3f * static_cast<float>(*kVrefintCal) / static_cast<float>(raw);
    if (vdda > 2.9f && vdda < 3.6f) vdda_ = vdda;   // descarta lecturas absurdas
}

int8_t Bateria::PorcentajePorVoltaje(float v)
{
    // Curva aproximada de una celda LiPo en reposo (V -> %)
    static const float tabla[][2] = {
        {3.30f, 0.0f}, {3.50f, 5.0f}, {3.68f, 20.0f}, {3.74f, 40.0f}, {3.80f, 60.0f},
        {3.90f, 75.0f}, {4.00f, 85.0f}, {4.10f, 95.0f}, {4.20f, 100.0f},
    };
    constexpr int n = sizeof(tabla) / sizeof(tabla[0]);

    if (v <= tabla[0][0])     return 0;
    if (v >= tabla[n - 1][0]) return 100;
    for (int i = 1; i < n; i++) {
        if (v < tabla[i][0]) {
            const float t = (v - tabla[i - 1][0]) / (tabla[i][0] - tabla[i - 1][0]);
            return static_cast<int8_t>(tabla[i - 1][1] + t * (tabla[i][1] - tabla[i - 1][1]));
        }
    }
    return 100;
}

void Bateria::Init()
{
    ActualizarVdda();
    ultimoVdda_ = HAL_GetTick();

    // Primera lectura sin filtro: punto de partida
    const float pinV = LeerPromedio(canalV_, 8) * vdda_ / 4095.0f;
    const float pinI = LeerPromedio(canalI_, 8) * vdda_ / 4095.0f;
    voltaje_   = pinV * kMultVoltaje;
    corriente_ = (pinI - kOffsetCorriente) * kAmpPorVolt;
    if (corriente_ < 0.0f) corriente_ = 0.0f;

    // Celdas: con la batería cargada cada celda está en <= 4.3 V
    if (voltaje_ > 3.0f) {
        celdas_ = static_cast<uint8_t>(std::ceil(voltaje_ / 4.3f));
        porcentajeInicial_ = PorcentajePorVoltaje(voltaje_ / celdas_);
        porcentaje_ = static_cast<int8_t>(porcentajeInicial_);
    }
}

void Bateria::Update(float dt)
{
    if (dt <= 0.0f) return;

    // VDDA una vez por segundo (cambia muy lento)
    if (HAL_GetTick() - ultimoVdda_ >= 1000) {
        ultimoVdda_ = HAL_GetTick();
        ActualizarVdda();
    }

    const float pinV = LeerPromedio(canalV_, 2) * vdda_ / 4095.0f;
    const float pinI = LeerPromedio(canalI_, 2) * vdda_ / 4095.0f;

    float vNuevo = pinV * kMultVoltaje;
    float iNuevo = (pinI - kOffsetCorriente) * kAmpPorVolt;
    if (iNuevo < 0.0f) iNuevo = 0.0f;

    // Filtros pasa bajas
    voltaje_   += dt / (kTauVoltaje   + dt) * (vNuevo - voltaje_);
    corriente_ += dt / (kTauCorriente + dt) * (iNuevo - corriente_);

    // mAh consumidos: A · s -> mAh
    mAh_ += corriente_ * dt * 1000.0f / 3600.0f;

    // Celdas (por si la batería se conectó después del arranque)
    if (celdas_ == 0 && voltaje_ > 3.0f) {
        celdas_ = static_cast<uint8_t>(std::ceil(voltaje_ / 4.3f));
        porcentajeInicial_ = PorcentajePorVoltaje(voltaje_ / celdas_);
    }

    // Porcentaje: por carga consumida si hay capacidad; si no, por voltaje
    if (celdas_ == 0) {
        porcentaje_ = -1;
    } else if (kCapacidadMah > 0.0f && porcentajeInicial_ >= 0.0f) {
        float p = porcentajeInicial_ - mAh_ / kCapacidadMah * 100.0f;
        if (p < 0.0f) p = 0.0f;
        porcentaje_ = static_cast<int8_t>(p);
    } else {
        porcentaje_ = PorcentajePorVoltaje(voltaje_ / celdas_);
    }
}