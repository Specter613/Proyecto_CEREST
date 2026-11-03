/*
 * bmp180.hpp
 *  Created on: 7 ago 2026
 *      Author: specter0163
 */
#ifndef INC_SENSORES_BMP180_HPP_
#define INC_SENSORES_BMP180_HPP_

#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cmath>

class BMP180 {
public:
    enum OSS : uint8_t {
        ULTRA_LOW_POWER = 0,
        STANDARD        = 1,
        HIGH_RES        = 2,
        ULTRA_HIGH_RES  = 3
    };

    explicit BMP180(I2C_HandleTypeDef *hi2c, OSS mode = ULTRA_HIGH_RES);

    bool Init();
    bool Process();   // no bloqueante, sin parámetros
    void CalibrateZero(uint8_t muestras = 10);

    float GetTemperature() const { return temperaturaC_; }
    float GetPressurePa()   const { return presionPa_; }
    float GetPressurehPa()  const { return presionPa_ / 100.0f; }
    float GetAltitude()     const { return altitudM_; }

private:
    static constexpr uint8_t BMP180_ADDR     = 0xEE;
    static constexpr uint8_t REG_CALIB_START = 0xAA;
    static constexpr uint8_t REG_CONTROL     = 0xF4;
    static constexpr uint8_t REG_DATA        = 0xF6;
    static constexpr uint8_t CMD_READ_TEMP   = 0x2E;
    static constexpr uint8_t CMD_READ_PRESS  = 0x34;

    I2C_HandleTypeDef *hi2c_;
    OSS mode_;

    enum State : uint8_t { STATE_IDLE, STATE_WAIT_TEMP, STATE_WAIT_PRESS };
    State state_ = STATE_IDLE;
    uint32_t lastCmdTick_ = 0;
    int32_t rawTemp_ = 0;

    int16_t  AC1 = 0, AC2 = 0, AC3 = 0;
    uint16_t AC4 = 0, AC5 = 0, AC6 = 0;
    int16_t  B1  = 0, B2  = 0;
    int16_t  MB  = 0, MC  = 0, MD  = 0;

    float temperaturaC_ = 0.0f;
    float presionPa_    = 0.0f;
    float altitudM_     = 0.0f;
    float presionRefPa_ = 101325.0f;

    // --- Filtro contra glitches (mismo mecanismo de AP_Baro de ArduPilot) ---
    float mediaPresion_ = 0.0f;
    bool mediaPresionInicializada_ = false;
    static constexpr float kRangoFiltroPct = 0.5f;

    bool PresionValida(float presionPa);

    bool ReadCalibrationData();
    void CalculateTrueValues(int32_t ut, int32_t up);
    float CalculateAltitude(float pressurePa, float refPressurePa);
};

#endif
