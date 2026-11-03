/*
 * bmp180.cpp
 *  Created on: 7 ago 2026
 *      Author: specter0163
 */
#include "Sensores/bmp180.hpp"

BMP180::BMP180(I2C_HandleTypeDef *hi2c, OSS mode)
    : hi2c_(hi2c), mode_(mode) {}

bool BMP180::Init() {
    return ReadCalibrationData();
}

bool BMP180::ReadCalibrationData() {
    uint8_t calibBuf[22];
    if (HAL_I2C_Mem_Read(hi2c_, BMP180_ADDR, REG_CALIB_START, I2C_MEMADD_SIZE_8BIT, calibBuf, 22, 100) != HAL_OK) {
        return false;
    }
    AC1 = (int16_t)(calibBuf[0] << 8 | calibBuf[1]);
    AC2 = (int16_t)(calibBuf[2] << 8 | calibBuf[3]);
    AC3 = (int16_t)(calibBuf[4] << 8 | calibBuf[5]);
    AC4 = (uint16_t)(calibBuf[6] << 8 | calibBuf[7]);
    AC5 = (uint16_t)(calibBuf[8] << 8 | calibBuf[9]);
    AC6 = (uint16_t)(calibBuf[10] << 8 | calibBuf[11]);
    B1  = (int16_t)(calibBuf[12] << 8 | calibBuf[13]);
    B2  = (int16_t)(calibBuf[14] << 8 | calibBuf[15]);
    MB  = (int16_t)(calibBuf[16] << 8 | calibBuf[17]);
    MC  = (int16_t)(calibBuf[18] << 8 | calibBuf[19]);
    MD  = (int16_t)(calibBuf[20] << 8 | calibBuf[21]);
    return true;
}

bool BMP180::PresionValida(float presionPa)
{
    if (!mediaPresionInicializada_)
    {
        mediaPresion_ = presionPa;
        mediaPresionInicializada_ = true;
        return true;
    }
    float rango = mediaPresion_ * kRangoFiltroPct;
    bool valida = fabsf(presionPa - mediaPresion_) <= rango;
    if (valida)
    {
        constexpr float alpha = 0.1f;
        mediaPresion_ = mediaPresion_*(1.0f-alpha) + presionPa*alpha;
    }
    return valida;
}

bool BMP180::Process() {
    uint32_t now = HAL_GetTick();

    switch (state_) {
        case STATE_IDLE: {
            uint8_t cmd = CMD_READ_TEMP;
            if (HAL_I2C_Mem_Write(hi2c_, BMP180_ADDR, REG_CONTROL, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10) == HAL_OK) {
                lastCmdTick_ = now;
                state_ = STATE_WAIT_TEMP;
            }
            break;
        }
        case STATE_WAIT_TEMP: {
            if (now - lastCmdTick_ >= 5) {
                uint8_t buf[2];
                if (HAL_I2C_Mem_Read(hi2c_, BMP180_ADDR, REG_DATA, I2C_MEMADD_SIZE_8BIT, buf, 2, 10) == HAL_OK) {
                    rawTemp_ = static_cast<int32_t>((buf[0] << 8) | buf[1]);
                    uint8_t cmd = CMD_READ_PRESS | (mode_ << 6);
                    if (HAL_I2C_Mem_Write(hi2c_, BMP180_ADDR, REG_CONTROL, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10) == HAL_OK) {
                        lastCmdTick_ = now;
                        state_ = STATE_WAIT_PRESS;
                    } else state_ = STATE_IDLE;
                } else state_ = STATE_IDLE;
            }
            break;
        }
        case STATE_WAIT_PRESS: {
            uint32_t pressDelay = 26;
            switch (mode_) {
                case ULTRA_LOW_POWER: pressDelay = 5;  break;
                case STANDARD:        pressDelay = 8;  break;
                case HIGH_RES:        pressDelay = 14; break;
                case ULTRA_HIGH_RES:  pressDelay = 26; break;
            }
            if (now - lastCmdTick_ >= pressDelay) {
                uint8_t buf[3];
                if (HAL_I2C_Mem_Read(hi2c_, BMP180_ADDR, REG_DATA, I2C_MEMADD_SIZE_8BIT, buf, 3, 10) == HAL_OK) {
                    int32_t rawPress = (buf[0] << 16) | (buf[1] << 8) | buf[2];
                    rawPress >>= (8 - mode_);
                    CalculateTrueValues(rawTemp_, rawPress);

                    if (!PresionValida(presionPa_)) {
                        state_ = STATE_IDLE;
                        return false;
                    }

                    altitudM_ = CalculateAltitude(presionPa_, presionRefPa_);
                    state_ = STATE_IDLE;
                    return true;
                }
                state_ = STATE_IDLE;
            }
            break;
        }
    }
    return false;
}

void BMP180::CalculateTrueValues(int32_t ut, int32_t up) {
    int32_t x1 = ((ut - static_cast<int32_t>(AC6)) * static_cast<int32_t>(AC5)) >> 15;
    int32_t x2 = (static_cast<int32_t>(MC) << 11) / (x1 + MD);
    int32_t b5 = x1 + x2;
    temperaturaC_ = static_cast<float>((b5 + 8) >> 4) / 10.0f;

    int32_t b6 = b5 - 4000;
    x1 = (B2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (AC2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((static_cast<int32_t>(AC1) * 4 + x3) << mode_) + 2) / 4;
    x1 = (AC3 * b6) >> 13;
    x2 = (B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (static_cast<uint32_t>(AC4) * static_cast<uint32_t>(x3 + 32768)) >> 15;
    uint32_t b7 = (static_cast<uint32_t>(up) - b3) * (50000UL >> mode_);
    int32_t p = 0;
    if (b7 < 0x80000000) p = (b7 * 2) / b4;
    else p = (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    presionPa_ = static_cast<float>(p);
}

float BMP180::CalculateAltitude(float pressurePa, float refPressurePa) {
    if (refPressurePa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(pressurePa / refPressurePa, 0.19029495f));
}

void BMP180::CalibrateZero(uint8_t muestras) {
    // Directo de AP_Baro::calibrate() — el sensor lee mal el primer
    // segundo tras arrancar, dando ~1m de error si no se espera.
    HAL_Delay(10000);

    float sumaPresion = 0.0f;
    uint8_t obtenidas = 0;
    while (obtenidas < muestras) {
        if (Process()) {
            sumaPresion += presionPa_;
            obtenidas++;
        }
        HAL_Delay(1);
    }
    presionRefPa_ = sumaPresion / static_cast<float>(muestras);
    altitudM_ = 0.0f;
}
