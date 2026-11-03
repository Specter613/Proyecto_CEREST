/*
 * qmc5883p.hpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#ifndef INC_SENSORES_QMC5883P_HPP_
#define INC_SENSORES_QMC5883P_HPP_


#include <cstdint>
#include <cmath>
#include "main.h"
#include "mpu6500.hpp"   // necesita MPU6500_Data para tilt-compensation
#include <Proceso/sd.hpp>

struct QMC5883Crudo{
    int16_t mx = 0, my = 0, mz = 0;
};

struct QMC5883Procesado{
    float mx = 0, my = 0, mz = 0;
    float heading = 0;
};

class QMC5883P{
public:
    explicit QMC5883P(I2C_HandleTypeDef *hi2c);

    void Inicio();
    bool LeeCrudo();                              // true si la lectura fue válida
    bool LeeProcesado();        // lee + calibra + tilt-compensa + heading

    // Calcula heading con tilt-compensation, a partir de roll/pitch externos.
    // Se llama DESPUÉS de Update(), usando los datos ya leídos.
    void NorteMagnetico(float roll_deg, float pitch_deg);

    const QMC5883Procesado &ObtenerProcesado() const { return procesado_; }
    const QMC5883Crudo  &ObtenerCrudo()  const { return crudo_; }

    // Calibración: solo la matemática, sin I/O de USB ni flash
    void AsignarOffsets(float ox, float oy, float oz);
    void ObtenerOffsets(float &ox, float &oy, float &oz) const;
    bool CalibracionMinimosCuadrados(const float muestrasX[], const float muestrasY[], const float muestrasZ[], uint16_t count);
    float ObtenerCalidadAjuste() const { return calidadAjuste_; }   // desviación relativa — indica si hay soft-iron
    // Muestreo crudo para calibración (min/max), usado por una capa externa
    // que decide cuándo mostrar progreso / guardar en flash
    void Calibracion(int16_t &min_x, int16_t &max_x,
                     int16_t &min_y, int16_t &max_y,
                     int16_t &min_z, int16_t &max_z);
    float YawCrudoAtan2(float roll_deg, float pitch_deg) const;

    bool GuardarCalibracion();
    bool CargarCalibracion();
    // BORRA estas 3 líneas, ya no se usan:
    // float scaleX_ = 1.0f, scaleY_ = 1.0f, scaleZ_ = 1.0f;

    // Confirma que ya tengas esto (de la respuesta anterior):
    // float matrizCorreccion_[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    void SetMatrizCorreccion(float m00,float m01,float m02, float m10,float m11,float m12, float m20,float m21,float m22);
    void ObtenerMatrizCorreccion(float m[3][3]) const;

private:
    I2C_HandleTypeDef *hi2c_;
    QMC5883Crudo  crudo_  = {};
    QMC5883Procesado procesado_ = {};

    float offsetX_ = 0.0f, offsetY_ = 0.0f, offsetZ_ = 0.0f;
    float scaleX_ = 1.0f, scaleY_ = 1.0f, scaleZ_ = 1.0f;   // soft-iron, reservado a futuro


    float calidadAjuste_ = 0.0f;
    float matrizCorreccion_[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    static constexpr float SCALA = 1000.0f;   // QMC5883P ±8 Gauss aprox
    static constexpr uint16_t QMC_ADDR  = (0x2C << 1);
    static constexpr uint16_t MPU_ADDR  = (0x68 << 1);   // registros compartidos I2C del breakout
    static constexpr float DeclinacionMagnetica = 3.5f;  // offset de montaje, calibrado empíricamente
};


#endif /* INC_SENSORES_QMC5883P_HPP_ */
