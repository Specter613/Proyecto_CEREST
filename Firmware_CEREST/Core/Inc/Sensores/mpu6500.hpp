/*
 * mpu6500.hpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#ifndef INC_SENSORES_MPU6500_HPP_
#define INC_SENSORES_MPU6500_HPP_

#include <cstdint>
#include <cmath>
#include "stm32f4xx_hal.h"
#include <Proceso/sd.hpp>

#define MPU6500_ACCEL_ESCALA   4096.0f//4096.0f  // 4096 LSB/g para +/- 8g
#define MPU6500_GYRO_ESCALA    65.5f    // 65.5 LSB/(deg/s) para +/- 500 deg/s

struct MPU6500_Crudo{
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
};

struct MPU6500_Procesado{
	float ax, ay, az;
	float gx, gy, gz;
	float roll, pitch;
};

enum class PosicionAccel{
	NIVELADO = 0,  // +Z
	INVERTIDO,	   // -Z
	FRENTE,        // +X
	ATRAS,         // -X
	IZQUIERDA,     // +Y
	DERECHA        // -Y
};

struct AccelPosDato{
	float x, y, z;
};

class MPU6500{
public:
	explicit MPU6500(SPI_HandleTypeDef *hspi);
	void Inicio();

	void AsignarOffsetsGyro(float ox, float oy, float oz);
	void ObtenerOffsetsGyro(float &ox, float &oy, float &oz) const;
	void AsignarOffsetsAccel(float ox, float oy, float oz);
	void ObtenerOffsetsAccel(float &ox, float &oy, float &oz) const;
	void AsignarEscalasAccel(float sx, float sy, float sz);
	void AsignarMatrizAccel(float m00,float m01,float m02, float m10,float m11,float m12, float m20,float m21,float m22);
    void ObtenerMatrizAccel(float m[3][3]) const;

	    // Rotación de nivel (trim): se aplica a acelerómetro y giroscopio en LeeProcesado()
    void AsignarTrim(const float r[9]);
    void ObtenerTrim(float r[3][3]) const;

	//Calibraciones Acelerometro
	bool CapturarPosicionAccel(PosicionAccel pos, AccelPosDato &PosDato, uint16_t muestras);
	bool Calibracion6Posiciones(const AccelPosDato pos[6]);
	bool CalibracionSimplexNivel(uint16_t muestras);
	bool CalibracionMinCuadrados(const float muestrasX[], const float muestrasY[], const float muestrasZ[], uint16_t contador);

	// Calibracion Giroscopio
	bool CalibracionGyro(uint16_t muestras);

	//Accede a la memoria SD
	bool GuardarCalibracion();
	bool CargarCalibracion();

	//Leer datos
	void LeeCrudo();
	void LeeProcesado();

	//Accede a los datos
	const MPU6500_Procesado &ObtenerProcesado() const { return procesado_; }
	const MPU6500_Crudo &ObtenerCrudo() const { return crudo_; }

private:

	SPI_HandleTypeDef *hspi_;
	MPU6500_Crudo crudo_ = {};
	MPU6500_Procesado procesado_ = {};

    float gyroOffset_[3]  = {0.0f, 0.0f, 0.0f};
    float accelOffset_[3] = {0.0f, 0.0f, 0.0f};
    float accelScale_[3] = {1.0f, 1.0f, 1.0f};
	float matrizAccel_[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	    // Rotación de nivel (identidad = sin corrección)
    float rotNivel_[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

};


#endif /* INC_SENSORES_MPU6500_HPP_ */
