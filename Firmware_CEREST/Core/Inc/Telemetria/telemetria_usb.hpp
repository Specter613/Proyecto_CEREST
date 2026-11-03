/*
 * telemetria_usb.hpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#ifndef INC_TELEMETRIA_TELEMETRIA_USB_HPP_
#define INC_TELEMETRIA_TELEMETRIA_USB_HPP_


#include <cstdint>
#include <cstring>
#include "Sensores/mpu6500.hpp"
#include "Sensores/qmc5883p.hpp"
#include "Sensores/mtf02.hpp"
#include "Sensores/gnss.hpp"
#include "Sensores/bmp180.hpp"
#include "Control/mahony.hpp"
#include "Control/ekf_Posicion.hpp"
#include "Radio/protocoloRadio.hpp"
#include "puente.h"
#include "Proceso/planificador.hpp"
#include "Actuadores/motores.hpp"
#include <cstdlib>

enum class Telemetria_USB_Modo : uint8_t
{
	Desactivado = 0,
			Gyro,
			Mag,
			Flow,
			Gps,
			Baro,
			Elrs,
			Ekf,
			StreamMag,
			StreamAccel,
			_count
};

struct Muestreo
{
	uint32_t tick = 0;
	uint32_t muestreo = 0;
};

class Telemetria_USB
{
public:
	//Telemetria_USB(MPU6500 &imu, QMC5883P &mag, MTF01 &flow, Gnss &gps , FuenteRc &elrs, Scheduler &scheduler);

		Telemetria_USB(MPU6500 &imu, QMC5883P &mag, BMP180 &baro, MTF01 &flow, Gnss &gps, RcProtocolo &elrs, MahonyOrientacion &orientacion,
					   EkfPosicion &ekfPosicion, Planificador &planificador, MotorController &motores);

	void Comando(char *cmd);
	void Update(); //Actualiza
	void Proceso_Calibracion(); //Para la calibracion de la IMU, Mag y Baro

	void AsignarModoApagado() { modo_ = Telemetria_USB_Modo::Desactivado; }
	void ObtenerMuestreo (Telemetria_USB_Modo modo, uint32_t muestreo);

	 static int ParsearFloats(char *ptr, float *out, int max_cant);
    void ProcesarComandoPC(char *cmd);   // renombrado, más claro que "linea completa"
private:
	void ImprimirGyro();
	void ImprimirMag();
	void ImprimirGps();
	void ImprimirFlow();
	void ImprimirBaro();
	void ImprimirEkf();
	void ImprimirElrs();
	void CalibrarGyroSolo();
	void ImprimirStreamMag();
	void ImprimirStreamAccel();
	// void CalibrarIMU();
	// void CalibrarAccelMultiPos();
	// void RunMagCalibracion();

	void CalStatus();
	void PlanificadorStatus();

	MPU6500 &imu_;
	QMC5883P &mag_;
	BMP180 &baro_;
	MTF01 &flow_;
	Gnss &gps_;
	RcProtocolo &elrs_;
	MahonyOrientacion &orientacion_;
	EkfPosicion &ekfPosicion_;
	Planificador &planificador_;
	MotorController &motores_;
	Telemetria_USB_Modo modo_ = Telemetria_USB_Modo::Desactivado; //DEFAULT

	//volatile bool solicitudCalMag_ = false;
	volatile bool solicitudCalEsc_ = false;
	volatile bool solicitudCalGyro_ = false;

	Muestreo muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::_count)];
};

#endif /* INC_TELEMETRIA_TELEMETRIA_USB_HPP_ */
