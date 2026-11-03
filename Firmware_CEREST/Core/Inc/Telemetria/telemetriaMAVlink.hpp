/*
 * telemetriaMAVlink.hpp
 *
 *  Created on: 8 ago 2026
 *      Author: specter0163
 *
 +------------------------+
 |   TelemetriaMavlink    |  (Genera y parsea paquetes MAVLink)
 +------------------------+
             |
             v  [Bytes MAVLink listos]
   ( Callback de Salida )
             |
       +-----+-------------------------+
       |                               |
       v                               v
[ Módulo ELRS (UART) ]    [ Módulo LoRa SX1276 (SPI) ]
 */

#ifndef INC_TELEMETRIA_TELEMETRIAMAVLINK_HPP_
#define INC_TELEMETRIA_TELEMETRIAMAVLINK_HPP_

#include <cstdint>
#include <cstring>
#include "Control/pid.hpp"

// Silenciar warnings de MAVLink C
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

#include "Sensores/mpu6500.hpp"
#include "Sensores/qmc5883p.hpp"
#include "Sensores/bmp180.hpp"
#include "Sensores/gnss.hpp"
#include "Proceso/SerialTx.hpp"
#include "Control/mahony.hpp"
#include "Control/ekf_Posicion.hpp"

// Callback de transmisión genérico (UART, LoRa, USB, etc.)
using MavlinkTxCallback = bool (*)(const uint8_t *data, uint16_t len, void *userCtx);

class TelemetriaMavlink {
public:
	TelemetriaMavlink(MPU6500 &imu, QMC5883P &mag, BMP180 &baro, MahonyOrientacion &mahony,GananciasPID &pid, EkfPosicion &ekf, const Gnss &gps);

	void Init();
	void Update(); // Se ejecuta de forma directa en App_Loop()

	// Configuración del puerto/transporte de salida
	void SetTxCallback(MavlinkTxCallback callback, void *userCtx = nullptr);

	// Recepción de datos (Tierra -> Dron)
	void ProcessRxByte(uint8_t byte);
	void OnMavlinkMessageReceived(const mavlink_message_t &msg);

	// Posición del estimador: x, y en m (mundo), z = altura en m (positiva)
	void SetPosicion(float x, float y, float z) { posX_ = x; posY_ = y; posZ_ = z; }
	// Batería (la llama la tarea que lee el módulo de potencia):
	//   voltaje en V, corriente en A, restante en % (0..100), consumido en mAh.
	//   Negativo = desconocido.
	void SetBateria(float voltaje, float corriente, int8_t restante = -1, float consumidoMah = -1.0f) {
		batVoltaje_ = voltaje; batCorriente_ = corriente; batRestante_ = restante; batMah_ = consumidoMah;
	}


	// Control de frecuencias dinámicas (en Hz)
	void SetImuHz(uint16_t hz);
	void SetHeartbeatHz(uint16_t hz);
	void SetVfrHudHz(uint16_t hz);
	void SetBateriaHz(uint16_t hz);
	void SetBateriaStatusHz(uint16_t hz);
	void SetPosicionHz(uint16_t hz);
	void SetGpsHz(uint16_t hz);       // GPS_RAW_INT (stream EXTENDED_STATUS)

	uint32_t ContadorCambios() const { return contadorCambios_; }

private:
	uint32_t contadorCambios_ = 0;
	uint32_t lastPidSendTick_ = 0;

	// --- Control de Tiempos e Intervalos (Milisegundos) ---
	// Todo lo que NO sea heartbeat arranca en 0 (apagado) — solo se
	// activa cuando el GCS lo pide explícitamente vía REQUEST_DATA_STREAM
	// o SET_MESSAGE_INTERVAL. Evita saturar el enlace desde el arranque.
	struct ConfigIntervalos {
		uint32_t imuMs       = 0;    // Apagado por defecto
		uint32_t heartbeatMs = 1000; // 1 Hz — siempre activo, es el "estoy vivo"
		uint32_t bateriaMs   = 500;  // 2 Hz — siempre activo, información de seguridad
		uint32_t bateriaStatusMs = 1000; // 1 Hz — siempre activo (BATTERY_STATUS, mAh)	
		uint32_t vfrHudMs    = 0;    // Apagado por defecto
		uint32_t posicionMs  = 0;    // Apagado por defecto
		uint32_t gpsMs       = 0;    // Apagado por defecto
	} intervalos_;

	struct UltimoTick {
		uint32_t imu       = 0;
		uint32_t heartbeat = 0;
		uint32_t vfrHud    = 0;
		uint32_t posicion  = 0;
		uint32_t gps       = 0;
		uint32_t bateria   = 0;
		uint32_t bateriaStatus = 0;
		
	} ticks_;

	// --- Gestión de envío de PIDs por ráfaga ---
	uint8_t pendingPidIndex_ = 0;
	bool sendingPidList_     = false;

	// --- Posición del estimador (la pasa puente.cpp con SetPosicion) ---
	float posX_ = 0.0f, posY_ = 0.0f, posZ_ = 0.0f;

	// --- Batería (la pasa la tarea del módulo de potencia con SetBateria) ---
	float  batVoltaje_   = -1.0f;
	float  batCorriente_ = -1.0f;
	int8_t batRestante_  = -1;
	float  batMah_       = -1.0f;

	// --- Referencias a Hardware y Datos ---
	MPU6500 &imu_;
	QMC5883P &mag_;
	BMP180 &baro_;
	MahonyOrientacion &mahony_;
	GananciasPID &pid_;
	EkfPosicion &ekf_;
	const Gnss &gps_;

	// --- Callback de Salida ---
	MavlinkTxCallback txCallback_ = nullptr;
	void *txCtx_                  = nullptr;

	// --- Identificadores MAVLink ---
	uint8_t systemId_    = 1;
	uint8_t componentId_ = MAV_COMP_ID_AUTOPILOT1;

	// --- Tabla de Parámetros ---
	struct MavParam {
		char id[16];
		float *valuePtr;
	};

	static constexpr uint16_t NUM_PARAMS = 15;
	MavParam paramTable_[NUM_PARAMS];

	// --- Empaquetadores internos de mensajes ---
	void SendHeartbeat();
	void SendAttitude();
	void SendVfrHud();
	void SendPosicion();
	bool SendParamValue(uint16_t index);
	void SendGpsRaw();
	void SendSysStatus();
	void SendBatteryStatus();

	// Envió físico final usando el callback        
	bool SendMessage(mavlink_message_t *msg);
};

#endif /* INC_TELEMETRIA_TELEMETRIAMAVLINK_HPP_ */