#ifndef INC_RADIO_PROTCOLORADIO_HPP_
#define INC_RADIO_PROTCOLORADIO_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "config.hpp"



struct RcDatos{
	uint16_t ch[16] = {}; //Si actua mal ponerlo en ch[16] =
	bool signalOk = false;
	uint8_t canalesActivos = 0;

	int16_t rssiDbm = 0; //Potencia Recibida
	uint8_t linkQuality = 0; // en porcentaje 0-100
};

class RcProtocolo{
public:
	virtual ~RcProtocolo() = default;
	virtual void Init() = 0;
	virtual void  Process() = 0;
	const RcDatos& GetData() const { return datos_; }

protected:
	RcDatos datos_;
};

#if !USAR_MAVLINK_PARA_RC

#include "Radio/crsf.hpp"

class RcProtocoloCrsf : public RcProtocolo {
public:
	explicit RcProtocoloCrsf(UART_HandleTypeDef *huart) : crsf_(huart) {}

	void Init() override;

    void Process() override;

private:

    // Conversión matemática rápida de rango CRSF (174-1811) a us (988-2012)
     static inline uint16_t CrsfToPwm(uint16_t crsfVal) {
    	 if (crsfVal == 0) return 1000;
    	 return static_cast<uint16_t>(((uint32_t)crsfVal * 5) / 8 + 880); // Va de 988 a 2011 mas adelante limitar de 1000-2000
     }
    CRSF crsf_;
    CRSF_Radio radioRaw_ = {};
};

using FuenteRc = RcProtocoloCrsf;

#else  // USAR_MAVLINK_PARA_RC

extern "C" {
#include "common/mavlink.h"
}
#include "Proceso/uart_dispatch.hpp"
#include "Telemetria/telemetriaMAVlink.hpp"

// Forward declaration para evitar inclusiones circulares en el header
class TelemetriaMavlink;

class RcProtocoloMavlink : public RcProtocolo{
public:

    explicit RcProtocoloMavlink(UART_HandleTypeDef *huart) : huart_(huart) {}

    void Init() override;

    void ManejarError();

    void Process() override;

    // Método para vincular la telemetría desde App_Init()
    void SetTelemetriaMavlink(TelemetriaMavlink *ptr) {
         telemetriaMavlink_ = ptr;
    }
private:
    static void RxFullHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxHalfHandlerTrampoline(UART_HandleTypeDef *huart);
    static void RxErrorHandlerTrampoline(UART_HandleTypeDef *huart);
    // --- FUNCIÓN CLAVE DE RECUPERACIÓN ---
    void RxErrorHandler();
    void RxFullHandler();
    void RxHalfHandler();

    void ParseByte(uint8_t byte);

    static constexpr uint16_t kSizeBuff = 64;

    UART_HandleTypeDef *huart_;
    uint8_t rxBufDMA_[kSizeBuff] = {};
    mavlink_message_t rxMsg_ = {};
    mavlink_status_t rxStatus_ = {};
    uint32_t lastUpdateTick_ = 0;

    static RcProtocoloMavlink *activeInstance_;

    TelemetriaMavlink *telemetriaMavlink_ = nullptr;

};

using FuenteRc = RcProtocoloMavlink;

// #if USAR_MAVLINK_PARA_RC
// using FuenteRc = RcProtocoloMavlink;
// #else
// using FuenteRc = RcProtocoloCrsf;
#endif

#endif 