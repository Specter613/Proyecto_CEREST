#include "Radio/protocoloRadio.hpp"


#if !USAR_MAVLINK_PARA_RC

void RcProtocoloCrsf::Init()
{
	crsf_.Init(&radioRaw_);
}

void RcProtocoloCrsf::Process()
{
    crsf_.Process();   // usa TU lógica ya probada, sin tocarla
    for(int i = 0; i < 16; i++) datos_.ch[i] = CrsfToPwm(radioRaw_.ch[i]);

    datos_.signalOk = (radioRaw_.status == CRSF_SIGNAL_OK);
    datos_.canalesActivos = radioRaw_.activeChannels;

    datos_.rssiDbm = radioRaw_.rssi1;
    datos_.linkQuality = radioRaw_.linkQuality;
}

#else  // USAR_MAVLINK_PARA_RC

RcProtocoloMavlink* RcProtocoloMavlink::activeInstance_ = nullptr;

void RcProtocoloMavlink::Init()
{
    activeInstance_ = this;
    huart_->Init.BaudRate = 460800;
    HAL_UART_Init(huart_);

    // ENLAZAS LA UART DIRECTA A LA TELEMETRÍA
    if (telemetriaMavlink_ != nullptr) {
        telemetriaMavlink_->Init();
    }

    UartDispatch::Registrar(huart_, RxFullHandlerTrampoline);
    UartDispatch::RegistrarMitad(huart_, RxHalfHandlerTrampoline);
    UartDispatch::RegistrarError(huart_, RxErrorHandlerTrampoline);
    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}
void RcProtocoloMavlink::ManejarError()
{
    HAL_UART_DMAStop(huart_);
    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}

void RcProtocoloMavlink::Process()
{
    datos_.signalOk = (HAL_GetTick() - lastUpdateTick_) < 500;
}

void RcProtocoloMavlink::RxFullHandlerTrampoline(UART_HandleTypeDef *huart)
{
    if(activeInstance_ != nullptr) activeInstance_->RxFullHandler();
}

void RcProtocoloMavlink::RxHalfHandlerTrampoline(UART_HandleTypeDef *huart)
{
     if(activeInstance_ != nullptr) activeInstance_->RxHalfHandler();
}

void RcProtocoloMavlink::RxErrorHandlerTrampoline(UART_HandleTypeDef *huart)
{
     if(activeInstance_ != nullptr) activeInstance_->RxErrorHandler();
}

  // --- FUNCIÓN CLAVE DE RECUPERACIÓN ---
void RcProtocoloMavlink::RxErrorHandler()
{
    __HAL_UART_CLEAR_PEFLAG(huart_);
    __HAL_UART_CLEAR_FEFLAG(huart_);
    __HAL_UART_CLEAR_NEFLAG(huart_);
    __HAL_UART_CLEAR_OREFLAG(huart_);

    huart_->ErrorCode = HAL_UART_ERROR_NONE;
    huart_->RxState = HAL_UART_STATE_READY;

    HAL_UART_Receive_DMA(huart_, rxBufDMA_, kSizeBuff);
}

void RcProtocoloMavlink::RxFullHandler()
{
     for(int i = kSizeBuff / 2; i < kSizeBuff; i++) ParseByte(rxBufDMA_[i]);
}

void RcProtocoloMavlink::RxHalfHandler()
{
     for(int i = 0; i < kSizeBuff / 2; i++) ParseByte(rxBufDMA_[i]);
}

void RcProtocoloMavlink::ParseByte(uint8_t byte)
{
    // 2. Parser de MAVLink
	if (mavlink_parse_char(MAVLINK_COMM_1, byte, &rxMsg_, &rxStatus_))
	{
          // 3. Capturar mensaje de canales RC (ID 70)
		if (rxMsg_.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE)
		{
            mavlink_rc_channels_override_t ov;
            mavlink_msg_rc_channels_override_decode(&rxMsg_, &ov);

            datos_.ch[0]  = ov.chan1_raw;  datos_.ch[1]  = ov.chan2_raw;
            datos_.ch[2]  = ov.chan3_raw;  datos_.ch[3]  = ov.chan4_raw;
            datos_.ch[4]  = ov.chan5_raw;  datos_.ch[5]  = ov.chan6_raw;
            datos_.ch[6]  = ov.chan7_raw;  datos_.ch[7]  = ov.chan8_raw;
            datos_.ch[8]  = ov.chan9_raw;  datos_.ch[9]  = ov.chan10_raw;
            datos_.ch[10] = ov.chan11_raw; datos_.ch[11] = ov.chan12_raw;
            datos_.ch[12] = ov.chan13_raw; datos_.ch[13] = ov.chan14_raw;
            datos_.ch[14] = ov.chan15_raw; datos_.ch[15] = ov.chan16_raw;

            datos_.canalesActivos = 16;// Modificar despues

            lastUpdateTick_ = HAL_GetTick();
		}
        else if(rxMsg_.msgid == MAVLINK_MSG_ID_RADIO_STATUS)
        {
             mavlink_radio_status_t radio;
             mavlink_msg_radio_status_decode(&rxMsg_, &radio);

             // En MAVLink/ELRS, radio.rssi mapea la potencia recibida.
             // Convertimos el rango MAVLink (0-254) a porcentaje de Link Quality (0-100%)
             datos_.linkQuality = static_cast<uint8_t>((radio.rssi * 100) / 254);

             // Estimación aproximada de RSSI en dBm según escala estándar ELRS MAVLink:
             // (254 = ~ -50 dBm, 0 = ~ -120 dBm)
             datos_.rssiDbm = -120 + static_cast<int16_t>((radio.rssi * 70) / 254);
        }
        else
        {
        	// 4. Cualquier otro mensaje MAVLink (Peticiones GCS, Handshakes, etc.)
        	// se transfiere a la instancia de TelemetriaMavlink
        	if (telemetriaMavlink_ != nullptr)
        	{
        	   telemetriaMavlink_->OnMavlinkMessageReceived(rxMsg_);
        	}
        }
	}
}

#endif