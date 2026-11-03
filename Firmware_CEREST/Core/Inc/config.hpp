/*
 * config.hpp — Configuración central de hardware habilitado
 *
 *  Un solo lugar para decidir qué hardware está físicamente presente
 *  en esta sesión de pruebas/vuelo. Cambiar un flag aquí NUNCA requiere
 *  tocar los constructores de Telemetria_USB/TelemetriaMavlink.
 */
#ifndef INC_CONFIG_HPP_
#define INC_CONFIG_HPP_

// ===== SENSORES =====
#define HABILITAR_MPU6500     1
#define HABILITAR_QMC5883     1
#define HABILITAR_BMP180      0   // no conectado durante esta depuración
#define HABILITAR_MTF01       1
#define HABILITAR_GPS         1
#define HABILITAR_BATERIA     1
#define HABILITAR_BUZZER      0
#define HABILITAR_VL53L0X     0

// ===== CONTROL =====
#define HABILITAR_DEBUG_PID   0 
#define DEBUG_ACTUALIZAR_PID   0
#define PRUEBA_MEZCLADOR_SIN_PID 0 // 1 = mezclador puro con RC, 0 = cascada normal con PID
#define MODO_SOLO_ANGULO      1    // 1 = solo bucle de ángulo, sin bucle de velocidad
#define DEPURAR_LOOP 0
#define MANTENER_POSICION 1 // Mantener la posicion actual 1 -> Activo 0->Inactivo

// ===== ACTUADORES =====
#define CALIBRACION_ESC       0

#define VISUALIZAR_FILTRO 0 // 1 -> Activo 0->Inactivo
#define VISUALIZAR_MTF02P 0 // 1 -> Activo 0->Inactivo    
 


// ===== RADIO (mutuamente excluyente — solo UNO puede ser FuenteRc) =====
#define USAR_MAVLINK_PARA_RC  1   // 1=MAVLink, 0=CRSF puro
#define INICIAR_TELEM_MAVLINK 1

// ===== TELEMETRÍA =====
//#define HABILITAR_TELEMETRIA_MAVLINK 1
#define MAVLINK_MODO_TRANSMISION     0   // 0=UART, 1=SPI/LoRa

#endif

