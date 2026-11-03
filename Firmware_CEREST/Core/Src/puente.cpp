#include "config.hpp"
#include "puente.h"
//Sensores
#include "Sensores/mpu6500.hpp"
#include "Sensores/qmc5883p.hpp"
#include "Sensores/bmp180.hpp"
#include "Sensores/mtf02.hpp"
#include "Sensores/bateria.hpp"
#include "Sensores/buzzer.hpp"
//Planificador
#include "Proceso/planificador.hpp"
//Telemetria
#include "Telemetria/telemetria_usb.hpp"
#include "Telemetria/telemetriaMAVlink.hpp"
//Radio
#include "Radio/protocoloRadio.hpp"
//Control
#include "Control/mahony.hpp"
#include "Control/ekf_Posicion.hpp"
#include "Control/pid.hpp"
// Actuadores
#include "Actuadores/motores.hpp"
#include "Actuadores/servo.hpp"
//Proceso
#include "Proceso/SerialTx.hpp"
#include "usbd_cdc_if.h"
#include <cstdlib> // Requerido para strtof

// Instancias de Hardware
extern SPI_HandleTypeDef hspi2;
static MPU6500 imu(&hspi2);

extern I2C_HandleTypeDef hi2c1;
static QMC5883P mag(&hi2c1);
static BMP180 baro(&hi2c1, BMP180::ULTRA_HIGH_RES);

extern UART_HandleTypeDef huart2;
static FuenteRc elrs(&huart2);

extern UART_HandleTypeDef huart6;
static MTF01 flow(&huart6);

extern UART_HandleTypeDef huart4;
static Gnss gps(&huart4);

extern TIM_HandleTypeDef htim3;
static MotorController motores(&htim3);

extern TIM_HandleTypeDef htim2;
static Servo servo(&htim2, TIM_CHANNEL_2);

extern ADC_HandleTypeDef hadc1;
static Bateria bateria(&hadc1, ADC_CHANNEL_11, ADC_CHANNEL_13);   // PC1 = voltaje, PC3 = corriente

static Buzzer buzzer(GPIOC, GPIO_PIN_2);   // PC2 -> resistencia -> base 2N3904

static uint8_t almacenTxMavlink[512];
static SerialTx mavlinkTx(&huart2, almacenTxMavlink, sizeof(almacenTxMavlink));

// Memoria Compartida
static Planificador planificador;


//Filtro
static MahonyOrientacion orientacion(2.5f, 0.01f); // Kp , Ki 
static EkfPosicion ekfPosicion;

static GananciasPID pid;
static ControladorPID pidRollAngulo(-250,250);
static ControladorPID pidPitchAngulo(-250,250);
static ControladorPID pidYawVelocidad(-300,300);
static ControladorPID pidRollVelocidad(-300,300);
static ControladorPID pidPitchVelocidad(-300,300);

// Telemetrías
static Telemetria_USB telemetria_usb(imu, mag, baro, flow, gps, elrs, orientacion , ekfPosicion, planificador, motores);
static TelemetriaMavlink telemetriaMavlink(imu, mag, baro, orientacion, pid, ekfPosicion, gps);

static constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;

// ============================================================================
// FUNCIONES DE APOYO
// ============================================================================
template<typename T>
static inline T Clamp(T val, T minVal, T maxVal)
{
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}
/**
 * @brief Aplica una ventana de tolerancia alrededor del punto neutro (1500us)
 *        para ignorar holguras mecánicas y ruido eléctrico de las palancas.
 */
static inline float AplicarZonaMuerta(float PWM, float centro = 1500.0f, float ventana = 12.0f)
{
    float diff = PWM - centro;
    // Si la lectura está dentro del rango +/- 12us del centro, se fuerza a 1500.0f exactos
    if (std::abs(diff) < ventana) return centro;
    return PWM;
}

// en config.hpp o al inicio de puente.cpp
static constexpr uint32_t MS(uint32_t ms) { return ms * 1000; }
// ============================================================================
// FUNCIONES CALLBACK DE TRANSMISIÓN MAVLINK (SE SEGREGAN SEGÚN EL MODO)
// ============================================================================

#if (MAVLINK_MODO_TRANSMISION == 0)
/**
 * @brief Transmisión MAVLink vía UART (Modo 0)
 */
static bool TransmitirMavlinkUart(const uint8_t *datos, uint16_t len, void *ctx)
{
    return mavlinkTx.Escribir(datos, len) == len;   // true solo si TODO el mensaje cupo
}

#elif (MAVLINK_MODO_TRANSMISION == 1)
/**
 * @brief Transmisión MAVLink vía SPI / LoRa (Modo 1 - En desarrollo)
 */
static void TransmitirMavlinkSpi(const uint8_t *datos, uint16_t len, void *ctx)
{
    // TODO: Cuando agregues la librería LoRa SPI, invocarás su envío aquí:
    // LoRaDriver *lora = static_cast<LoRaDriver*>(ctx);
    // lora->SendPacket(datos, len);
}
#endif

// ============================================================================
// TAREAS DEL SCHEDULER
// ============================================================================
static void TareaServo(void *ctx, uint32_t muestreo_us)
{
    const RcDatos &rc = elrs.GetData();
    if (!rc.signalOk) return;          // sin señal: el servo se queda donde estaba
    servo.SetDesdeCanal(rc.ch[9]);     // canal 10 del ELRS
}

#if HABILITAR_MPU6500
static void TareaGyro(void *ctx, uint32_t muestreo_us){

    imu.LeeProcesado();
    // AQUÍ SE DECLARA Y CALCULA EL dt DE INTEGRACIÓN DE LA IMU:
    // 'muestreo' viene del scheduler (3 ms). Dividido entre 1000 da dt = 0.003 segundos.
    float dt = muestreo_us / 1000000.0f;
    const MPU6500_Procesado &d = imu.ObtenerProcesado();

    orientacion.Update6DOF(d.gx * DEG2RAD, d.gy * DEG2RAD, d.gz * DEG2RAD,
                            d.ax, d.ay, d.az, dt);

    float roll, pitch, yaw;
    orientacion.GetEuler(roll, pitch, yaw);
    ekfPosicion.Predecir(roll, pitch, yaw, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, dt);

    telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Gyro, muestreo_us);
}
#endif

#if HABILITAR_QMC5883
static void TareaMag(void *ctx, uint32_t muestreo)
{
	mag.LeeProcesado();
    const QMC5883Procesado &m = mag.ObtenerProcesado();
    //orientacion.UpdateMag(-m.mx, -m.my, m.mz);//Inverdido
    orientacion.UpdateMag(m.mx, m.my, m.mz);//
    //mag.NorteMagnetico(imu.ObtenerProcesado().roll, imu.ObtenerProcesado().pitch);
    mag.YawCrudoAtan2(imu.ObtenerProcesado().roll, imu.ObtenerProcesado().pitch);
	telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Mag, muestreo);
}
#endif

static void ElrsTask(void *ctx, uint32_t muestreo)
{
    elrs.Process();
    telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Elrs, muestreo);
}     

#if HABILITAR_BMP180
static void BaroTask(void *ctx, uint32_t muestreo) {
    if (baro.Process()) {
        telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Baro, muestreo);
    }
}
#endif

#if HABILITAR_GPS
static void GpsTask(void *ctx, uint32_t deltaMs)
{
    gps.Update();
    telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Gps, deltaMs);
}
#endif

#if HABILITAR_MTF01
static void FlowTask(void *ctx, uint32_t muestreo)
{
    float dt = muestreo / 1000000.0f; // Convertir a segundos
    flow.ActualizarPosicion(dt);
    //const PosicionMTF &pos = flow.GetPosicion(); // pos.x e pos.y
    if (flow.ConsumirDatoNuevo()) {
        const MTF01_Data &f = flow.GetData();
        bool datoValido = (f.quality > 50) && (f.status == 1);
        ekfPosicion.CorregirFlujo(static_cast<float>(f.flowX), static_cast<float>(f.flowY),
                                static_cast<float>(f.distance), datoValido);
    }
    telemetria_usb.ObtenerMuestreo(Telemetria_USB_Modo::Flow, muestreo);
}
#endif

#if HABILITAR_BATERIA
static void TareaBateria(void *ctx, uint32_t muestreo_us)
{
    bateria.Update(muestreo_us / 1000000.0f);
    telemetriaMavlink.SetBateria(bateria.Voltaje(), bateria.Corriente(),
                                 bateria.Porcentaje(), bateria.mAh());
}
#endif

#if HABILITAR_BUZZER
// ====================================================================================
//  Buzzer: avanza la melodía y avisa cuando el GPS tiene más de 4 satélites en uso.
//  Si el GPS baja a 4 o menos durante más de 5 s, se vuelve a avisar al recuperarlos.
// ====================================================================================
static void TareaBuzzer(void *ctx, uint32_t muestreo_us)
{
    const uint32_t ahora = HAL_GetTick();

#if HABILITAR_GPS
    static bool     gpsAnunciado = false;
    static uint32_t tickSinSatelites = 0;

    const GnssData &g = gps.GetData();
    if (g.numSV > 4) {
        tickSinSatelites = ahora;
        if (!gpsAnunciado && !buzzer.Sonando()) {
            buzzer.Reproducir(kMelodiaGpsListo);
            gpsAnunciado = true;
        }
    } else if (gpsAnunciado && (ahora - tickSinSatelites) > 5000) {
        gpsAnunciado = false;   // perdió satélites un buen rato: volver a avisar
    }
#endif

    buzzer.Update(ahora);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL INICIO


#if MANTENER_POSICION
/**
 * @brief Tarea de control de vuelo, corre cada 4ms (250Hz) — coincide
 * con el refresco del PWM ya configurado a 250Hz. Toma la señal de
 * radio, la convierte a consignas de ángulo/velocidad, ejecuta la
 * cascada de PID y aplica el resultado a los motores.
 *
 * Con el switch SD (canal 8) activo y las palancas de roll/pitch centradas,
 * el modo de mantener posición calcula las consignas de ángulo en lugar
 * del mando, usando la posición y velocidad del estimador de flujo óptico.
 */

// ====================================================================================
//  Estadísticas del periodo de la tarea (para depuración)
// ====================================================================================
static float dtMin = 999.0f, dtMax = 0.0f, dtSuma = 0.0f;
static uint32_t contador = 0;

// ====================================================================================
//  Throttle
// ====================================================================================
static constexpr float kThrottleMax       = 1800.0f;  // µs: throttle máximo del piloto
static constexpr float kUmbralControlador = 1100.0f;  // µs: arriba de esto se activa el controlador

// ====================================================================================
//  MANTENER POSICIÓN (flujo óptico) — misma estructura que el Loiter/PosHold de ArduPilot:
//
//    posición ──(raíz cuadrada)──> velocidad deseada ──(PI con anti-windup)──> aceleración
//             ──(yaw: mundo -> cuerpo)──> ──(accel_to_lean_angles)──> consignas roll/pitch
//
//  Esas consignas entran a la cascada de siempre (P de ángulo -> PID de velocidad angular).
//  La altura NO se controla aquí: sigue siendo manual con el throttle.
// ====================================================================================
static constexpr uint8_t kCanalPosHold   = 7;      // rc.ch[7] = canal 8, switch SD (>1500 = activo)

// --- Etapa 1: posición -> velocidad deseada ---
static constexpr float   kKpPos          = 10.0f;   // 1/s: ganancia P cerca del punto (zona lineal)
static constexpr float   kDecMax         = 0.5f;   // m/s²: desaceleración máxima al llegar al punto
static constexpr float   kVelMax         = 0.1f;   // m/s: velocidad máxima para regresar al punto

// --- Etapa 2: velocidad -> aceleración deseada ---
static constexpr float   kKpVel          = 5.75f;   // 1/s: error de velocidad -> aceleración
static constexpr float   kKiVel          = 0.001f;   // 1/s²: integral del error (corrige viento/desbalance)
static constexpr float   kIntVelMax      = 1.0f;   // m/s: límite de la integral
static constexpr float   kAccMax         = 1.5f;   // m/s²: aceleración máxima pedida

// --- Etapa 3-4: aceleración -> ángulos ---
static constexpr float   kAnguloMaxHold  = 2.5f;  // °: inclinación máxima que puede pedir el modo
static constexpr float   kGravedad       = 9.81f;  // m/s²

// --- Condiciones ---
static constexpr float   kAlturaMinHold  = 0.15f;  // m: abajo de esto el flujo no es confiable

// --- Signos: verificados SIN HÉLICES (el dron regresa al punto) ---
static constexpr float   kSignoPitchHold = 1.0f;   // pitch para acelerar hacia el frente (+X del dron)
static constexpr float   kSignoRollHold  = -1.0f;  // roll para acelerar hacia la izquierda (+Y del dron)
static constexpr float   kSignoYawHold   = 1.0f;   // igual que kSignoYaw de ekf_posicion.cpp

// --- Estado del modo ---
static bool  holdActivo = false;                   // el modo ya tiene un punto objetivo guardado
static bool  reiniciarConsignasHold = true;        // reiniciar el suavizado en la próxima llamada
static float posObjX = 0.0f, posObjY = 0.0f;       // m: punto que se quiere mantener (mundo)
static float intVelX = 0.0f, intVelY = 0.0f;       // integral del error de velocidad (mundo)

// Limita un valor float entre lo y hi
static float LimitarF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/**
 * @brief Calcula las consignas de ángulo (°) para regresar y mantenerse en el punto objetivo.
 * @param yawDeg     yaw de Mahony (°), el mismo con el que el estimador gira la velocidad al mundo
 * @param dt         periodo de la tarea (s)
 * @param reiniciar  true en la primera llamada tras activar el modo: reinicia el suavizado
 * @param rollCmd    [salida] consigna de roll (°)
 * @param pitchCmd   [salida] consigna de pitch (°)
 */
static void ConsignasPosHold(float yawDeg, float dt, bool reiniciar, float &rollCmd, float &pitchCmd)
{
    static constexpr float kTauConsigna = 0.10f;   // s: suavizado de las consignas de salida
    static float rollFilt = 0.0f, pitchFilt = 0.0f;

    // Posición y velocidad actuales del estimador de flujo óptico (marco del mundo)
    const float px = ekfPosicion.PosX(), py = ekfPosicion.PosY();
    const float vx = ekfPosicion.VelX(), vy = ekfPosicion.VelY();

    // ETAPA 1: Posición -> velocidad deseada (controlador raíz cuadrada de ArduPilot)
    const float ex = posObjX - px;
    const float ey = posObjY - py;
    const float dist = std::sqrt(ex * ex + ey * ey);
    const float dLin = kDecMax / (kKpPos * kKpPos);

    float velDeseada;
    if (dist <= dLin) {
        velDeseada = kKpPos * dist;
    } else {
        velDeseada = std::sqrt(2.0f * kDecMax * (dist - 0.5f * dLin));
    }
    if (velDeseada > kVelMax) velDeseada = kVelMax;

    float vdx = 0.0f, vdy = 0.0f;
    if (dist > 1e-4f) {
        vdx = velDeseada * ex / dist;
        vdy = velDeseada * ey / dist;
    }

    // ETAPA 2: Velocidad -> aceleración deseada (PI con anti-windup por integración condicionada)
    const float evx = vdx - vx;
    const float evy = vdy - vy;

    // Integral tentativa
    const float intX = LimitarF(intVelX + evx * dt, -kIntVelMax, kIntVelMax);
    const float intY = LimitarF(intVelY + evy * dt, -kIntVelMax, kIntVelMax);
    const float axPrueba = kKpVel * evx + kKiVel * intX;
    const float ayPrueba = kKpVel * evy + kKiVel * intY;

    if (std::sqrt(axPrueba * axPrueba + ayPrueba * ayPrueba) <= kAccMax) {
        // Sin saturación: se integra normal
        intVelX = intX;
        intVelY = intY;
    } else {
        // Saturado: solo se integra en el eje donde el error ayuda a salir de la saturación
        if (evx * axPrueba < 0.0f) intVelX = intX;
        if (evy * ayPrueba < 0.0f) intVelY = intY;
    }

    float ax = kKpVel * evx + kKiVel * intVelX;
    float ay = kKpVel * evy + kKiVel * intVelY;

    const float a = std::sqrt(ax * ax + ay * ay);
    if (a > kAccMax) { ax *= kAccMax / a; ay *= kAccMax / a; }

    // ETAPA 3: Mundo -> cuerpo con el yaw
    const float yaw = kSignoYawHold * yawDeg * 0.01745329f;
    const float c = std::cos(yaw), s = std::sin(yaw);
    const float axCuerpo =  c * ax + s * ay;
    const float ayCuerpo = -s * ax + c * ay;

    // ETAPA 4: Aceleración -> ángulos (accel_to_lean_angles de ArduPilot):
    //   pitch = atan2(a_frente, g);  roll = atan2(a_lado · cos(pitch), g)
    //   El cos(pitch) toma en cuenta que, al inclinarse en pitch, queda menos empuje vertical.
    const float pitchRad = std::atan2(axCuerpo, kGravedad);
    const float rollRad  = std::atan2(ayCuerpo * std::cos(pitchRad), kGravedad);

    const float pitchCrudo = LimitarF(kSignoPitchHold * pitchRad * 57.29578f, -kAnguloMaxHold, kAnguloMaxHold);
    const float rollCrudo  = LimitarF(kSignoRollHold  * rollRad  * 57.29578f, -kAnguloMaxHold, kAnguloMaxHold);

    // ETAPA 5: Suavizado de las consignas (quita los saltos del ruido del flujo)
    if (reiniciar) {
        rollFilt = 0.0f;
        pitchFilt = 0.0f;
    }
    const float alfa = dt / (kTauConsigna + dt);
    rollFilt  += alfa * (rollCrudo  - rollFilt);
    pitchFilt += alfa * (pitchCrudo - pitchFilt);

    rollCmd  = rollFilt;
    pitchCmd = pitchFilt;
}

static void TareaMotor(void *ctx, uint32_t muestreo_us)
{
    // ==================================================================================
    //  1. Lectura del mando
    // ==================================================================================
    const RcDatos &rc = elrs.GetData();
    bool signalLost = !rc.signalOk;
    bool armed = (rc.ch[4] > 1500);                    // canal 5: armado

    uint16_t chThrottle = Clamp(rc.ch[0], (uint16_t)1000, (uint16_t)2000);
    uint16_t chRoll     = Clamp(rc.ch[1], (uint16_t)1000, (uint16_t)2000);
    uint16_t chPitch    = Clamp(rc.ch[2], (uint16_t)1000, (uint16_t)2000);
    uint16_t chYaw      = Clamp(rc.ch[3], (uint16_t)1000, (uint16_t)2000);

    float throttle_us = static_cast<float>(chThrottle);
    if (throttle_us > kThrottleMax) throttle_us = kThrottleMax;

    float dt = muestreo_us / 1000000.0f;

    // ==================================================================================
    //  2. Controlador apagado: desarmado, sin señal, o throttle debajo del umbral
    //     Se reinician los PID y el modo de posición (al volver, guarda un punto nuevo).
    // ==================================================================================
    if (!armed || signalLost || throttle_us < kUmbralControlador)
    {
        pidRollAngulo.Reiniciar();
        pidPitchAngulo.Reiniciar();
        pidYawVelocidad.Reiniciar();
        pidRollVelocidad.Reiniciar();
        pidPitchVelocidad.Reiniciar();
        holdActivo = false;
        motores.Actualizar(armed && !signalLost, signalLost, throttle_us, 0.0f, 0.0f, 0.0f);
        return;
    }

    float throttleOut = throttle_us;

#if PRUEBA_MEZCLADOR_SIN_PID
    // Prueba del mezclador: las palancas van directo a los motores, sin PID
    float rollOut  = (AplicarZonaMuerta(chRoll)  - 1500.0f);
    float pitchOut = (AplicarZonaMuerta(chPitch) - 1500.0f);
    float yawOut   = (AplicarZonaMuerta(chYaw)   - 1500.0f);
#else
    // ==================================================================================
    //  3. Consignas del piloto
    //     Roll/pitch: ángulo (°), ±10° a tope de palanca.  Yaw: velocidad (°/s).
    // ==================================================================================
    float ConsignarollAng  = 0.02f*(AplicarZonaMuerta(chRoll)  - 1500.0f);
    float ConsignapitchAng = 0.02f*(AplicarZonaMuerta(chPitch) - 1500.0f);
    float ConsignayawVel   = 0.20f*(AplicarZonaMuerta(chYaw)   - 1500.0f);

    // Orientación (Mahony) y giroscopio
    float rollFiltro, pitchFiltro, yawFiltro;
    orientacion.GetEuler(rollFiltro, pitchFiltro, yawFiltro);
    const MPU6500_Procesado &d = imu.ObtenerProcesado();

    // ==================================================================================
    //  4. Mantener posición (switch SD, canal 8)
    //
    //     - Al activar el switch: guarda la posición actual como objetivo.
    //     - Mientras el piloto mueve roll/pitch: manda el piloto y el objetivo sigue al dron.
    //     - Con las palancas centradas: el modo calcula las consignas de ángulo.
    //     - Debajo de kAlturaMinHold el modo no actúa (el flujo no es confiable).
    //     - Cada vez que se fija un objetivo nuevo, el suavizado se reinicia explícitamente.
    // ==================================================================================
    const bool switchHold  = (rc.ch[kCanalPosHold] > 1500);
    const bool alturaOk    = (ekfPosicion.AlturaUsada() >= kAlturaMinHold);
    const bool pilotoMueve = (ConsignarollAng != 0.0f) || (ConsignapitchAng != 0.0f);

    if (switchHold && alturaOk) {
        if (!holdActivo || pilotoMueve) {
            posObjX = ekfPosicion.PosX();
            posObjY = ekfPosicion.PosY();
            intVelX = 0.0f;
            intVelY = 0.0f;
            holdActivo = true;
            reiniciarConsignasHold = true;
        }
        if (!pilotoMueve) {
            ConsignasPosHold(yawFiltro, dt, reiniciarConsignasHold, ConsignarollAng, ConsignapitchAng);
            reiniciarConsignasHold = false;
        }
    } else {
        holdActivo = false;
        reiniciarConsignasHold = true;
    }

    // Medición de yaw tal como la usa el PID (convención de la palanca)
    const float yawVelMedida = -d.gz;

    // Estadísticas del periodo de la tarea
    if (dt > 0.0f) {
        if (dt < dtMin) dtMin = dt;
        if (dt > dtMax) dtMax = dt;
        dtSuma += dt;
        contador++;
    }

    // ==================================================================================
    //  5. Cascada de control
    //     Lazo exterior: ángulo (°) -> velocidad angular deseada (°/s)
    //     Lazo interior: velocidad angular (°/s) -> salida al mezclador (µs)
    //     Yaw: solo lazo de velocidad.
    // ==================================================================================
    float rollOutAngulo  = pidRollAngulo.ActualizarPID(ConsignarollAng,  rollFiltro,  dt,
        pid.kp_angulo_roll,  pid.ki_angulo_roll,  pid.kd_angulo_roll);

    float pitchOutAngulo = pidPitchAngulo.ActualizarPID(ConsignapitchAng, pitchFiltro, dt,
        pid.kp_angulo_pitch, pid.ki_angulo_pitch, pid.kd_angulo_pitch);

    float rollOut  = pidRollVelocidad.ActualizarPID(rollOutAngulo,  d.gx, dt,
        pid.kp_velocidad_roll,  pid.ki_velocidad_roll,  pid.kd_velocidad_roll);

    float pitchOut = pidPitchVelocidad.ActualizarPID(pitchOutAngulo, d.gy, dt,
        pid.kp_velocidad_pitch, pid.ki_velocidad_pitch, pid.kd_velocidad_pitch);

    float yawOut   = pidYawVelocidad.ActualizarPID(ConsignayawVel, yawVelMedida, dt,
        pid.kp_velocidad_yaw, pid.ki_velocidad_yaw, pid.kd_velocidad_yaw);

#if DEBUG_ACTUALIZAR_PID
    // ==================================================================================
    //  Depuración: línea "PID,..." para capturar_pid_csv.py (cada 4 ciclos = 62.5 Hz)
    //  El buffer es static: CDC_Transmit_FS envía desde esta memoria DESPUÉS de regresar.
    // ==================================================================================
    static uint8_t div = 0;
    if (++div >= 4) {
        div = 0;
        static char dbg[320];
        int dl = snprintf(dbg, sizeof(dbg),
            "PID,%lu,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
            (unsigned long)HAL_GetTick(),
            ConsignarollAng, rollFiltro, ConsignarollAng - rollFiltro,
            pidRollAngulo.ObtenerTerminoP(), pidRollAngulo.ObtenerTerminoI(), pidRollAngulo.ObtenerTerminoD(), rollOut,
            ConsignapitchAng, pitchFiltro, ConsignapitchAng - pitchFiltro,
            pidPitchAngulo.ObtenerTerminoP(), pidPitchAngulo.ObtenerTerminoI(), pidPitchAngulo.ObtenerTerminoD(), pitchOut,
            ConsignayawVel, yawVelMedida, ConsignayawVel - yawVelMedida,
            pidYawVelocidad.ObtenerTerminoP(), pidYawVelocidad.ObtenerTerminoI(), pidYawVelocidad.ObtenerTerminoD(), yawOut);
        if (dl > 0) CDC_Transmit_FS((uint8_t*)dbg, (uint16_t)dl);
    }
#endif
#endif

    // ==================================================================================
    //  6. Mezclador -> ESC
    // ==================================================================================
    motores.Actualizar(armed, signalLost, throttleOut, rollOut, pitchOut, yawOut);
}

#else
/**
 * @brief Tarea de control de vuelo, corre cada 4ms (250Hz) — coincide
 * con el refresco del PWM ya configurado a 250Hz. Toma la señal de
 * radio, la convierte a consignas de ángulo/velocidad, ejecuta la
 * cascada de PID y aplica el resultado a los motores.
 */
static float dtMin = 999.0f, dtMax = 0.0f, dtSuma = 0.0f;
static uint32_t contador = 0;

static constexpr float kThrottleMax       = 1800.0f;  // µs: throttle máximo del piloto
static constexpr float kUmbralControlador = 1300.0f;  // µs: arriba de esto se activa el controlador

static void TareaMotor(void *ctx, uint32_t muestreo_us)
{
    const RcDatos &rc = elrs.GetData();
    bool signalLost = !rc.signalOk;
    bool armed = (rc.ch[4] > 1500);

    uint16_t chThrottle = Clamp(rc.ch[0], (uint16_t)1000, (uint16_t)2000);
    uint16_t chRoll     = Clamp(rc.ch[1], (uint16_t)1000, (uint16_t)2000);
    uint16_t chPitch    = Clamp(rc.ch[2], (uint16_t)1000, (uint16_t)2000);
    uint16_t chYaw      = Clamp(rc.ch[3], (uint16_t)1000, (uint16_t)2000);

    float throttle_us = static_cast<float>(chThrottle);
    if (throttle_us > kThrottleMax) throttle_us = kThrottleMax;

    float dt = muestreo_us / 1000000.0f;

    // Controlador apagado: desarmado, sin señal, o throttle debajo del umbral
    if (!armed || signalLost || throttle_us < kUmbralControlador)
    {
        pidRollAngulo.Reiniciar();
        pidPitchAngulo.Reiniciar();
        pidYawVelocidad.Reiniciar();
        pidRollVelocidad.Reiniciar();
        pidPitchVelocidad.Reiniciar();
        motores.Actualizar(armed && !signalLost, signalLost, throttle_us, 0.0f, 0.0f, 0.0f);
        return;
    }

    float throttleOut = throttle_us;

#if PRUEBA_MEZCLADOR_SIN_PID
    float rollOut  = (AplicarZonaMuerta(chRoll)  - 1500.0f);
    float pitchOut = (AplicarZonaMuerta(chPitch) - 1500.0f);
    float yawOut   = (AplicarZonaMuerta(chYaw)   - 1500.0f);
#else
    float ConsignarollAng  = 0.02f*(AplicarZonaMuerta(chRoll)  - 1500.0f);
    float ConsignapitchAng = 0.02f*(AplicarZonaMuerta(chPitch) - 1500.0f);
    float ConsignayawVel   = 0.24f*(AplicarZonaMuerta(chYaw)   - 1500.0f);

    float rollFiltro, pitchFiltro, yawFiltro;
    orientacion.GetEuler(rollFiltro, pitchFiltro, yawFiltro);
    const MPU6500_Procesado &d = imu.ObtenerProcesado();

    // Medición de yaw tal como la usa el PID (convención de la palanca)
    const float yawVelMedida = -d.gz;

    if (dt > 0.0f) {
        if (dt < dtMin) dtMin = dt;
        if (dt > dtMax) dtMax = dt;
        dtSuma += dt;
        contador++;
    }

    float rollOutAngulo  = pidRollAngulo.ActualizarPID(ConsignarollAng,  rollFiltro,  dt,
        pid.kp_angulo_roll,  pid.ki_angulo_roll,  pid.kd_angulo_roll);

    float pitchOutAngulo = pidPitchAngulo.ActualizarPID(ConsignapitchAng, pitchFiltro, dt,
        pid.kp_angulo_pitch, pid.ki_angulo_pitch, pid.kd_angulo_pitch);

    float rollOut  = pidRollVelocidad.ActualizarPID(rollOutAngulo,  d.gx, dt,
        pid.kp_velocidad_roll,  pid.ki_velocidad_roll,  pid.kd_velocidad_roll);

    float pitchOut = pidPitchVelocidad.ActualizarPID(pitchOutAngulo, d.gy, dt,
        pid.kp_velocidad_pitch, pid.ki_velocidad_pitch, pid.kd_velocidad_pitch);

    float yawOut   = pidYawVelocidad.ActualizarPID(ConsignayawVel, yawVelMedida, dt,
        pid.kp_velocidad_yaw, pid.ki_velocidad_yaw, pid.kd_velocidad_yaw);

#if DEBUG_ACTUALIZAR_PID
    static uint8_t div = 0;
    if (++div >= 4) {
        div = 0;
        char dbg[320];
        int dl = snprintf(dbg, sizeof(dbg),
            "PID,%lu,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
            (unsigned long)HAL_GetTick(),
            ConsignarollAng, rollFiltro, ConsignarollAng - rollFiltro,
            pidRollAngulo.ObtenerTerminoP(), pidRollAngulo.ObtenerTerminoI(), pidRollAngulo.ObtenerTerminoD(), rollOut,
            ConsignapitchAng, pitchFiltro, ConsignapitchAng - pitchFiltro,
            pidPitchAngulo.ObtenerTerminoP(), pidPitchAngulo.ObtenerTerminoI(), pidPitchAngulo.ObtenerTerminoD(), pitchOut,
            ConsignayawVel, yawVelMedida, ConsignayawVel - yawVelMedida,
            pidYawVelocidad.ObtenerTerminoP(), pidYawVelocidad.ObtenerTerminoI(), pidYawVelocidad.ObtenerTerminoD(), yawOut);
        CDC_Transmit_FS((uint8_t*)dbg, dl);
    }
#endif
#endif

    motores.Actualizar(armed, signalLost, throttleOut, rollOut, pitchOut, yawOut);
}
#endif
//FIN CONTROL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void AlCompletarTxMavlink(UART_HandleTypeDef *huart) { mavlinkTx.AlCompletarTx(); }

#if VISUALIZAR_FILTRO
static void ImprimirDebugTiempoReal()
{
    // Decimado por TIEMPO, no por conteo de llamadas — App_Loop no corre
    // a frecuencia fija garantizada (depende del presupuesto del
    // planificador), así que contar llamadas daría una tasa variable.
    static uint32_t ultimoTick = 0;
    uint32_t ahora = HAL_GetTick();
    if (ahora - ultimoTick < 50) return;   // ~20 Hz — sube o baja a gusto
    ultimoTick = ahora;

    const MPU6500_Procesado &d = imu.ObtenerProcesado();
    float YawMag = mag.YawCrudoAtan2(d.roll, d.pitch);   // "yaw" del sensor: viene del magnetómetro, no del acelerómetro
    float rollEkf, pitchEkf, yawEkf;
    orientacion.GetEuler(rollEkf, pitchEkf, yawEkf);

    char dbg[180];
    int dl = snprintf(dbg, sizeof(dbg),
        "RT,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        ahora, d.roll, d.pitch, YawMag, rollEkf, pitchEkf, yawEkf);
    CDC_Transmit_FS((uint8_t*)dbg, dl);
}
#endif

#if VISUALIZAR_MTF02P 
static void ImprimirDebugPosicion()
{
    static uint32_t ultimoTick = 0;
    uint32_t ahora = HAL_GetTick();
    if (ahora - ultimoTick < 50) return;   // ~20 Hz
    ultimoTick = ahora;

    char dbg[160];
    int dl = snprintf(dbg, sizeof(dbg), "POS,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\r\n",
        ahora,
        ekfPosicion.VelX(), ekfPosicion.VelY(),
        ekfPosicion.PosX(), ekfPosicion.PosY(),
        ekfPosicion.SesgoX(), ekfPosicion.SesgoY(),
        ekfPosicion.AlturaUsada(), (unsigned long)ekfPosicion.RechazosFlujo());
    CDC_Transmit_FS((uint8_t*)dbg, dl);
}
#endif

extern "C" void App_Init()
{

    motores.Inicio();
    servo.Init();
    #if CALIBRACION_ESC
        motores.CalibrarEsc();
    #endif
    
    #if HABILITAR_BUZZER
        buzzer.Init();
    #endif

    #if INICIAR_TELEM_MAVLINK
    telemetriaMavlink.Init();
    elrs.SetTelemetriaMavlink(&telemetriaMavlink);
    #endif

    elrs.Init();
    pid.Cargar(); 

	#if HABILITAR_MPU6500
		imu.Inicio();
		imu.CargarCalibracion();
        imu.CalibracionGyro(2000);
	#endif

	#if HABILITAR_QMC5883
		mag.Inicio();
		mag.CargarCalibracion();
	#endif

	#if HABILITAR_BMP180
		baro.Init();
		baro.CalibrateZero(10);
	#endif

    #if HABILITAR_GPS
        gps.Init();
    #endif

    #if HABILITAR_MTF01
        flow.Init();
    #endif

    #if HABILITAR_BATERIA
        bateria.Init();
    #endif    



	    // Configurar el callback de transmisión según el #define de MAVLINK_MODO_TRANSMISION
    #if (MAVLINK_MODO_TRANSMISION == 0)
    	UartDispatch::RegistrarTx(&huart2, AlCompletarTxMavlink);
    	telemetriaMavlink.SetTxCallback(TransmitirMavlinkUart, &huart2);
    #elif (MAVLINK_MODO_TRANSMISION == 1)
        //telemetriaMavlink.SetTxCallback(TransmitirMavlinkSpi, nullptr /* &loraDriver */);
    #endif

    #if HABILITAR_MPU6500
        planificador.AgregarTarea(TareaGyro, nullptr, MS(3), PrioridadTarea::CRITICO, 300, "Gyro");
    #endif
    planificador.AgregarTarea(ElrsTask, nullptr, MS(2), PrioridadTarea::CRITICO, 100, "Elrs");
    planificador.AgregarTarea(TareaMotor, nullptr, MS(4), PrioridadTarea::CRITICO, 800, "Motor");
    planificador.AgregarTarea(TareaServo, nullptr, MS(20), PrioridadTarea::CRITICO, 100, "Servo");
	#if HABILITAR_QMC5883
		planificador.AgregarTarea(TareaMag,  nullptr, MS(6), PrioridadTarea:: NORMAL, 300, "Mag");
	#endif
	#if HABILITAR_BMP180
		planificador.AgregarTarea(BaroTask, nullptr, MS(20), PrioridadTarea::NORMAL, 200, "Baro");
	#endif
    #if HABILITAR_GPS
        planificador.AgregarTarea(GpsTask, nullptr, MS(200), PrioridadTarea::NORMAL, 200, "Gps");
    #endif
	#if HABILITAR_MTF01
		planificador.AgregarTarea(FlowTask, nullptr, MS(10), PrioridadTarea::NORMAL, 50, "Flow");
	#endif
    #if HABILITAR_BATERIA
        planificador.AgregarTarea(TareaBateria, nullptr, MS(50), PrioridadTarea::NORMAL, 300, "Bat");
    #endif
    #if HABILITAR_BUZZER
        planificador.AgregarTarea(TareaBuzzer, nullptr, MS(10), PrioridadTarea::NORMAL, 50, "Buzzer");
        buzzer.Reproducir(kMelodiaInicioListo);
    #endif

    buzzer.Reproducir(kMelodiaInicioListo);
}

extern "C" void App_Loop()
{

	planificador.Inicio(4500); // presupuesto de 3.5ms por vuelta, ajusta según pruebes
	telemetria_usb.Proceso_Calibracion();
	telemetria_usb.Update();
    telemetriaMavlink.Update();

    #if VISUALIZAR_FILTRO
    ImprimirDebugTiempoReal();
    #endif

    #if VISUALIZAR_MTF02P 
    ImprimirDebugPosicion();
    #endif

    #if DEPURAR_LOOP
    static uint32_t ultimoReporte = 0;
    if (HAL_GetTick() - ultimoReporte >= 1000) {
        ultimoReporte = HAL_GetTick();
        if (contador > 0) {
            float dtProm = dtSuma / contador;
            char dbg[120];
            int dl = snprintf(dbg, sizeof(dbg),
                "LOOPHZ,motor,n=%lu,dt_min=%.3f,dt_prom=%.3f,dt_max=%.3f,hz_prom=%.1f\r\n",
                (unsigned long)contador, dtMin, dtProm, dtMax, 1.0f/dtProm);
            CDC_Transmit_FS((uint8_t*)dbg, dl);
        }
        contador = 0; dtSuma = 0; dtMin = 999.0f; dtMax = 0.0f;
    }
    #endif
    mavlinkTx.Bombear();
}

extern "C" void App_Comando(char *chunk)
{
    static char buf_acumulado[256];
    static uint16_t idx = 0;

    while (*chunk)
    {
        char c = *chunk++;
        if (c == '\r' || c == '\n')
        {
            if (idx > 0)
            {
                buf_acumulado[idx] = '\0';
                telemetria_usb.Comando(buf_acumulado);
                idx = 0;
            }
        }
        else if (idx < sizeof(buf_acumulado) - 1)
        {
            buf_acumulado[idx++] = c;
        }
    }
}



// #include <cmath>

// void EnviarTelemetriaLKF(uint32_t muestreo_us)
// {
//     static uint32_t ultimoTickUs = 0;
//     uint32_t ahoraUs = HAL_GetTick() * 1000; // microsegundos
    
//     // Decimado: Enviar cada ~10 ms (100 Hz)
//     if (ahoraUs - ultimoTickUs < 10000) return; 
//     ultimoTickUs = ahoraUs;

//     // 1. Datos IMU (Acelerómetro en m/s^2)
//     const MPU6500_Procesado &imu_data = imu.ObtenerProcesado();
//     float ax = imu_data.ax * 9.81f;
//     float ay = imu_data.ay * 9.81f;
//     float az = imu_data.az * 9.81f;

//     // 2. Obtener Cuaterniones y convertir a ángulos Euler (Roll y Pitch)
//     float q0, q1, q2, q3;
//     orientacion.GetQuaternion(q0, q1, q2, q3);
//     float roll,pitch,yaw;

//     orientacion.GetEuler(roll, pitch, yaw); // Estan en grados

//     // 4. Datos del Óptico y ToF
//     const MTF01_Data &flow_data = flow.GetData();

//     // A. Altura real perpendicular al suelo
//     float dist_m = static_cast<float>(flow_data.distance) / 1000.0f; // Convertir a metros
//     float cos_roll  = std::cos(roll * DEG2RAD);
//     float cos_pitch = std::cos(pitch * DEG2RAD); // Probare en grados primero aver si funciona
    
//     float z_m = dist_m * cos_roll * cos_pitch;

//     //Leemos los flujos en X y Y m/s
//     float flow_vx = (static_cast<float>(flow_data.flowX) * z_m) / 100.0f; // Convertir a m/s
//     float flow_vy = (static_cast<float>(flow_data.flowY) * z_m) / 100.0f; // Convertir a m/s

//     float flow_vx_w= flow_vx * std::cos(yaw * DEG2RAD) - flow_vy * std::sin(yaw * DEG2RAD); // orientacion en yaw w = world
//     float flow_vy_w = flow_vx * std::sin(yaw * DEG2RAD) + flow_vy * std::cos(yaw * DEG2RAD);

//     // 5. Transmisión CSV a Python
//     char buffer[256];
//     int len = snprintf(buffer, sizeof(buffer),
//         "%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
//         (unsigned long)ahoraUs,
//         ax, ay, az,
//         q0, q1, q2, q3,
//         imu_data.gx, imu_data.gy, imu_data.gz,
//         flow_vx_w, // Velocidad lineal m/s
//         flow_vy_w, // Velocidad lineal m/s
//         z_m    // Altura perpendicular real en metros
//     );

//     CDC_Transmit_FS(reinterpret_cast<uint8_t*>(buffer), len);
// }

// static void TareaTelemetriaLKF(void *ctx, uint32_t muestreo_us)
// {
//     EnviarTelemetriaLKF(muestreo_us);
// }

    //planificador.AgregarTarea(TareaTelemetriaLKF, nullptr, MS(50), PrioridadTarea::NORMAL, 100, "TelemLKF");