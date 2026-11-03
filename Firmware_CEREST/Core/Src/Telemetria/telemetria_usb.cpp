#include "Telemetria/telemetria_usb.hpp"
#include "Control/mahony.hpp"
#include "usbd_cdc_if.h"
#include <cstdio>
#include "Proceso/sd.hpp"
#include "Control/pid.hpp"  
#include <cmath>

//Telemetria_USB::Telemetria_USB(MPU6500 &imu, QMC5883 &mag, MTF01 &flow, Gnss &gps, FuenteRc &elrs, Scheduler &scheduler)
//								: imu_(imu), mag_(mag), flow_(flow), gps_(gps), elrs_(elrs), scheduler_(scheduler){}

Telemetria_USB::Telemetria_USB(MPU6500 &imu, QMC5883P &mag, BMP180 &baro, MTF01 &flow, Gnss &gps, RcProtocolo &elrs,
                                MahonyOrientacion &orientacion, EkfPosicion &ekfPosicion,
                                Planificador &planificador, MotorController &motores)
                                : imu_(imu), mag_(mag), baro_(baro), flow_(flow), gps_(gps), elrs_(elrs), orientacion_(orientacion),
                                  ekfPosicion_(ekfPosicion), planificador_(planificador), motores_(motores) {}


void Telemetria_USB::Comando(char *cmd){
    // Comandos extendidos de PC (llevan ':') van a un manejador separado
    if(strchr(cmd, ':') != nullptr)
    {
        ProcesarComandoPC(cmd);
        return;
    }
    if(strcmp(cmd,"x") == 0){
		modo_ = Telemetria_USB_Modo::Desactivado;
		static const char msg[] =  "Lectura Detenida \r\n"
								   "Escriba: \r\n"
								   "GYRO - MAG - GPS - ELRS - FLOW - BARO - CALMAG - CALGyro - CALESC - CALSTATUS - SCHEDSTATUS \r\n"
								   "MAVLINK - x(Detener)";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
	else if(strcmp(cmd,"GYRO") == 0) modo_ = Telemetria_USB_Modo::Gyro;
	else if(strcmp(cmd, "MAG") == 0) modo_ = Telemetria_USB_Modo::Mag;
	else if(strcmp(cmd, "FLOW") == 0) modo_ = Telemetria_USB_Modo::Flow;
	else if(strcmp(cmd, "BARO") == 0) modo_ = Telemetria_USB_Modo::Baro;
	else if(strcmp(cmd,"GPS") == 0) modo_ = Telemetria_USB_Modo::Gps;
	else if(strcmp(cmd,"ELRS") == 0) modo_ =Telemetria_USB_Modo::Elrs;
	else if(strcmp(cmd,"EKF") == 0) modo_ = Telemetria_USB_Modo::Ekf;
	//else if(strcmp(cmd,"CALMAG") == 0) solicitudCalMag_ = true;
	else if(strcmp(cmd, "CALSTATUS") == 0) CalStatus();
	else if(strcmp(cmd, "CALESC") == 0) solicitudCalEsc_ = true;
	else if(strcmp(cmd, "CALGyro") == 0) solicitudCalGyro_= true;
    else if(strcmp(cmd, "STREAMMAG") == 0) modo_ = Telemetria_USB_Modo::StreamMag;
    else if(strcmp(cmd, "STREAMACCEL") == 0) modo_ = Telemetria_USB_Modo::StreamAccel;
	else if(strcmp(cmd, "SCHEDSTATUS") == 0) PlanificadorStatus();
	//else if(strcmp(cmd, "SCHEDSTATUS") == 0) SchedStatus();
}

void Telemetria_USB::CalStatus()
{
    bool magOk = mag_.CargarCalibracion();
    float mox = 0, moy = 0, moz = 0;
    if(magOk) mag_.ObtenerOffsets(mox, moy, moz);

    bool imuOk = imu_.CargarCalibracion();
    float gox = 0, goy = 0, goz = 0, aox = 0, aoy = 0, aoz = 0;
    if(imuOk)
    {
        imu_.ObtenerOffsetsGyro(gox, goy, goz);
        imu_.ObtenerOffsetsAccel(aox, aoy, aoz);
    }

    GananciasPID pidData;
    bool pidOk = pidData.Cargar();

    static char msg[1200];   // subido de 900: se agregó la matriz del trim
    int len = 0;

    // --- Formateo de Sensores ---
    if(magOk)
    {
        float m[3][3];
        mag_.ObtenerMatrizCorreccion(m);
        len += snprintf(msg + len, sizeof(msg) - len,
            "MAG: OK offset(%.4f, %.4f, %.4f)\r\n"
            "  Matriz: [%.3f %.3f %.3f]\r\n"
            "          [%.3f %.3f %.3f]\r\n"
            "          [%.3f %.3f %.3f]\r\n",
            mox, moy, moz,
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]);
    }
    else
        len += snprintf(msg + len, sizeof(msg) - len, "MAG: sin calibrar\r\n");

    if(imuOk)
    {
        float ma[3][3];
        imu_.ObtenerMatrizAccel(ma);
        len += snprintf(msg + len, sizeof(msg) - len,
            "IMU: OK gyro(%.3f,%.3f,%.3f) accel_offset(%.3f,%.3f,%.3f)\r\n"
            "  Matriz Accel: [%.3f %.3f %.3f]\r\n"
            "                [%.3f %.3f %.3f]\r\n"
            "                [%.3f %.3f %.3f]\r\n",
            gox, goy, goz, aox, aoy, aoz,
            ma[0][0], ma[0][1], ma[0][2],
            ma[1][0], ma[1][1], ma[1][2],
            ma[2][0], ma[2][1], ma[2][2]);

        // --- Trim de nivel ---
        // La tercera fila de la rotación es la gravedad que se midió en el paso de nivel:
        // de ahí salen los ángulos que corrige el trim.
        float rt[3][3];
        imu_.ObtenerTrim(rt);
        const float rollTrim  = atan2f(rt[2][1], rt[2][2]) * 57.29578f;
        const float pitchTrim = atan2f(-rt[2][0], sqrtf(rt[2][1]*rt[2][1] + rt[2][2]*rt[2][2])) * 57.29578f;
        len += snprintf(msg + len, sizeof(msg) - len,
            "  Trim nivel: roll=%.2f deg pitch=%.2f deg\r\n"
            "  Matriz Trim:  [%.4f %.4f %.4f]\r\n"
            "                [%.4f %.4f %.4f]\r\n"
            "                [%.4f %.4f %.4f]\r\n",
            rollTrim, pitchTrim,
            rt[0][0], rt[0][1], rt[0][2],
            rt[1][0], rt[1][1], rt[1][2],
            rt[2][0], rt[2][1], rt[2][2]);
    }
    else
        len += snprintf(msg + len, sizeof(msg) - len, "IMU: sin calibrar\r\n");

    if(pidOk)
    {
        len += snprintf(msg + len, sizeof(msg) - len,
            "PID Angulo:\r\n"
            "  Roll:  P=%.2f I=%.2f D=%.2f\r\n"
            "  Pitch: P=%.2f I=%.2f D=%.2f\r\n"
            "PID Rate (velocidad):\r\n"
            "  Roll:  P=%.3f I=%.3f D=%.4f\r\n"
            "  Pitch: P=%.3f I=%.3f D=%.4f\r\n"
            "  Yaw:   P=%.3f I=%.3f D=%.4f\r\n",
            pidData.kp_angulo_roll,  pidData.ki_angulo_roll,  pidData.kd_angulo_roll,
            pidData.kp_angulo_pitch, pidData.ki_angulo_pitch, pidData.kd_angulo_pitch,
            pidData.kp_velocidad_roll,   pidData.ki_velocidad_roll,   pidData.kd_velocidad_roll,
            pidData.kp_velocidad_pitch,  pidData.ki_velocidad_pitch,  pidData.kd_velocidad_pitch,
            pidData.kp_velocidad_yaw,    pidData.ki_velocidad_yaw,    pidData.kd_velocidad_yaw);
    }
    else
    {
        len += snprintf(msg + len, sizeof(msg) - len, "PID: sin calibrar\r\n");
    }

    if (len > (int)sizeof(msg)) len = sizeof(msg) - 1;   // por si el mensaje se truncó
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::PlanificadorStatus(){
    static char msg[600];
    int totalLen = 0;

    totalLen += snprintf(msg + totalLen, sizeof(msg) - totalLen, "=== SCHEDULER STATUS ===\r\n");

    for(uint8_t i = 0; i < planificador_.ObtenerCantidadTareas(); i++)
    {
        const Planificador::Tarea &t = planificador_.ObtenerTarea(i);
        totalLen += snprintf(msg + totalLen, sizeof(msg) - totalLen,
            "%-10s overrun=%lu skipped=%lu\r\n",
            t.name, t.cuentaExcedida, t.cuentaOmitida);

        if(totalLen >= (int)sizeof(msg) - 50) break;   // protección contra overflow del buffer
    }

    CDC_Transmit_FS((uint8_t*)msg, totalLen);
}

void Telemetria_USB::Update(){
	switch(modo_){
	case Telemetria_USB_Modo::Gyro : ImprimirGyro(); break;
	case Telemetria_USB_Modo::Mag:  ImprimirMag();  break;
	case Telemetria_USB_Modo::Flow:  ImprimirFlow();  break;
	case Telemetria_USB_Modo::Baro : ImprimirBaro(); break;
	case Telemetria_USB_Modo::Gps: ImprimirGps(); break;
	case Telemetria_USB_Modo::Elrs: ImprimirElrs(); break;
    case Telemetria_USB_Modo::Ekf:  ImprimirEkf();  break;
    case Telemetria_USB_Modo::StreamMag:   ImprimirStreamMag();   break;   
    case Telemetria_USB_Modo::StreamAccel: ImprimirStreamAccel(); break;  
	case Telemetria_USB_Modo::Desactivado:  default: break;
	}
}

void Telemetria_USB::ObtenerMuestreo(Telemetria_USB_Modo modo, uint32_t muestreo){
	uint8_t idx = static_cast<uint8_t>(modo);
	muestreo_[idx].tick = HAL_GetTick();
	muestreo_[idx].muestreo = muestreo;
}

void Telemetria_USB::ImprimirGyro(){
    const MPU6500_Procesado &d = imu_.ObtenerProcesado();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::Gyro)];

    static char msg[200];
    int len = snprintf(msg, sizeof(msg),
        "GYRO\r\nAX:%.2f AY:%.2f AZ:%.2f\r\nGX:%.2f GY:%.2f GZ:%.2f\r\nRoll:%.2f Pitch:%.2f\r\n"
        "tick:%lu dt:%luus\r\n",
        d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.roll, d.pitch,
        s.tick, s.muestreo);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::ImprimirMag(){
    const QMC5883Procesado &d = mag_.ObtenerProcesado();
    float YawMag = mag_.YawCrudoAtan2(imu_.ObtenerProcesado().roll, imu_.ObtenerProcesado().pitch);
    const Muestreo &s = muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::Mag)];

    static char msg[150];
    int len = snprintf(msg, sizeof(msg),
        "MAG\r\nMX:%.2f MY:%.2f MZ:%.2f Heading:%.2f\r\ntick:%lu dt:%luus\r\n",
        d.mx, d.my, d.mz, YawMag, s.tick, s.muestreo);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::ImprimirBaro(){
	const Muestreo &s = muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::Baro)];

	static char msg[150];
	    int len = snprintf(msg, sizeof(msg),
	        "BARO\r\nTemp:%.2fC Pres:%.2fhPa Alt:%.2fm\r\ntick:%lu dt:%luus\r\n",
	        baro_.GetTemperature(),
	        baro_.GetPressurehPa(),
	        baro_.GetAltitude(),
	        s.tick,
	        s.muestreo);

	    // 3. Transmitir por USB CDC
	    CDC_Transmit_FS(reinterpret_cast<uint8_t*>(msg), static_cast<uint16_t>(len));
}

void Telemetria_USB::ImprimirFlow(){
    const MTF01_Data &d = flow_.GetData();
    const PosicionMTF &pos = flow_.GetPosicion();
    // FLOW se actualiza por interrupción, de forma asíncrona — no pasa
    // por el scheduler, así que aquí "tick" es el momento de la CONSULTA,
    // no necesariamente el momento exacto del último byte recibido.
    static char msg[180];
    int len = snprintf(msg, sizeof(msg),
        "FLOW\r\nDistance: %.2f mm\r\nVelX: %.2f,PosX: %.2f VelY: %.2f, PosY: %.2f\r\nQuality: %d Status: %d\r\n"
        "consultado en tick:%lu\r\n",
        d.distance, d.flowX, pos.x, d.flowY, pos.y, d.quality, d.status, HAL_GetTick());
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::ImprimirGps(){
    const GnssData &d = gps_.GetData();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::Gps)];

    // Tipo de fix con nombre (NAV-PVT: 0 sin fix, 2 = 2D, 3 = 3D)
    const char *nombreFix;
    switch (d.fixType) {
        case 0:  nombreFix = "sin fix";   break;
        case 1:  nombreFix = "solo DR";   break;
        case 2:  nombreFix = "2D";        break;
        case 3:  nombreFix = "3D";        break;
        case 4:  nombreFix = "GNSS+DR";   break;
        case 5:  nombreFix = "solo hora"; break;
        default: nombreFix = "?";         break;
    }

    static char msg[400];
    int len = snprintf(msg, sizeof(msg),
        "GPS\r\nDay: %02d-%02d-%04d\r\nTime (UTC): %02d:%02d:%02d\r\n"
        "Status of fix: %d (%s)\r\nSat used: %d Sat count: %d\r\n"
        "Lat: %.7f Lon: %.7f\r\n"
        "Precision horiz.(m): %.2f Precision vert.(m): %.2f\r\n"
        "Height ellip.(m): %.2f Height MSL(m): %.2f\r\n"
        "Ground Speed(2D)(m/s): %.2f\r\ntick:%lu dt:%luus\r\n",
        d.day, d.month, d.year, d.hour, d.min, d.sec,
        d.fixType, nombreFix, d.numSV, d.satCount, d.fLat, d.fLon,
        static_cast<float>(d.hAcc) / 1000.0f, static_cast<float>(d.vAcc) / 1000.0f,
        static_cast<float>(d.height) / 1000.0f, static_cast<float>(d.hMSL) / 1000.0f,
        static_cast<float>(d.gSpeed) / 1000.0f,
        (unsigned long)s.tick, (unsigned long)s.muestreo);
    CDC_Transmit_FS((uint8_t*)msg, len);  
}


void Telemetria_USB::ImprimirEkf()
{
    float roll, pitch, yaw;
    orientacion_.GetEuler(roll, pitch, yaw);

    float vx, vy, vz, px, py, pz;
    ekfPosicion_.GetVelocidad(vx, vy, vz);
    ekfPosicion_.GetPosicion(px, py, pz);

    static char msg[220];
    int len = snprintf(msg, sizeof(msg),
        "EKF\r\nRoll:%.2f Pitch:%.2f Yaw:%.2f\r\n"
        "Vel: %.2f,%.2f,%.2f  Pos: %.2f,%.2f,%.2f\r\n"
        "Rechazos: Accel=%lu Yaw=%lu Baro=%lu Flujo=%lu\r\n",
        roll, pitch, yaw, vx, vy, vz, px, py, pz,
        orientacion_.GetRechazosAccel(), orientacion_.GetRechazosYaw(),
        ekfPosicion_.GetRechazosBaro(), ekfPosicion_.GetRechazosFlujo());
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::ImprimirElrs()
{
    const RcDatos &d = elrs_.GetData();
    const Muestreo &s = muestreo_[static_cast<uint8_t>(Telemetria_USB_Modo::Elrs)];

    static char msg[256];

    int len = snprintf(msg, sizeof(msg),
        "ELRS | Status: %d | ActCh: %d | RSSI: %d dBm | LQ: %u%%\r\n"
        "CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH10:%u CH8: %u\r\n"
        "tick:%lu dt:%luus\r\n",
        d.signalOk ? 1 : 0,
        d.canalesActivos,
        d.rssiDbm,
        d.linkQuality,
        d.ch[0], d.ch[1], d.ch[2], d.ch[3], d.ch[4], d.ch[9], d.ch[7],
        s.tick,
        s.muestreo);

    CDC_Transmit_FS(reinterpret_cast<uint8_t*>(msg), len);
}
void Telemetria_USB::Proceso_Calibracion(){
	// if(solicitudCalMag_){
	// 	solicitudCalMag_ = false;
	// 	RunMagCalibracion();
	// }
    if(solicitudCalGyro_){
        solicitudCalGyro_ = false;
        CalibrarGyroSolo();
    }
	if(solicitudCalEsc_){
		solicitudCalEsc_ = false;
		motores_.CalibrarEsc();
	}
}

void Telemetria_USB::ImprimirStreamMag()
{
    static uint32_t ultimoEnvio = 0;
    uint32_t ahora = HAL_GetTick();
    if(ahora - ultimoEnvio < 50) return;   // mismo espaciado que tu C++ original
    ultimoEnvio = ahora;

    if(!mag_.LeeCrudo()) return;

    const QMC5883Crudo &raw = mag_.ObtenerCrudo();
    static char msg[40];
    int len = snprintf(msg, sizeof(msg), "%d,%d,%d\r\n", raw.mx, raw.my, raw.mz);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

// telemetria_usb.cpp — mismo patrón que ImprimirStreamMag, con espaciado real
void Telemetria_USB::ImprimirStreamAccel()
{
    static uint32_t ultimoEnvio = 0;
    uint32_t ahora = HAL_GetTick();
    if(ahora - ultimoEnvio < 50) return;
    ultimoEnvio = ahora;

    imu_.LeeCrudo();
    const MPU6500_Crudo &raw = imu_.ObtenerCrudo();
    static char msg[40];
    int len = snprintf(msg, sizeof(msg), "%d,%d,%d\r\n", raw.ax, raw.ay, raw.az);
    CDC_Transmit_FS((uint8_t*)msg, len);
}

void Telemetria_USB::CalibrarGyroSolo()
{
    char msg[100]; int len;
    len = snprintf(msg, sizeof(msg), "\r\n=== CALIBRACION GYRO ===\r\nDeja el dron PLANO e INMOVIL...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, len);
    HAL_Delay(2000);

    imu_.CalibracionGyro(4000);
    imu_.GuardarCalibracion();   // guarda gyro; accel conserva lo ya cargado en memoria

    len = snprintf(msg, sizeof(msg), "Gyro calibrado y guardado.\r\n");
    CDC_Transmit_FS((uint8_t*)msg, len);
}


/**
 * @brief Parsea N valores float separados por comas desde un string.
 * Usa strtof() en vez de sscanf(), ya que sscanf con %f requiere
 * _scanf_float en el enlazador — símbolo ausente en este toolchain
 * (newlib-nano), mientras que strtof() no tiene esa dependencia.
 */
int Telemetria_USB::ParsearFloats(char *ptr, float *out, int max_cant)
{
    int leidos = 0;
    for (int i = 0; i < max_cant; i++)
    {
        char *endptr;
        out[i] = strtof(ptr, &endptr);
        if (ptr == endptr) break;   // no se pudo leer ningún número aquí
        leidos++;
        if (*endptr == ',') endptr++;
        ptr = endptr;
    }
    return leidos;
}

/**
 * @brief Procesa comandos extendidos enviados desde un script de PC
 * (Python), distintos de los comandos cortos de usuario (GYRO, MAG, etc).
 * Centraliza todo el manejo de calibración remota en un solo lugar.
 */
void Telemetria_USB::ProcesarComandoPC(char *cmd)
{
    if(strncmp(cmd, "SETCALMAG:", 10) == 0)
    {
        float v[12] = {0};
        int leidos = ParsearFloats(cmd + 10, v, 12);

        if(leidos == 12)
        {
            mag_.AsignarOffsets(v[0], v[1], v[2]);
            mag_.SetMatrizCorreccion(v[3], v[4], v[5],
                                      v[6], v[7], v[8],
                                      v[9], v[10], v[11]);
            mag_.GuardarCalibracion();
            static const char ok[] = "MAG OK\r\n";
            CDC_Transmit_FS((uint8_t*)ok, strlen(ok));
        }
        else
        {
            static const char err[] = "MAG ERROR: se esperaban 12 valores\r\n";
            CDC_Transmit_FS((uint8_t*)err, strlen(err));
        }
        return;
    }

    if(strncmp(cmd, "SETCALACCEL:", 12) == 0)
    {
        float v[12] = {0};
        int leidos = ParsearFloats(cmd + 12, v, 12);

        if(leidos == 12)
        {
            imu_.AsignarOffsetsAccel(v[0], v[1], v[2]);
            imu_.AsignarMatrizAccel(v[3],v[4],v[5], v[6],v[7],v[8], v[9],v[10],v[11]);
            imu_.GuardarCalibracion();
            static const char ok[] = "ACCEL OK\r\n";
            CDC_Transmit_FS((uint8_t*)ok, strlen(ok));
        }
        else
        {
            static const char err[] = "ACCEL ERROR: se esperaban 12 valores\r\n";
            CDC_Transmit_FS((uint8_t*)err, strlen(err));
        }
        return;
    }

    // Trim de nivel: matriz de rotación 3x3 (9 valores, por filas)
    if(strncmp(cmd, "SETTRIM:", 8) == 0)
    {
        float v[9] = {0};
        int leidos = ParsearFloats(cmd + 8, v, 9);

        if(leidos == 9)
        {
            imu_.AsignarTrim(v);
            imu_.GuardarCalibracion();
            static const char ok[] = "TRIM OK\r\n";
            CDC_Transmit_FS((uint8_t*)ok, strlen(ok));
        }
        else
        {
            static const char err[] = "TRIM ERROR: se esperaban 9 valores\r\n";
            CDC_Transmit_FS((uint8_t*)err, strlen(err));
        }
        return;
    }

    static const char noReconocido[] = "Comando PC no reconocido\r\n";
    CDC_Transmit_FS((uint8_t*)noReconocido, strlen(noReconocido));
}
// // telemetria_usb.hpp, en la sección private:
// static constexpr uint16_t kMaxMuestrasCalibracion = 1000;   // 10 pos × 150 muestras (el caso más grande)
// static float bufferCalibX_[kMaxMuestrasCalibracion];
// static float bufferCalibY_[kMaxMuestrasCalibracion];
// static float bufferCalibZ_[kMaxMuestrasCalibracion];

// void Telemetria_USB::RunMagCalibracion()
// {
//     static constexpr uint16_t TOTAL_SAMPLES = 150;   // sigue siendo tu tamaño real de uso

//     int16_t minX=32767, maxX=-32768, minY=32767, maxY=-32768, minZ=32767, maxZ=-32768;

//     static char msg[256];
//     int len = snprintf(msg, sizeof(msg),
//         "CALIBRACION MAG (minimos cuadrados 3D)\r\nMueve en OCHOS y gira en todas direcciones\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);
//     HAL_Delay(3000);

//     uint16_t count = 0;
//     uint32_t lastTime = HAL_GetTick();

//     while (count < TOTAL_SAMPLES)
//     {
//         if (HAL_GetTick() - lastTime >= 50)
//         {
//             lastTime = HAL_GetTick();
//             if (mag_.LeeCrudo())
//             {
//                 const QMC5883Crudo &raw = mag_.ObtenerCrudo();
//                 // Usa el buffer compartido, NO un arreglo propio
//                 bufferCalibX_[count] = static_cast<float>(raw.mx);
//                 bufferCalibY_[count] = static_cast<float>(raw.my);
//                 bufferCalibZ_[count] = static_cast<float>(raw.mz);

//                 if(raw.mx < minX) minX = raw.mx;
//                 if(raw.mx > maxX) maxX = raw.mx;
//                 if(raw.my < minY) minY = raw.my;
//                 if(raw.my > maxY) maxY = raw.my;
//                 if(raw.mz < minZ) minZ = raw.mz;
//                 if(raw.mz > maxZ) maxZ = raw.mz;

//                 count++;
//                 if (count % 40 == 0)
//                 {
//                     len = snprintf(msg, sizeof(msg), "Progreso: %d%%\r\n", (count*100)/TOTAL_SAMPLES);
//                     CDC_Transmit_FS((uint8_t*)msg, len);
//                 }
//             }
//         }
//     }

//     constexpr int16_t kRangoMinimoRequerido = 800;
//     int16_t rangoX = maxX - minX, rangoY = maxY - minY, rangoZ = maxZ - minZ;
//     if (rangoX < kRangoMinimoRequerido || rangoY < kRangoMinimoRequerido || rangoZ < kRangoMinimoRequerido)
//     {
//         len = snprintf(msg, sizeof(msg),
//             "ADVERTENCIA: cobertura insuficiente (rangos X=%d Y=%d Z=%d) — gira mas en 3D y repite\r\n",
//             rangoX, rangoY, rangoZ);
//         CDC_Transmit_FS((uint8_t*)msg, len);
//         return;
//     }

//     // Usa el buffer compartido aquí también
//     if (mag_.CalibracionMinimosCuadrados(bufferCalibX_, bufferCalibY_, bufferCalibZ_, count))
//     {
//         mag_.GuardarCalibracion();
//         float ox, oy, oz;
//         mag_.ObtenerOffsets(ox, oy, oz);
//         len = snprintf(msg, sizeof(msg),
//             "OK. Offsets: X=%.4f Y=%.4f Z=%.4f | Calidad: %.1f%% (menor es mejor)\r\n",
//             ox, oy, oz, mag_.ObtenerCalidadAjuste()*100.0f);
//         CDC_Transmit_FS((uint8_t*)msg, len);

//         if(mag_.ObtenerCalidadAjuste() > 0.08f)
//         {
//             len = snprintf(msg, sizeof(msg),
//                 "NOTA: calidad > 8%% sugiere soft-iron notorio — considera la extension con matriz de correccion\r\n");
//             CDC_Transmit_FS((uint8_t*)msg, len);
//         }
//     }
//     else
//     {
//         len = snprintf(msg, sizeof(msg), "ERROR: matriz singular (rotacion insuficiente en 3D)\r\n");
//         CDC_Transmit_FS((uint8_t*)msg, len);
//     }
// }


// void Telemetria_USB::CalibrarIMU() {
//     char msg[128];
//     int len;

//     // ==========================================
//     // PASO 1: CALIBRACIÓN DE GIROSCOPIO (2 seg)
//     // ==========================================
//     len = snprintf(msg, sizeof(msg), "\r\n=== CALIBRACION IMU ===\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);

//     len = snprintf(msg, sizeof(msg), "[1/2] Calibrando Giroscopio... ¡DEJA EL DRON PLANO E INMOVIL!\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);
//     HAL_Delay(1000);

//     imu_.CalibracionGyro(2000); // Mide bias estático

//     len = snprintf(msg, sizeof(msg), "Giroscopio OK.\r\n\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);

//     // ==========================================
//     // PASO 2: CALIBRACIÓN DE ACELERÓMETRO (6 POS)
//     // ==========================================
//     len = snprintf(msg, sizeof(msg), "[2/2] Iniciando Acelerometro de 6 posiciones...\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);

//     static const char* posNames[6] = {
//         "1/6: NIVELADO (Plano en mesa, +Z hacia arriba)",
//         "2/6: INVERTIDO (De cabeza, +Z hacia abajo)",
//         "3/6: FRENTE (Apuntando el frente al piso, +X abajo)",
//         "4/6: ATRAS (Apuntando la cola al piso, -X abajo)",
//         "5/6: IZQUIERDA (Lado izquierdo al piso, +Y abajo)",
//         "6/6: DERECHA (Lado derecho al piso, -Y abajo)"
//     };

//     AccelPosDato posData[6];

//     for (int i = 0; i < 6; i++) {
//         // Aprovecha que la postura 1 ya está plana por el gyro
//         if (i > 0) {
//             len = snprintf(msg, sizeof(msg), "\r\nAcomoda el dron en: %s\r\nIniciando en 5s...\r\n", posNames[i]);
//             CDC_Transmit_FS((uint8_t*)msg, len);

//             for (int c = 5; c > 0; c--) {
//                 len = snprintf(msg, sizeof(msg), "%d...\r\n", c);
//                 CDC_Transmit_FS((uint8_t*)msg, len);
//                 HAL_Delay(1000);
//             }
//         } else {
//             len = snprintf(msg, sizeof(msg), "\r\nTomando postura 1/6 (Nivelado)...\r\n");
//             CDC_Transmit_FS((uint8_t*)msg, len);
//         }

//         len = snprintf(msg, sizeof(msg), "Midiendo... ¡No mover!\r\n");
//         CDC_Transmit_FS((uint8_t*)msg, len);

//         imu_.CapturarPosicionAccel(static_cast<PosicionAccel>(i), posData[i], 2000);
//     }

//     // ==========================================
//     // PASO 3: CÁLCULO Y PERSISTENCIA DE DATOS
//     // ==========================================
//     imu_.Calibracion6Posiciones(posData);
//     imu_.GuardarCalibracion(); // Guarda gyro offsets + accel scale/offsets en flash/SD

//     len = snprintf(msg, sizeof(msg), "\r\n¡Calibracion IMU completa y guardada en memoria!\r\n");
//     CDC_Transmit_FS((uint8_t*)msg, len);
// }


// void Telemetria_USB::CalibrarAccelMultiPos()
// {
//     static constexpr uint8_t kNumPosiciones = 10;
//     static constexpr uint16_t kMuestrasPorPosicion = 150;
//     // SIN arreglos propios — usa bufferCalibX_/Y_/Z_ de la clase
//     uint16_t total = 0;

//     int64_t sumGx = 0, sumGy = 0, sumGz = 0;
//     uint16_t muestrasGyro = 0;

//     char msg[128]; int len;
//     len = snprintf(msg,sizeof(msg), "\r\n=== CALIBRACION IMU COMPLETA (Gyro + Accel, %d posiciones) ===\r\n", kNumPosiciones);
//     CDC_Transmit_FS((uint8_t*)msg,len);

//     for(uint8_t pos=0; pos<kNumPosiciones; pos++)
//     {
//         len = snprintf(msg,sizeof(msg),
//             "\r\nColoca el dron en una orientacion NUEVA y distinta (no importa cual exactamente)\r\nQUIETO — iniciando en 4s...\r\n");
//         CDC_Transmit_FS((uint8_t*)msg,len);
//         HAL_Delay(4000);

//         for(uint16_t i=0;i<kMuestrasPorPosicion;i++)
//         {
//             imu_.LeeCrudo();
//             const MPU6500_Crudo &raw = imu_.ObtenerCrudo();

//             bufferCalibX_[total] = raw.ax / MPU6500_ACCEL_ESCALA;
//             bufferCalibY_[total] = raw.ay / MPU6500_ACCEL_ESCALA;
//             bufferCalibZ_[total] = raw.az / MPU6500_ACCEL_ESCALA;
//             total++;

//             if(pos == 0)
//             {
//                 sumGx += raw.gx;
//                 sumGy += raw.gy;
//                 sumGz += raw.gz;
//                 muestrasGyro++;
//             }

//             HAL_Delay(2);
//         }
//         len = snprintf(msg,sizeof(msg), "Posicion %d/%d capturada.\r\n", pos+1, kNumPosiciones);
//         CDC_Transmit_FS((uint8_t*)msg,len);
//     }

//     imu_.AsignarOffsetsGyro(
//         static_cast<float>(sumGx) / muestrasGyro,
//         static_cast<float>(sumGy) / muestrasGyro,
//         static_cast<float>(sumGz) / muestrasGyro
//     );

//     if(imu_.CalibracionMinCuadrados(bufferCalibX_, bufferCalibY_, bufferCalibZ_, total))
//     {
//         imu_.GuardarCalibracion();
//         len = snprintf(msg,sizeof(msg), "\r\nCalibracion completa (Gyro + Accel) y guardada.\r\n");
//     }
//     else
//     {
//         len = snprintf(msg,sizeof(msg), "\r\nERROR: cobertura insuficiente del acelerometro — repite con posiciones mas variadas.");
//     }
//     CDC_Transmit_FS((uint8_t*)msg,len);
// }