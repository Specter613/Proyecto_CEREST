/*
 * mpu6500.cpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#include <Sensores/mpu6500.hpp>

static constexpr uint16_t CS_PIN = GPIO_PIN_12;
#define CS_PUERTO GPIOB

MPU6500::MPU6500(SPI_HandleTypeDef *hspi) : hspi_(hspi) {}

static inline void CS_Bajo(){
	HAL_GPIO_WritePin(CS_PUERTO, CS_PIN, GPIO_PIN_RESET);
}

static inline void CS_Alto(){
	HAL_GPIO_WritePin(CS_PUERTO, CS_PIN, GPIO_PIN_SET);
}

void MPU6500::Inicio(){
	uint8_t dato[2];
 
	// =========================================================================
	// PWR_MGMT_1 (0x6B) — Reset completo
	// -------------------------------------------------------------------------
	//  bit 7  DEVICE_RESET : 1 = reinicia todos los registros (se borra solo)
	//  bit 6  SLEEP        : 1 = modo dormido
	//  bit 5  CYCLE        : 1 = ciclo dormir/despertar (bajo consumo)
	//  bit 4  GYRO_STANDBY : 1 = giroscopio en espera
	//  bit 3  PD_PTAT      : 1 = apaga el sensor de temperatura
	//  bits 2:0 CLKSEL     : fuente de reloj (ver siguiente bloque)
	//
	//  0x80 = reset completo. Esperar ~100 ms después.
	// =========================================================================
	dato[0] = 0x6B; dato[1] = 0x80;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(100);
 
	// =========================================================================
	// PWR_MGMT_1 (0x6B) — Despertar y elegir reloj (CLKSEL, bits 2:0)
	// -------------------------------------------------------------------------
	//  0x00 : oscilador interno de 20 MHz
	//  0x01 : autoselección del mejor reloj (PLL del giroscopio si está listo) <- recomendado
	//  0x02 - 0x05 : igual que 0x01 (autoselección)
	//  0x06 : oscilador interno de 20 MHz
	//  0x07 : detiene el reloj
	// =========================================================================
	dato[0] = 0x6B; dato[1] = 0x01;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(100);
 
	// =========================================================================
	// USER_CTRL (0x6A) — Opcional pero recomendado al usar SPI
	// -------------------------------------------------------------------------
	//  bit 4  I2C_IF_DIS : 1 = deshabilita la interfaz I2C y deja solo SPI.
	//  La hoja de datos recomienda activarlo cuando el sensor se usa por SPI.
	//
	//  0x10 = solo SPI
	// =========================================================================
	dato[0] = 0x6A; dato[1] = 0x10;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
 
	// =========================================================================
	// CONFIG (0x1A) — Filtro pasa bajas del GIROSCOPIO (DLPF_CFG, bits 2:0)
	// Válido con FCHOICE_B = 00 en GYRO_CONFIG (0x1B bits 1:0).
	// -------------------------------------------------------------------------
	//  Valor | Ancho de banda | Retraso  | Muestreo interno
	//  0x00  |    250 Hz      |  0.97 ms |  8 kHz
	//  0x01  |    184 Hz      |  2.9  ms |  1 kHz
	//  0x02  |     92 Hz      |  3.9  ms |  1 kHz
	//  0x03  |     41 Hz      |  5.9  ms |  1 kHz   <- actual (lazo de 250 Hz)
	//  0x04  |     20 Hz      |  9.9  ms |  1 kHz
	//  0x05  |     10 Hz      | 17.85 ms |  1 kHz
	//  0x06  |      5 Hz      | 33.48 ms |  1 kHz
	//  0x07  |   3600 Hz      |  0.17 ms |  8 kHz
	//
	//  Con FCHOICE_B distinto de 00 el DLPF se ignora:
	//    FCHOICE_B = 01 o 11 : 8800 Hz, 0.064 ms, 32 kHz
	//    FCHOICE_B = 10      : 3600 Hz, 0.11  ms, 32 kHz
	//
	//  Otros bits de este registro:
	//    bit 6     FIFO_MODE    : 1 = no sobrescribir la FIFO cuando se llena
	//    bits 5:3  EXT_SYNC_SET : sincronización externa por pin FSYNC (0 = desactivada)
	// =========================================================================
	dato[0] = 0x1A; dato[1] = 0x04;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
 
	// =========================================================================
	// GYRO_CONFIG (0x1B) — Rango del GIROSCOPIO (GYRO_FS_SEL, bits 4:3)
	// -------------------------------------------------------------------------
	//  Valor | Rango        | Sensibilidad  (MPU6500_GYRO_ESCALA)
	//  0x00  | ±250  °/s    | 131.0 LSB/(°/s)
	//  0x08  | ±500  °/s    |  65.5 LSB/(°/s)   <- actual
	//  0x10  | ±1000 °/s    |  32.8 LSB/(°/s)
	//  0x18  | ±2000 °/s    |  16.4 LSB/(°/s)
	//
	//  Otros bits:
	//    bits 7:5  XG/YG/ZG_ST : autoprueba por eje (0 = normal)
	//    bits 1:0  FCHOICE_B   : 00 = usar el DLPF de CONFIG (normal)
	//
	//  Si cambias el rango, cambia también MPU6500_GYRO_ESCALA.
	// =========================================================================
	dato[0] = 0x1B; dato[1] = 0x08;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
 
	// =========================================================================
	// ACCEL_CONFIG (0x1C) — Rango del ACELERÓMETRO (ACCEL_FS_SEL, bits 4:3)
	// -------------------------------------------------------------------------
	//  Valor | Rango  | Sensibilidad  (MPU6500_ACCEL_ESCALA)
	//  0x00  | ±2  g  | 16384 LSB/g
	//  0x08  | ±4  g  |  8192 LSB/g
	//  0x10  | ±8  g  |  4096 LSB/g   <- actual (margen contra saturación por vibración)
	//  0x18  | ±16 g  |  2048 LSB/g
	//
	//  Otros bits:
	//    bits 7:5  XA/YA/ZA_ST : autoprueba por eje (0 = normal)
	//
	//  Si cambias el rango, cambia también MPU6500_ACCEL_ESCALA y recalibra.
	// =========================================================================
	dato[0] = 0x1C; dato[1] = 0x10;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
 
	// =========================================================================
	// ACCEL_CONFIG2 (0x1D) — Filtro pasa bajas del ACELERÓMETRO
	// (ACCEL_FCHOICE_B bit 3 = 0, A_DLPF_CFG bits 2:0)
	// -------------------------------------------------------------------------
	//  Valor | Ancho de banda | Retraso  | Muestreo
	//  0x00  |   218.1 Hz     |  1.88 ms |  1 kHz
	//  0x01  |   218.1 Hz     |  1.88 ms |  1 kHz
	//  0x02  |    99   Hz     |  2.88 ms |  1 kHz
	//  0x03  |    44.8 Hz     |  4.88 ms |  1 kHz
	//  0x04  |    21.2 Hz     |  8.87 ms |  1 kHz   <- actual
	//  0x05  |    10.2 Hz     | 16.83 ms |  1 kHz
	//  0x06  |     5.05 Hz    | 32.48 ms |  1 kHz
	//  0x07  |   420   Hz     |  1.38 ms |  1 kHz
	//
	//  0x08 (ACCEL_FCHOICE_B = 1): sin DLPF, 1.13 kHz, 0.75 ms, muestreo 4 kHz
	//
	//  En el acelerómetro el retraso casi no importa: el filtro de orientación
	//  solo lo usa para corregir la deriva lentamente.
	// =========================================================================
	dato[0] = 0x1D; dato[1] = 0x04;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
 
	// =========================================================================
	// SMPLRT_DIV (0x19) — Frecuencia de muestreo de salida
	// -------------------------------------------------------------------------
	//  Frecuencia = 1 kHz / (1 + SMPLRT_DIV)
	//  Solo aplica con FCHOICE_B = 00 y DLPF_CFG entre 1 y 6.
	//
	//  Valor | Frecuencia
	//  0x00  | 1000 Hz   <- actual
	//  0x01  |  500 Hz
	//  0x03  |  250 Hz
	//  0x04  |  200 Hz
	//  0x09  |  100 Hz
	//  0x13  |   50 Hz
	//
	//  Con DLPF activo, leer más lento que esta frecuencia no causa aliasing:
	//  el DLPF ya quitó lo que no se puede representar.
	// =========================================================================
	dato[0] = 0x19; dato[1] = 0x01;
	CS_Bajo();
	HAL_SPI_Transmit(hspi_, dato, 2, HAL_MAX_DELAY);
	CS_Alto();
	HAL_Delay(10);
}

void MPU6500::LeeCrudo(){
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    // Dirección inicial (ACCEL_XOUT_H) + bit de lectura
    tx[0] = 0x3B | 0x80;

    CS_Bajo();
    HAL_SPI_TransmitReceive(hspi_, tx, rx, 15, 5);
    CS_Alto();

    crudo_.ax = static_cast<int16_t>((rx[1] << 8) | rx[2]);
    crudo_.ay = static_cast<int16_t>((rx[3] << 8) | rx[4]);
    crudo_.az = static_cast<int16_t>((rx[5] << 8) | rx[6]);

    crudo_.gx = static_cast<int16_t>((rx[9]  << 8) | rx[10]);
    crudo_.gy = static_cast<int16_t>((rx[11] << 8) | rx[12]);
    crudo_.gz = static_cast<int16_t>((rx[13] << 8) | rx[14]);

}

void MPU6500::LeeProcesado(){
    LeeCrudo();

    float ax = crudo_.ax / MPU6500_ACCEL_ESCALA - accelOffset_[0];
    float ay = crudo_.ay / MPU6500_ACCEL_ESCALA - accelOffset_[1];
    float az = crudo_.az / MPU6500_ACCEL_ESCALA - accelOffset_[2];

    // Matriz completa, mismo patrón que el magnetómetro
    const float axc = matrizAccel_[0][0]*ax + matrizAccel_[0][1]*ay + matrizAccel_[0][2]*az;
    const float ayc = matrizAccel_[1][0]*ax + matrizAccel_[1][1]*ay + matrizAccel_[1][2]*az;
    const float azc = matrizAccel_[2][0]*ax + matrizAccel_[2][1]*ay + matrizAccel_[2][2]*az;

    const float gx = (crudo_.gx - gyroOffset_[0]) / MPU6500_GYRO_ESCALA;
    const float gy = (crudo_.gy - gyroOffset_[1]) / MPU6500_GYRO_ESCALA;
    const float gz = (crudo_.gz - gyroOffset_[2]) / MPU6500_GYRO_ESCALA;

    // Trim de nivel: corrige la inclinación del montaje (acelerómetro y giroscopio)
    const float (&R)[3][3] = rotNivel_;
    procesado_.ax = R[0][0]*axc + R[0][1]*ayc + R[0][2]*azc;
    procesado_.ay = R[1][0]*axc + R[1][1]*ayc + R[1][2]*azc;
    procesado_.az = R[2][0]*axc + R[2][1]*ayc + R[2][2]*azc;

    procesado_.gx = R[0][0]*gx + R[0][1]*gy + R[0][2]*gz;
    procesado_.gy = R[1][0]*gx + R[1][1]*gy + R[1][2]*gz;
    procesado_.gz = R[2][0]*gx + R[2][1]*gy + R[2][2]*gz;

	//Sobre eje x
	procesado_.roll = atan2f(procesado_.ay, procesado_.az) * 180.0f / static_cast<float>(M_PI);
    //Sobre eje y
	procesado_.pitch = atan2f(-procesado_.ax, sqrt((procesado_.ay * procesado_.ay) + (procesado_.az * procesado_.az)))
					  * 180.0f / static_cast<float>(M_PI);
}

void MPU6500::AsignarOffsetsGyro(float ox, float oy, float oz)
{
    gyroOffset_[0] = ox;
    gyroOffset_[1] = oy;
    gyroOffset_[2] = oz;
}

void MPU6500::ObtenerOffsetsGyro(float &ox, float &oy, float &oz) const
{
    ox = gyroOffset_[0];
    oy = gyroOffset_[1];
    oz = gyroOffset_[2];
}

void MPU6500::AsignarOffsetsAccel(float ox, float oy, float oz)
{
    accelOffset_[0] = ox;
    accelOffset_[1] = oy;
    accelOffset_[2] = oz;
}

void MPU6500::ObtenerOffsetsAccel(float &ox, float &oy, float &oz) const
{
    ox = accelOffset_[0];
    oy = accelOffset_[1];
    oz = accelOffset_[2];
}

struct ImuCalibData{
    float gyroOffset[3];
    float accelOffset[3];
    float matrizAccel[3][3];
    float rotNivel[3][3];      // trim de nivel
};

bool MPU6500::GuardarCalibracion()
{
    alignas(4) static ImuCalibData data;

    data.gyroOffset[0] = gyroOffset_[0];
    data.gyroOffset[1] = gyroOffset_[1];
    data.gyroOffset[2] = gyroOffset_[2];

    data.accelOffset[0] = accelOffset_[0];
    data.accelOffset[1] = accelOffset_[1];
    data.accelOffset[2] = accelOffset_[2];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            data.matrizAccel[i][j] = matrizAccel_[i][j];
            data.rotNivel[i][j]    = rotNivel_[i][j];
        }

    return SDcard::Guardar(SDLayout::kImuFilename, SDLayout::kImuMagic, &data, sizeof(data));
}

bool MPU6500::CargarCalibracion()
{
    alignas(4) static ImuCalibData data;
    if(SDcard::Cargar(SDLayout::kImuFilename, SDLayout::kImuMagic, &data, sizeof(data)))
    {
        AsignarOffsetsGyro(data.gyroOffset[0], data.gyroOffset[1], data.gyroOffset[2]);
        AsignarOffsetsAccel(data.accelOffset[0], data.accelOffset[1], data.accelOffset[2]);

        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
            {
                matrizAccel_[i][j] = data.matrizAccel[i][j];
                rotNivel_[i][j]    = data.rotNivel[i][j];
            }

        return true;
    }
    return false;
}

bool MPU6500::CalibracionGyro(uint16_t muestras) {
    // 1. OBLIGATORIO: Limpiar offsets viejos antes de promediar
    gyroOffset_[0] = 0.0f;
    gyroOffset_[1] = 0.0f;
    gyroOffset_[2] = 0.0f;

    int32_t sumX = 0, sumY = 0, sumZ = 0;

    for (uint16_t i = 0; i < muestras; i++) {
        LeeCrudo();
        sumX += crudo_.gx;
        sumY += crudo_.gy;
        sumZ += crudo_.gz;
        HAL_Delay(2);
    }

    // Guardar el promedio RAW de reposo
    gyroOffset_[0] = static_cast<float>(sumX) / muestras;
    gyroOffset_[1] = static_cast<float>(sumY) / muestras;
    gyroOffset_[2] = static_cast<float>(sumZ) / muestras;

    return true;
}

// mpu6500.hpp
void MPU6500::AsignarEscalasAccel(float sx, float sy, float sz)
{
    accelScale_[0] = sx; accelScale_[1] = sy; accelScale_[2] = sz;
}

void MPU6500::AsignarMatrizAccel(float m00,float m01,float m02, float m10,float m11,float m12, float m20,float m21,float m22)
{
    matrizAccel_[0][0]=m00; matrizAccel_[0][1]=m01; matrizAccel_[0][2]=m02;
    matrizAccel_[1][0]=m10; matrizAccel_[1][1]=m11; matrizAccel_[1][2]=m12;
    matrizAccel_[2][0]=m20; matrizAccel_[2][1]=m21; matrizAccel_[2][2]=m22;
}

void MPU6500::ObtenerMatrizAccel(float m[3][3]) const
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            m[i][j] = matrizAccel_[i][j];
}

void MPU6500::AsignarTrim(const float r[9])
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            rotNivel_[i][j] = r[3*i + j];
}

void MPU6500::ObtenerTrim(float r[3][3]) const
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            r[i][j] = rotNivel_[i][j];
}
// bool MPU6500::CapturarPosicionAccel(PosicionAccel pos, AccelPosDato &PosDato, uint16_t muestras) {
//     float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

//     for (uint16_t i = 0; i < muestras; i++) {
//         LeeCrudo();
//         // Capturar valor físico PURO en 'g' sin aplicar offsets en proceso
//         sumX += crudo_.ax / MPU6500_ACCEL_ESCALA;
//         sumY += crudo_.ay / MPU6500_ACCEL_ESCALA;
//         sumZ += crudo_.az / MPU6500_ACCEL_ESCALA;
//         HAL_Delay(2);
//     }

//     PosDato.x = sumX / muestras;
//     PosDato.y = sumY / muestras;
//     PosDato.z = sumZ / muestras;

//     return true;
// }


// bool MPU6500::Calibracion6Posiciones(const AccelPosDato pos[6]) {
//     // +X = frente del dron. FRONT apunta +X hacia abajo → mínimo.
//     float maxX = pos[static_cast<int>(PosicionAccel::ATRAS)].x;
//     float minX = pos[static_cast<int>(PosicionAccel::FRENTE)].x;

//     // +Y = tu izquierda. LEFT apunta +Y hacia abajo → mínimo.
//     float maxY = pos[static_cast<int>(PosicionAccel::DERECHA)].y;
//     float minY = pos[static_cast<int>(PosicionAccel::IZQUIERDA)].y;

//     // Z sin cambios — ya estaba correcto
//     float maxZ = pos[static_cast<int>(PosicionAccel::NIVELADO)].z;
//     float minZ = pos[static_cast<int>(PosicionAccel::INVERTIDO)].z;

//     accelOffset_[0] = (maxX + minX) / 2.0f;
//     accelOffset_[1] = (maxY + minY) / 2.0f;
//     accelOffset_[2] = (maxZ + minZ) / 2.0f;

//     accelScale_[0] = 2.0f / (maxX - minX);
//     accelScale_[1] = 2.0f / (maxY - minY);
//     accelScale_[2] = 2.0f / (maxZ - minZ);

//     return true;
// }

// bool MPU6500::CalibracionSimplexNivel(uint16_t muestras)
// {
//     float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

//     for (uint16_t i = 0; i < muestras; i++) {
//         LeeCrudo();
//         sumX += crudo_.ax / MPU6500_ACCEL_ESCALA;
//         sumY += crudo_.ay / MPU6500_ACCEL_ESCALA;
//         sumZ += crudo_.az / MPU6500_ACCEL_ESCALA;
//         HAL_Delay(2);
//     }

//     float avgX = sumX / muestras;
//     float avgY = sumY / muestras;
//     float avgZ = sumZ / muestras;

//     // Nivelado y quieto: X debe leer 0, Y debe leer 0, Z debe leer +1g
//     // (confirmado: +Z hacia arriba = +1g, según tu convención física real)
//     accelOffset_[0] = avgX;
//     accelOffset_[1] = avgY;
//     accelOffset_[2] = avgZ - 1.0f;

//     // Este método usa UNA sola posición — no puede determinar escala real
//     // por eje (eso requiere múltiples orientaciones). Se deja sin corregir,
//     // a cambio de eliminar por completo el riesgo de mala orientación
//     // durante la captura, que fue la causa del bug anterior.
//     accelScale_[0] = 1.0f;
//     accelScale_[1] = 1.0f;
//     accelScale_[2] = 1.0f;

//     return true;
// }
// mpu6500.cpp

// bool MPU6500::CalibracionMinCuadrados(const float muestrasX[], const float muestrasY[], const float muestrasZ[], uint16_t contador)
// {
//     if (contador < 60) return false;   // ~6-10 orientaciones, varias muestras cada una

//     // Sistema lineal 6x6: A*x² + B*y² + C*z² + G*x + H*y + I*z = 1
//     // (elipsoide alineado a los ejes — sin términos cruzados, porque el
//     // error de fábrica del acelerómetro es POR EJE, no rotado como el
//     // soft-iron del magnetómetro)
//     double M[6][6] = {};
//     double b[6] = {};

//     double sx=0,sy=0,sz=0, sx2=0,sy2=0,sz2=0, sxy=0,sxz=0,syz=0;
//     double sx3=0,sy3=0,sz3=0, sx2y=0,sx2z=0,sxy2=0,sy2z=0,sxz2=0,syz2=0;
//     double sx4=0,sy4=0,sz4=0, sx2y2=0,sx2z2=0,sy2z2=0;
//     //double sx2yy=0; // no usado, placeholder evitado abajo

//     for(uint16_t i=0;i<contador;i++)
//     {
//         double x=muestrasX[i], y=muestrasY[i], z=muestrasZ[i];
//         double x2=x*x, y2=y*y, z2=z*z;
//         sx+=x; sy+=y; sz+=z;
//         sx2+=x2; sy2+=y2; sz2+=z2;
//         sxy+=x*y; sxz+=x*z; syz+=y*z;
//         sx3+=x2*x; sy3+=y2*y; sz3+=z2*z;
//         sx2y+=x2*y; sx2z+=x2*z; sxy2+=x*y2; sy2z+=y2*z; sxz2+=x*z2; syz2+=y*z2;
//         sx4+=x2*x2; sy4+=y2*y2; sz4+=z2*z2;
//         sx2y2+=x2*y2; sx2z2+=x2*z2; sy2z2+=y2*z2;
//     }

//     // Fila por cada termino [x2,y2,z2,x,y,z], columnas = [A,B,C,G,H,I]
//     M[0][0]=sx4;   M[0][1]=sx2y2; M[0][2]=sx2z2; M[0][3]=sx3;  M[0][4]=sx2y; M[0][5]=sx2z;
//     M[1][1]=sy4;   M[1][2]=sy2z2; M[1][3]=sxy2;  M[1][4]=sy3;  M[1][5]=sy2z;
//     M[2][2]=sz4;   M[2][3]=sxz2;  M[2][4]=syz2;  M[2][5]=sz3;
//     M[3][3]=sx2;   M[3][4]=sxy;   M[3][5]=sxz;
//     M[4][4]=sy2;   M[4][5]=syz;
//     M[5][5]=sz2;
//     for(int i=0;i<6;i++) for(int j=i+1;j<6;j++) M[j][i]=M[i][j];

//     b[0]=sx2; b[1]=sy2; b[2]=sz2; b[3]=sx; b[4]=sy; b[5]=sz;

//     // Eliminación gaussiana 6x6 (mismo patrón ya verificado con el 9x9 del magnetómetro)
//     double sol[6];
//     for(int col=0; col<6; col++)
//     {
//         int piv=col; double mayor=fabs(M[col][col]);
//         for(int f=col+1; f<6; f++) if(fabs(M[f][col])>mayor){mayor=fabs(M[f][col]); piv=f;}
//         if(mayor < 1e-9) return false;
//         if(piv!=col){ for(int k=0;k<6;k++){double t=M[col][k];M[col][k]=M[piv][k];M[piv][k]=t;} double t=b[col];b[col]=b[piv];b[piv]=t; }
//         for(int f=col+1; f<6; f++)
//         {
//             double factor=M[f][col]/M[col][col];
//             for(int k=col;k<6;k++) M[f][k]-=factor*M[col][k];
//             b[f]-=factor*b[col];
//         }
//     }
//     for(int f=5; f>=0; f--)
//     {
//         double suma=b[f];
//         for(int k=f+1;k<6;k++) suma-=M[f][k]*sol[k];
//         sol[f]=suma/M[f][f];
//     }

//     double A=sol[0],B=sol[1],C=sol[2],G=sol[3],H=sol[4],I=sol[5];
//     if(fabs(A)<1e-9 || fabs(B)<1e-9 || fabs(C)<1e-9) return false;

//     double offX=-G/(2*A), offY=-H/(2*B), offZ=-I/(2*C);
//     double K = 1.0 + G*G/(4*A) + H*H/(4*B) + I*I/(4*C);
//     if(K < 1e-9) return false;

//     accelOffset_[0]=static_cast<float>(offX);
//     accelOffset_[1]=static_cast<float>(offY);
//     accelOffset_[2]=static_cast<float>(offZ);
//     accelScale_[0]=static_cast<float>(sqrt(A/K));
//     accelScale_[1]=static_cast<float>(sqrt(B/K));
//     accelScale_[2]=static_cast<float>(sqrt(C/K));

//     return true;
// }
