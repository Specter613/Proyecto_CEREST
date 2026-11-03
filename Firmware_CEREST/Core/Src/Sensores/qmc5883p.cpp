#include "Sensores/qmc5883p.hpp"
#include "Proceso/sd.hpp"

QMC5883P::QMC5883P(I2C_HandleTypeDef *hi2c) : hi2c_(hi2c) {}

void QMC5883P::Inicio()
{
    uint8_t data;

    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x00; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x02; HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);

    data = 0x06; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x29, 1, &data, 1, HAL_MAX_DELAY); // axis sign
    data = 0x08; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x0B, 1, &data, 1, HAL_MAX_DELAY); // range 8G + set/reset
    data = 0xC9; HAL_I2C_Mem_Write(hi2c_, QMC_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY); // continuous, 200Hz
}

bool QMC5883P::LeeCrudo()
{
    uint8_t reg = 0x01;
    uint8_t buffer[6];

    if(HAL_I2C_Master_Transmit(hi2c_, QMC_ADDR, &reg, 1, 5) != HAL_OK)
        return false;

    if(HAL_I2C_Master_Receive(hi2c_, QMC_ADDR, buffer, 6, 5) != HAL_OK)
        return false;

    crudo_.mx = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    crudo_.my = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    crudo_.mz = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
    return true;
}

// bool QMC5883P::LeeProcesado()
// {
//     if(!LeeCrudo()) return false;

//     float mx = crudo_.mx / SCALA;
//     float my = crudo_.my / SCALA;
//     float mz = crudo_.mz / SCALA;

//     // Hard iron
//     mx -= offsetX_;
//     my -= offsetY_;
//     mz -= offsetZ_;

//     // Soft iron
//     mx *= scaleX_;
//     my *= scaleY_;
//     mz *= scaleZ_;

//     procesado_.mx = my;  //
//     procesado_.my = -mx;  // 
//     procesado_.mz = -mz;  // 
//     return true;
// }

bool QMC5883P::LeeProcesado()
{
    if(!LeeCrudo()) return false;

    float mx = crudo_.mx / SCALA;
    float my = crudo_.my / SCALA;
    float mz = crudo_.mz / SCALA;

    // Hard iron
    mx -= offsetX_;
    my -= offsetY_;
    mz -= offsetZ_;

    // Soft iron — matriz completa 3x3, reemplaza los escalares scaleX_/Y_/Z_
    float cx = matrizCorreccion_[0][0]*mx + matrizCorreccion_[0][1]*my + matrizCorreccion_[0][2]*mz;
    float cy = matrizCorreccion_[1][0]*mx + matrizCorreccion_[1][1]*my + matrizCorreccion_[1][2]*mz;
    float cz = matrizCorreccion_[2][0]*mx + matrizCorreccion_[2][1]*my + matrizCorreccion_[2][2]*mz;

    // Mapeo de ejes físico — MISMO que ya verificamos y confirmamos antes,
    // ahora aplicado sobre los valores ya corregidos por la matriz
    // procesado_.mx = cy;    // Gx (frente) = My corregido
    // procesado_.my = -cx;   // Gy (izquierda) = -Mx corregido
        // procesado_.mz = -cz;   // Z invertido, confirmado en pruebas anteriores
        procesado_.mx = cy;    // Gx (frente) = My corregido
        procesado_.my = -cx;   // Gy (izquierda) = -Mx corregido
        procesado_.mz = cz;   // Z invertido, confirmado en pruebas anteriores
        return true;
    }
void QMC5883P::ObtenerMatrizCorreccion(float m[3][3]) const
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            m[i][j] = matrizCorreccion_[i][j];
}

float QMC5883P::YawCrudoAtan2(float roll_deg, float pitch_deg) const
{
    float r = roll_deg  * (static_cast<float>(M_PI) / 180.0f);
    float p = pitch_deg * (static_cast<float>(M_PI) / 180.0f);

    float cosR = cosf(r), sinR = sinf(r);
    float cosP = cosf(p), sinP = sinf(p);

    float mx = procesado_.mx;
    float my = procesado_.my;
    float mz = procesado_.mz;
    // Sistema de Coordenadas Ned


    //float mxh = mx * cosP + mz * sinP;
    //float myh = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

    float mxh = mx*cosP + my*sinR*sinP + mz*cosR*sinP;
    float myh = my*cosR - mz*sinR;

    return atan2f(-myh, mxh) * (180.0f / static_cast<float>(M_PI));   // [-180, 180] 
}

    // Declinación magnética de Guadalajara, Jalisco — VERIFICAR con NOAA
    // (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) usando
    // tus coordenadas exactas antes de vuelo real. Positivo = Este.
void QMC5883P::NorteMagnetico(float roll_deg, float pitch_deg)
{
    float r = roll_deg  * (static_cast<float>(M_PI) / 180.0f);
    float p = pitch_deg * (static_cast<float>(M_PI) / 180.0f);

    float cosR = cosf(r), sinR = sinf(r);
    float cosP = cosf(p), sinP = sinf(p);

    // Proyección con compensación de inclinación corregida (Fórmulas de Euler estándar)
    float Xh = procesado_.mx * cosP + procesado_.my * sinR * sinP - procesado_.mz * cosR * sinP;
    float Yh = procesado_.my * cosR - procesado_.mz * sinR;

    // Cálculo del ángulo en grados [-180, 180]
    float heading = atan2f(Yh, Xh) * (180.0f / static_cast<float>(M_PI));

    // Aplicar Declinación Magnética
    heading += DeclinacionMagnetica;

    // Normalizar a rango [0, 360)
    while (heading < 0.0f)   heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    procesado_.heading = heading;
}

void QMC5883P::AsignarOffsets(float ox, float oy, float oz)
{
    offsetX_ = ox;
    offsetY_ = oy;
    offsetZ_ = oz;
}

void QMC5883P::ObtenerOffsets(float &ox, float &oy, float &oz) const
{
    ox = offsetX_;
    oy = offsetY_;
    oz = offsetZ_;
}

void QMC5883P::Calibracion(int16_t &min_x, int16_t &max_x,
                           int16_t &min_y, int16_t &max_y,
                           int16_t &min_z, int16_t &max_z)
{
    if(!LeeCrudo()) return;

    if(crudo_.mx < min_x) min_x = crudo_.mx;
    if(crudo_.mx > max_x) max_x = crudo_.mx;
    if(crudo_.my < min_y) min_y = crudo_.my;
    if(crudo_.my > max_y) max_y = crudo_.my;
    if(crudo_.mz < min_z) min_z = crudo_.mz;
    if(crudo_.mz > max_z) max_z = crudo_.mz;
}

// struct MagCalibData
// {
//     float offsetX, offsetY, offsetZ;
// };

// bool QMC5883P::GuardarCalibracion()
// {
//     // 'alignas(4) static' saca la estructura de la Pila
//     alignas(4) static MagCalibData data;
//     data.offsetX = offsetX_;
//     data.offsetY = offsetY_;
//     data.offsetZ = offsetZ_;

//     // Se añadió el 'return' faltante que provocaba el crash
//     return SDcard::Guardar(SDLayout::kMagFilename, SDLayout::kMagMagic, &data, sizeof(data));
// }

// bool QMC5883P::CargarCalibracion()
// {
//     alignas(4) static MagCalibData data;
//     if(SDcard::Cargar(SDLayout::kMagFilename, SDLayout::kMagMagic, &data, sizeof(data)))
//     {
//         AsignarOffsets(data.offsetX, data.offsetY, data.offsetZ);
//         return true;
//     }
//     return false;
// }

struct MagCalibData
{
    float offsetX, offsetY, offsetZ;
    float matriz[3][3];   // <-- AGREGADO
};

bool QMC5883P::GuardarCalibracion()
{
    alignas(4) static MagCalibData data;
    data.offsetX = offsetX_;
    data.offsetY = offsetY_;
    data.offsetZ = offsetZ_;

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            data.matriz[i][j] = matrizCorreccion_[i][j];   // <-- AGREGADO

    return SDcard::Guardar(SDLayout::kMagFilename, SDLayout::kMagMagic, &data, sizeof(data));
}

bool QMC5883P::CargarCalibracion()
{
    alignas(4) static MagCalibData data;
    if(SDcard::Cargar(SDLayout::kMagFilename, SDLayout::kMagMagic, &data, sizeof(data)))
    {
        AsignarOffsets(data.offsetX, data.offsetY, data.offsetZ);

        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                matrizCorreccion_[i][j] = data.matriz[i][j];   // <-- AGREGADO

        return true;
    }
    return false;
}

// qmc5883p.cpp
void QMC5883P::SetMatrizCorreccion(float m00,float m01,float m02, float m10,float m11,float m12, float m20,float m21,float m22)
{
    matrizCorreccion_[0][0]=m00; matrizCorreccion_[0][1]=m01; matrizCorreccion_[0][2]=m02;
    matrizCorreccion_[1][0]=m10; matrizCorreccion_[1][1]=m11; matrizCorreccion_[1][2]=m12;
    matrizCorreccion_[2][0]=m20; matrizCorreccion_[2][1]=m21; matrizCorreccion_[2][2]=m22;
}

// bool QMC5883P::CalibracionMinimosCuadrados(const float samplesX[], const float samplesY[], const float samplesZ[], uint16_t count)
// {
//     if (count < 100) return false;

//     static constexpr double LSB_SCALE = 3000.0;   // normalización interna, evita desbordamientos
//     double sumX=0,sumY=0,sumZ=0, sumX2=0,sumY2=0,sumZ2=0, sumXY=0,sumXZ=0,sumYZ=0;
//     double sumX3=0,sumY3=0,sumZ3=0;
//     double sumX1Y2=0,sumX1Z2=0,sumY1X2=0,sumY1Z2=0,sumZ1X2=0,sumZ1Y2=0;

//     for (uint16_t i = 0; i < count; i++) {
//         double x = samplesX[i] / LSB_SCALE;
//         double y = samplesY[i] / LSB_SCALE;
//         double z = samplesZ[i] / LSB_SCALE;
//         double x2=x*x, y2=y*y, z2=z*z;
//         sumX+=x; sumY+=y; sumZ+=z;
//         sumX2+=x2; sumY2+=y2; sumZ2+=z2;
//         sumXY+=x*y; sumXZ+=x*z; sumYZ+=y*z;
//         sumX3+=x2*x; sumY3+=y2*y; sumZ3+=z2*z;
//         sumX1Y2+=x*y2; sumX1Z2+=x*z2;
//         sumY1X2+=y*x2; sumY1Z2+=y*z2;
//         sumZ1X2+=z*x2; sumZ1Y2+=z*y2;
//     }

//     double N = count;
//     double A1 = 2*(sumX2-(sumX*sumX)/N);
//     double B1 = 2*(sumXY-(sumX*sumY)/N);
//     double C1 = 2*(sumXZ-(sumX*sumZ)/N);
//     double D1 = (sumX3+sumX1Y2+sumX1Z2)-(sumX*(sumX2+sumY2+sumZ2))/N;
//     double A2=B1, B2=2*(sumY2-(sumY*sumY)/N), C2=2*(sumYZ-(sumY*sumZ)/N);
//     double D2 = (sumY3+sumY1X2+sumY1Z2)-(sumY*(sumX2+sumY2+sumZ2))/N;
//     double A3=C1, B3=C2, C3=2*(sumZ2-(sumZ*sumZ)/N);
//     double D3 = (sumZ3+sumZ1X2+sumZ1Y2)-(sumZ*(sumX2+sumY2+sumZ2))/N;

//     double det = A1*(B2*C3-B3*C2) - B1*(A2*C3-A3*C2) + C1*(A2*B3-A3*B2);
//     if (fabs(det) < 1e-9) return false;   // rotación insuficiente en 3D

//     // FIX DE UNIDADES: se queda en unidades "crudo/LSB_SCALE" — NO se
//     // multiplica de vuelta por LSB_SCALE. Como Process() usa SCALE=1000
//     // (no 3000), reescalamos aquí para que coincida exactamente.
//     double ox = (D1*(B2*C3-B3*C2) - B1*(D2*C3-D3*C2) + C1*(D2*B3-D3*B2)) / det;
//     double oy = (A1*(D2*C3-D3*C2) - D1*(A2*C3-A3*C2) + C1*(A2*D3-A3*D2)) / det;
//     double oz = (A1*(B2*D3-B3*D2) - B1*(A2*D3-A3*D2) + D1*(A2*B3-A3*B2)) / det;

//     offsetX_ = static_cast<float>(ox * (LSB_SCALE / SCALA));   // convierte de escala 3000 a escala 1000
//     offsetY_ = static_cast<float>(oy * (LSB_SCALE / SCALA));
//     offsetZ_ = static_cast<float>(oz * (LSB_SCALE / SCALA));

//     // --- Calidad del ajuste: qué tan constante es la magnitud tras corregir ---
//     // Coeficiente de variación alto = probable soft-iron significativo sin corregir.
//     double sumaRadio = 0.0, sumaRadio2 = 0.0;
//     for (uint16_t i = 0; i < count; i++) {
//         double dx = samplesX[i]/SCALA - offsetX_;
//         double dy = samplesY[i]/SCALA - offsetY_;
//         double dz = samplesZ[i]/SCALA - offsetZ_;
//         double r = sqrt(dx*dx + dy*dy + dz*dz);
//         sumaRadio += r;
//         sumaRadio2 += r*r;
//     }
//     double radioPromedio = sumaRadio / count;
//     double varianza = (sumaRadio2/count) - (radioPromedio*radioPromedio);
//     double desviacion = sqrt(varianza > 0 ? varianza : 0);
//     calidadAjuste_ = static_cast<float>(radioPromedio > 1e-6 ? (desviacion / radioPromedio) : 1.0);

//     return true;
// }