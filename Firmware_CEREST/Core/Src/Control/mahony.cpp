#include "Control/mahony.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

MahonyOrientacion::MahonyOrientacion(float kp, float ki)
    : kp_(kp), ki_(ki) {
    q_[0] = 1.0f; q_[1] = 0.0f; q_[2] = 0.0f; q_[3] = 0.0f;
    eInt_[0] = 0.0f; eInt_[1] = 0.0f; eInt_[2] = 0.0f;
}

void MahonyOrientacion::QuatNormalize() {
    float norm = std::sqrt(q_[0]*q_[0] + q_[1]*q_[1] + q_[2]*q_[2] + q_[3]*q_[3]);
    if (norm < 1e-9f) { q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f; return; }
    float invNorm = 1.0f / norm;
    q_[0] *= invNorm; q_[1] *= invNorm; q_[2] *= invNorm; q_[3] *= invNorm;
}

void MahonyOrientacion::IntegrarConGyroCorregido(float gx, float gy, float gz, float dt) {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    q_[0] += dq0 * dt; q_[1] += dq1 * dt; q_[2] += dq2 * dt; q_[3] += dq3 * dt;
    QuatNormalize();
}

/** @brief Siembra roll/pitch desde la PRIMERA lectura real de
 *  acelerometro -- evita arrancar en identidad y tener que "viajar"
 *  desde ahi hasta la orientacion real (que es justo lo que se veia
 *  en las graficas: convergencia lenta y distinta cada vez, segun
 *  donde estuviera el dron al momento del reset). */
void MahonyOrientacion::InicializarDesdeAccel(float ax, float ay, float az) {
    float norma = std::sqrt(ax*ax + ay*ay + az*az);
    if (norma < 1e-6f) return;
    ax /= norma; ay /= norma; az /= norma;

    float roll_rad  = std::atan2(ay, std::sqrt(ay*ay + az*az));
    float pitch_rad = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    float cr = std::cos(roll_rad*0.5f), sr = std::sin(roll_rad*0.5f);
    float cp = std::cos(pitch_rad*0.5f), sp = std::sin(pitch_rad*0.5f);

    q_[0] = cr*cp; q_[1] = sr*cp; q_[2] = cr*sp; q_[3] = -sr*sp;
    QuatNormalize();
    rollPitchSembrado_ = true;
}

/** @brief Siembra el yaw desde la PRIMERA lectura real de
 *  magnetometro, conservando el roll/pitch que ya tiene q_ en ese
 *  momento (viene de InicializarDesdeAccel, o de varios ciclos de
 *  correccion normal si ya paso tiempo). */
void MahonyOrientacion::InicializarYawDesdeMag(float mx, float my, float mz) {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    // Campo magnético al marco tierra con la orientación actual (mismas fórmulas de hx, hy de Update6DOF)
    float hx = mx*(q0*q0+q1*q1-q2*q2-q3*q3) + 2.0f*my*(q1*q2-q0*q3) + 2.0f*mz*(q1*q3+q0*q2);
    float hy = 2.0f*mx*(q1*q2+q0*q3) + my*(q0*q0-q1*q1+q2*q2-q3*q3) + 2.0f*mz*(q2*q3-q0*q1);
    float yaw = std::atan2(-hy, hx);              // giro necesario para apuntar al norte
    float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    // q = q_yaw ⊗ q_actual (giro alrededor del eje vertical de la tierra)
    q_[0] = cy*q0 - sy*q3;
    q_[1] = cy*q1 - sy*q2;
    q_[2] = cy*q2 + sy*q1;
    q_[3] = cy*q3 + sy*q0;
    QuatNormalize();
    yawSembrado_ = true;
}

void MahonyOrientacion::UpdateMag(float mx, float my, float mz) {
    float norm_m = std::sqrt(mx * mx + my * my + mz * mz);
    if (norm_m < 1e-6f) { rechazosMag_++; return; }
    float invM = 1.0f / norm_m;
    mx *= invM; my *= invM; mz *= invM;

    if (!yawSembrado_) {
        if (!rollPitchSembrado_) return;   // espera a que Update6DOF ya haya sembrado roll/pitch
        InicializarYawDesdeMag(mx, my, mz);
        mxUlt_ = mx; myUlt_ = my; mzUlt_ = mz;
        magDisponible_ = true;
        return;
    }

    mxUlt_ = mx;
    myUlt_ = my;
    mzUlt_ = mz;
    magDisponible_ = true;
}

void MahonyOrientacion::Update6DOF(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float dt) {
    if (!rollPitchSembrado_) {
        InicializarDesdeAccel(ax, ay, az);
        return;   // primera llamada: solo siembra, no corrige todavia
    }

    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];

    float norm_a = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm_a < 1e-6f) { rechazosAccel_++; return; }
    float invA = 1.0f / norm_a;
    ax *= invA; ay *= invA; az *= invA;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    if (magDisponible_) {
        float hx = mxUlt_*(q0*q0+q1*q1-q2*q2-q3*q3) + 2.0f*myUlt_*(q1*q2-q0*q3) + 2.0f*mzUlt_*(q1*q3+q0*q2);
        float hy = 2.0f*mxUlt_*(q1*q2+q0*q3) + myUlt_*(q0*q0-q1*q1+q2*q2-q3*q3) + 2.0f*mzUlt_*(q2*q3-q0*q1);
        float bx = std::sqrt(hx*hx + hy*hy);
        float bz = 2.0f*mxUlt_*(q1*q3-q0*q2) + 2.0f*myUlt_*(q2*q3+q0*q1) + mzUlt_*(q0*q0-q1*q1-q2*q2+q3*q3);

        float wx = bx*(q0*q0+q1*q1-q2*q2-q3*q3) + bz*2.0f*(q1*q3-q0*q2);
        float wy = bx*2.0f*(q1*q2-q0*q3) + bz*2.0f*(q0*q1+q2*q3);
        float wz = bx*2.0f*(q0*q2+q1*q3) + bz*(q0*q0-q1*q1-q2*q2+q3*q3);

        ex += myUlt_*wz - mzUlt_*wy;
        ey += mzUlt_*wx - mxUlt_*wz;
        ez += mxUlt_*wy - myUlt_*wx;
    }

    if (ki_ > 0.0f) {
        eInt_[0] += ex * dt;
        eInt_[1] += ey * dt;
        eInt_[2] += ez * dt;
    } else {
        eInt_[0] = 0.0f; eInt_[1] = 0.0f; eInt_[2] = 0.0f;
    }

    gx += kp_ * ex + ki_ * eInt_[0];
    gy += kp_ * ey + ki_ * eInt_[1];
    gz += kp_ * ez + ki_ * eInt_[2];

    IntegrarConGyroCorregido(gx, gy, gz, dt);
}

void MahonyOrientacion::GetQuaternion(float &q0, float &q1, float &q2, float &q3) const {
    q0 = q_[0]; q1 = q_[1]; q2 = q_[2]; q3 = q_[3];
}

void MahonyOrientacion::GetEuler(float &roll_deg, float &pitch_deg, float &yaw_deg) const {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    float roll  = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    float asinVal = 2.0f * (q0 * q2 - q3 * q1);
    if (asinVal > 1.0f) asinVal = 1.0f;
    if (asinVal < -1.0f) asinVal = -1.0f;
    float pitch = std::asin(asinVal);

    float yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    roll_deg  = roll * (180.0f / M_PI);
    pitch_deg = pitch * (180.0f / M_PI);
    yaw_deg   = yaw * (180.0f / M_PI);
}