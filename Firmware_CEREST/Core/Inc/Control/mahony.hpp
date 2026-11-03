#ifndef INC_CONTROL_MAHONY_HPP_
#define INC_CONTROL_MAHONY_HPP_

#include <cmath>
#include <cstdint>

class MahonyOrientacion
{
public:
    MahonyOrientacion(float kp, float ki);

    void Update6DOF(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);

    void UpdateMag(float mx, float my, float mz);

    void GetQuaternion(float &q0, float &q1, float &q2, float &q3) const;
    void GetEuler(float &roll_deg, float &pitch_deg, float &yaw_deg) const;

    uint32_t GetRechazosAccel() const { return rechazosAccel_; }
    uint32_t GetRechazosYaw()   const { return rechazosMag_; }

private:
    void QuatNormalize();
    void IntegrarConGyroCorregido(float gx, float gy, float gz, float dt);
    void InicializarDesdeAccel(float ax, float ay, float az);
    void InicializarYawDesdeMag(float mx, float my, float mz);

    float kp_, ki_;
    float q_[4];
    float eInt_[3];
    float mxUlt_ = 0.0f, myUlt_ = 0.0f, mzUlt_ = 0.0f;
    bool magDisponible_ = false;

    // Valores por defecto puestos AQUI, en el header, no en la lista
    // del constructor -- evita cualquier ambiguedad de orden de
    // inicializacion de miembros en C++.
    bool rollPitchSembrado_ = false;
    bool yawSembrado_ = false;

    uint32_t rechazosAccel_ = 0;
    uint32_t rechazosMag_ = 0;
};

#endif