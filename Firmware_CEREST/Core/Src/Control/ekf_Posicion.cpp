/*
 * ekf_posicion.cpp — complementario acelerómetro + flujo óptico (ec. 12.2 del libro)
 */
#include "Control/ekf_Posicion.hpp"
#include <cmath>

static constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;
static constexpr float kG       = 9.81f;

// Sentido del yaw de Mahony: +1 si aumenta al girar ANTIHORARIO visto desde arriba,
// −1 si aumenta en sentido HORARIO (convención de brújula)
static constexpr float kSignoYaw = 1.0f;

// Posición del sensor respecto a la IMU (m): X al frente, Y a la izquierda
static constexpr float kSensorOffsetX = 0.0274f;
static constexpr float kSensorOffsetY = 0.0f;

void EkfPosicion::Predecir(float rollDeg, float pitchDeg, float yawDeg,
                           float ax, float ay, float az,
                           float gx, float gy, float gz, float dt)
{
    roll_ = rollDeg; pitch_ = pitchDeg; yaw_ = kSignoYaw * yawDeg;
    sumaGx_ += gx; sumaGy_ += gy; sumaGz_ += gz;
    nGiro_++;
    dt_ += dt;

    if (kUsarAccel) {
        // Aceleración al mundo: R = Rz(yaw)·Ry(pitch)·Rx(roll), filas X y Y (la gravedad está en Z)
        const float cr = std::cos(rollDeg * kDeg2Rad),  sr = std::sin(rollDeg * kDeg2Rad);
        const float cp = std::cos(pitchDeg * kDeg2Rad), sp = std::sin(pitchDeg * kDeg2Rad);
        const float cy = std::cos(yaw_ * kDeg2Rad),     sy = std::sin(yaw_ * kDeg2Rad);
        const float awx = (cy*cp) * ax + (cy*sp*sr - sy*cr) * ay + (cy*sp*cr + sy*sr) * az;
        const float awy = (sy*cp) * ax + (sy*sp*sr + cy*cr) * ay + (sy*sp*cr - cy*sr) * az;
        vx_ += awx * kG * dt;
        vy_ += awy * kG * dt;
    }

    px_ += vx_ * dt;
    py_ += vy_ * dt;
}

void EkfPosicion::CorregirFlujo(float flujoX, float flujoY, float distancia, bool datoValido)
{
    // Giro promedio entre lecturas (rad/s)
    const float n = (nGiro_ > 0) ? static_cast<float>(nGiro_) : 1.0f;
    const float wxAct = sumaGx_ / n * kDeg2Rad;
    const float wyAct = sumaGy_ / n * kDeg2Rad;
    const float wzAct = sumaGz_ / n * kDeg2Rad;
    sumaGx_ = sumaGy_ = sumaGz_ = 0; nGiro_ = 0;
    const float dt = dt_;
    dt_ = 0;

    // Ganancia del complementario para este intervalo
    float k = kWFlujo * dt;
    if (k > 1.0f) k = 1.0f;

    // Retraso de una lectura (como en el libro)
    const float roll = rollAnt_ * kDeg2Rad, pitch = pitchAnt_ * kDeg2Rad, yaw = yawAnt_ * kDeg2Rad;
    const float wx = wxAnt_, wy = wyAnt_, wz = wzAnt_;
    rollAnt_ = roll_; pitchAnt_ = pitch_; yawAnt_ = yaw_;
    wxAnt_ = wxAct; wyAnt_ = wyAct; wzAnt_ = wzAct;

    if (!datoValido || distancia < kAlturaMin || dt <= 0.0f) {
        rechazosFlujo_++;
        vx_ -= k * vx_;
        vy_ -= k * vy_;
        return;
    }

    // Altura perpendicular al suelo
    const float cosInclin = std::cos(roll) * std::cos(pitch);
    const float h = distancia * cosInclin;
    pz_ = h;

    // 1. Compensación de rotaciones de roll y pitch (ec. 12.2 en el centro de la imagen)
    float vBodyX = flujoX * cosInclin + (wy * h);
    float vBodyY = flujoY * cosInclin - (wx * h);

    // 2. Brazo del sensor en yaw: v_centro = v_sensor − ω × r
    vBodyX += wz * kSensorOffsetY;
    vBodyY -= wz * kSensorOffsetX;

    // 3. Proyección al mundo con la matriz de rotación completa (vBodyZ ≈ 0)
    const float cr = std::cos(roll),  sr = std::sin(roll);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const float cy = std::cos(yaw),   sy = std::sin(yaw);

    const float vxFlujo = (cy*cp) * vBodyX + (cy*sp*sr - sy*cr) * vBodyY;
    const float vyFlujo = (sy*cp) * vBodyX + (sy*sp*sr + cy*cr) * vBodyY;

    // Complementario: el flujo corrige la velocidad del mundo
    vx_ += k * (vxFlujo - vx_);
    vy_ += k * (vyFlujo - vy_);
}