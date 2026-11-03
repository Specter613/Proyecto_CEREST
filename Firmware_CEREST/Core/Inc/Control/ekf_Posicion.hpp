/*
 * ekf_posicion.hpp
 *
 * Filtro complementario de velocidad horizontal: acelerómetro (parte rápida) + flujo óptico (parte lenta).
 *
 * 1) Flujo -> velocidad del dron (ejes del cuerpo), con la ec. 12.2 del libro en el centro de la
 *    imagen (x_img = y_img = 0), pasada de píxeles/s a m/s (× h / f):
 *      ẋ_t = ẋ − ( 0·ωx − f·ωy + 0·ωz )  ->  vx = v_mtf_x · cosφcosθ + ωy · h
 *      ẏ_t = ẏ − ( f·ωx − 0·ωy − 0·ωz )  ->  vy = v_mtf_y · cosφcosθ − ωx · h
 *    (en el centro de la imagen los términos de ωz valen 0)
 *      vy −= ωz · kBrazoX                        giro en yaw: el sensor está kBrazoX al frente
 *
 * 2) Al mundo con la rotación de yaw de Mahony.
 *
 * 3) Complementario:
 *      Predecir:  v += a_mundo · dt              (si kUsarAccel)
 *      Flujo:     v += kWFlujo · (v_flujo − v) · dt
 *    kWFlujo [1/s] = frecuencia de cruce: abajo de ella manda el flujo, arriba el acelerómetro.
 */
#ifndef INC_CONTROL_EKF_POSICION_HPP_
#define INC_CONTROL_EKF_POSICION_HPP_

#include <cstdint>

class EkfPosicion {
public:
    EkfPosicion() = default;

    /** TareaGyro: ángulos de GetEuler (°), acelerómetro (g), giro del MPU (°/s), dt (s). */
    void Predecir(float rollDeg, float pitchDeg, float yawDeg,
                  float ax, float ay, float az,
                  float gx, float gy, float gz, float dt);

    /** FlowTask (dato nuevo): velocidad del MTF en m/s (tu driver), distancia en m. */
    void CorregirFlujo(float flujoX, float flujoY, float distancia, bool datoValido);

    void FijarPosicion(float x, float y) { px_ = x; py_ = y; }

    float PosX() const { return px_; }
    float PosY() const { return py_; }
    float PosZ() const { return pz_; }
    float VelX() const { return vx_; }
    float VelY() const { return vy_; }
    float VelZ() const { return 0.0f; }
    float AlturaUsada() const { return pz_; }
    uint32_t RechazosFlujo() const { return rechazosFlujo_; }

    // --- Compatibilidad con Telemetria_USB y la línea POS ---
    float SesgoX() const { return 0.0f; }
    float SesgoY() const { return 0.0f; }
    float QActualX() const { return 0.0f; }
    float RActualX() const { return 0.0f; }
    float QActualY() const { return 0.0f; }
    float RActualY() const { return 0.0f; }
    uint32_t RechazosAltura() const { return 0; }
    void GetVelocidad(float &vx, float &vy, float &vz) const { vx = vx_; vy = vy_; vz = 0.0f; }
    void GetPosicion(float &px, float &py, float &pz) const { px = px_; py = py_; pz = pz_; }
    uint32_t GetRechazosBaro()  const { return 0; }
    uint32_t GetRechazosFlujo() const { return rechazosFlujo_; }

    // ======================= AJUSTES =======================
    static constexpr bool  kUsarAccel  = false;    // true: complementario con acelerómetro
    static constexpr float kWFlujo     = 8.0f;     // 1/s: cruce del complementario (8 1/s ≈ 1.3 Hz)
    static constexpr float kAlturaMin  = 0.08f;    // m: el flujo del MTF-02P mide desde 8 cm
    static constexpr float kBrazoX     = 0.0474f;  // m: sensor al frente de la IMU (giro en yaw)

private:
    // Lo que llega de TareaGyro
    float roll_ = 0, pitch_ = 0, yaw_ = 0;          // ° (Mahony)
    float sumaGx_ = 0, sumaGy_ = 0, sumaGz_ = 0;    // °/s acumulados entre lecturas
    uint32_t nGiro_ = 0;
    float dt_ = 0;                                  // s entre lecturas del sensor

    // Lectura anterior: ángulos y giros (retraso de una lectura, como en el libro)
    float rollAnt_ = 0, pitchAnt_ = 0, yawAnt_ = 0;
    float wxAnt_ = 0, wyAnt_ = 0, wzAnt_ = 0;       // rad/s

    float px_ = 0, py_ = 0, vx_ = 0, vy_ = 0, pz_ = 0;
    uint32_t rechazosFlujo_ = 0;
};

#endif /* INC_CONTROL_EKF_POSICION_HPP_ */