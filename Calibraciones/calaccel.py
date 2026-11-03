"""
calibrar_accel_6pos.py — Calibración del acelerómetro en 6 posiciones + trim de nivel,
al estilo de ArduPilot ("Calibrate Accel" + "Calibrate Level").

Usa los comandos del firmware:
    STREAMACCEL   -> el dron manda "ax,ay,az" CRUDOS cada 50 ms
    x             -> detiene el stream
    SETCALACCEL:  -> offset (3) + matriz 3x3 (9), se guarda en la SD
    SETTRIM:      -> rotación de nivel 3x3 (9, por filas), se guarda en la SD

Uso:
    pip install pyserial numpy --break-system-packages
    python3 calibrar_accel_6pos.py
"""

import math
import sys
import time

import numpy as np
import serial

PUERTO = "/dev/ttyACM0"
BAUD = 115200
MUESTRAS_POR_POSICION = 100   # a 50 ms c/u: 5 s por posición
ACCEL_ESCALA = 4096.0         # DEBE coincidir con MPU6500_ACCEL_ESCALA (±8 g)
MAX_STD_G = 0.02              # g: si la lectura varía más, el dron se movió
MAX_INCLINACION_CARA = 0.35   # g fuera del eje principal (~20°): cara mal apoyada
MAX_INCLINACION_NIVEL = 10.0  # °: para el nivel, la superficie debe estar casi plana

POSICIONES = [
    ("NIVELADO",        "En su posición normal (patas abajo), sobre una superficie plana"),
    ("INVERTIDO",       "De cabeza (patas arriba)"),
    ("NARIZ ABAJO",     "Apoyado sobre el frente, con la nariz apuntando al suelo"),
    ("NARIZ ARRIBA",    "Apoyado sobre la cola, con la nariz apuntando al cielo"),
    ("IZQUIERDA ABAJO", "Apoyado sobre su lado izquierdo"),
    ("DERECHA ABAJO",   "Apoyado sobre su lado derecho"),
]
NOMBRE_EJE = "XYZ"


def capturar(ser, n):
    """Captura n lecturas crudas del acelerómetro y las devuelve en g (n x 3)."""
    ser.reset_input_buffer()
    ser.write(b"STREAMACCEL\r\n")
    time.sleep(0.1)

    muestras = []
    limite = time.time() + n * 0.05 * 3 + 5.0
    while len(muestras) < n:
        if time.time() > limite:
            ser.write(b"x\r\n")
            raise RuntimeError("No llegan datos del dron (¿puerto correcto? ¿firmware con STREAMACCEL?)")
        linea = ser.readline().decode(errors="ignore").strip()
        partes = linea.split(",")
        if len(partes) != 3:
            continue
        try:
            muestras.append([float(p) for p in partes])
        except ValueError:
            continue

    ser.write(b"x\r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()   # descarta el menú que imprime el comando "x"
    return np.array(muestras) / ACCEL_ESCALA


def capturar_quieto(ser, texto):
    """Espera ENTER, captura y repite mientras el dron se haya movido."""
    while True:
        input(texto)
        print("  Capturando, NO lo muevas...", end="", flush=True)
        datos = capturar(ser, MUESTRAS_POR_POSICION)
        variacion = datos.std(axis=0).max()
        if variacion > MAX_STD_G:
            print(f" se movió (variación {variacion:.3f} g). Repite.")
            continue
        print(" listo.")
        return datos.mean(axis=0)


def rotacion_a_vertical(u):
    """Rotación (Rodrigues) que lleva el vector unitario u a (0, 0, 1)."""
    kx, ky = u[1], -u[0]
    s2 = kx * kx + ky * ky
    c = u[2]
    if s2 < 1e-12:
        return np.eye(3)
    f = (1.0 - c) / s2
    return np.array([
        [1.0 - f * ky * ky, f * kx * ky,       ky],
        [f * kx * ky,       1.0 - f * kx * kx, -kx],
        [-ky,               kx,                c],
    ])


def enviar(ser, comando, confirmacion):
    """Manda un comando y devuelve True si llega la confirmación esperada."""
    ser.reset_input_buffer()
    ser.write(comando.encode())
    time.sleep(1.0)
    respuesta = ser.read(200).decode(errors="ignore").strip()
    print(f"  Respuesta: {respuesta}")
    return confirmacion in respuesta


def main():
    ser = serial.Serial(PUERTO, BAUD, timeout=1)
    time.sleep(2)
    ser.write(b"x\r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("=== CALIBRACIÓN DEL ACELERÓMETRO: 6 POSICIONES + TRIM DE NIVEL ===")
    print("Cada cara del dron va una vez hacia abajo, en el orden que quieras.")
    print("En cada paso: acomódalo QUIETO, apóyalo bien y presiona ENTER.\n")

    # ------------------------------------------------------------------
    # 1. Seis posiciones
    # ------------------------------------------------------------------
    promedios, objetivos, usadas = [], [], set()
    for i, (nombre, descripcion) in enumerate(POSICIONES, start=1):
        while True:
            prom = capturar_quieto(ser, f"[{i}/7] {nombre}: {descripcion}.\n  Presiona ENTER cuando esté listo...")

            eje = int(np.argmax(np.abs(prom)))
            signo = 1 if prom[eje] > 0 else -1
            fuera = np.linalg.norm(np.delete(prom, eje))

            if not (0.8 < abs(prom[eje]) < 1.25):
                print(f"  El eje principal mide {prom[eje]:.2f} g (debería ser ~±1 g). Repite.")
                continue
            if fuera > MAX_INCLINACION_CARA:
                print(f"  La cara está muy inclinada ({fuera:.2f} g fuera del eje). Apóyalo mejor y repite.")
                continue
            if (eje, signo) in usadas:
                print(f"  Esa cara ({'+' if signo > 0 else '-'}{NOMBRE_EJE[eje]}) ya se capturó. "
                      f"Pon el dron sobre otra cara y repite.")
                continue

            usadas.add((eje, signo))
            objetivo = np.zeros(3)
            objetivo[eje] = signo
            promedios.append(prom)
            objetivos.append(objetivo)
            print(f"  OK: {'+' if signo > 0 else '-'}{NOMBRE_EJE[eje]} = {prom[eje]:+.3f} g "
                  f"(crudo: {prom[0]:+.3f}, {prom[1]:+.3f}, {prom[2]:+.3f})\n")
            break

    # ------------------------------------------------------------------
    # 2. Ajuste: objetivo = M · (crudo − offset) = M·crudo + c
    # ------------------------------------------------------------------
    X = np.array(promedios)
    T = np.array(objetivos)
    A = np.hstack([X, np.ones((6, 1))])
    W, *_ = np.linalg.lstsq(A, T, rcond=None)
    M = W[:3, :].T
    c = W[3, :]
    offset = -np.linalg.solve(M, c)

    corregidos = (M @ (X - offset).T).T
    errores = np.linalg.norm(corregidos - T, axis=1)
    print("=== Resultado de las 6 posiciones ===")
    print(f"Offset (g): X={offset[0]:+.4f}  Y={offset[1]:+.4f}  Z={offset[2]:+.4f}")
    for k, (cor, err) in enumerate(zip(corregidos, errores)):
        print(f"  Posición {k + 1}: corregido ({cor[0]:+.3f}, {cor[1]:+.3f}, {cor[2]:+.3f})  error {err * 1000:.1f} mg")
    if errores.max() > 0.03:
        print("ADVERTENCIA: error mayor a 30 mg en alguna posición; conviene repetir la calibración.")

    # ------------------------------------------------------------------
    # 3. Trim de nivel: rotación que lleva la gravedad en posición normal a (0, 0, 1)
    # ------------------------------------------------------------------
    print("\n[7/7] NIVEL: pon el dron en su posición normal de vuelo, sobre la superficie")
    print("      MÁS PLANA que tengas (revísala con un nivel o con la app del celular).")
    while True:
        prom = capturar_quieto(ser, "  Presiona ENTER cuando esté listo...")
        g = M @ (prom - offset)
        u = g / np.linalg.norm(g)
        inclinacion = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
        if inclinacion > MAX_INCLINACION_NIVEL:
            print(f"  Inclinado {inclinacion:.1f}° (¿está en posición normal y sobre algo plano?). Repite.")
            continue
        break

    roll_trim = math.degrees(math.atan2(u[1], u[2]))
    pitch_trim = math.degrees(math.atan2(-u[0], math.hypot(u[1], u[2])))
    R = rotacion_a_vertical(u)
    verif = R @ g

    print(f"  Trim: roll {roll_trim:+.2f}°, pitch {pitch_trim:+.2f}°")
    print(f"  Verificación en nivel: ({verif[0]/np.linalg.norm(g):+.3f}, "
          f"{verif[1]/np.linalg.norm(g):+.3f}, {verif[2]/np.linalg.norm(g):+.3f}) (debería ser ~0, 0, +1)")
    if abs(roll_trim) > 5.0 or abs(pitch_trim) > 5.0:
        print("  ADVERTENCIA: trim mayor a 5°; revisa que la placa no esté montada muy inclinada.")

    # ------------------------------------------------------------------
    # 4. Enviar al dron: primero la calibración, luego el trim
    # ------------------------------------------------------------------
    cmd_cal = (f"SETCALACCEL:{offset[0]:.5f},{offset[1]:.5f},{offset[2]:.5f},"
               f"{M[0,0]:.5f},{M[0,1]:.5f},{M[0,2]:.5f},"
               f"{M[1,0]:.5f},{M[1,1]:.5f},{M[1,2]:.5f},"
               f"{M[2,0]:.5f},{M[2,1]:.5f},{M[2,2]:.5f}\r\n")
    cmd_trim = (f"SETTRIM:{R[0,0]:.6f},{R[0,1]:.6f},{R[0,2]:.6f},"
                f"{R[1,0]:.6f},{R[1,1]:.6f},{R[1,2]:.6f},"
                f"{R[2,0]:.6f},{R[2,1]:.6f},{R[2,2]:.6f}\r\n")
    print(f"\n{cmd_cal.strip()}\n{cmd_trim.strip()}")

    if input("¿Enviar y guardar en el dron? (s/n): ").strip().lower() != "s":
        print("Cancelado. No se envió nada.")
        ser.close()
        return

    print("Enviando calibración...")
    ok_cal = enviar(ser, cmd_cal, "ACCEL OK")
    print("Enviando trim...")
    ok_trim = enviar(ser, cmd_trim, "TRIM OK")

    if ok_cal and ok_trim:
        print("\nListo: calibración y trim guardados en la SD.")
    else:
        print("\nADVERTENCIA: falta alguna confirmación. Revisa las respuestas de arriba.")
    ser.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrumpido.")
        sys.exit(1)