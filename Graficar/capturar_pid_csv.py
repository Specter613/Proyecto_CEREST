"""
Captura en vivo tu debug "PID,..." (de TareaMotor) y lo guarda
directo como un CSV con columnas nombradas, listo para graficar_sintonia_pid.py.

A diferencia de capturar_log.py (que guarda texto crudo), este parsea
cada línea al vuelo y descarta lo que no es un PID válido — así el CSV
queda limpio desde el inicio, sin líneas cortadas por el USB.

Formato esperado de la línea (ver TareaMotor en puente.cpp):
    PID,tick,rollSP,rollAct,rollErr,rollP,rollI,rollD,rollOut,
        pitchSP,pitchAct,pitchErr,pitchP,pitchI,pitchD,pitchOut,
        yawSP,yawAct,yawErr,yawP,yawI,yawD,yawOut

Uso:
    pip install pyserial --break-system-packages
    python3 capturar_pid_csv.py --port /dev/ttyACM0 --segundos 20 --out prueba1.csv
"""

import argparse
import csv
import re
import time
import serial

COLUMNAS = [
    "tick_ms",
    "roll_sp", "roll_act", "roll_err", "roll_P", "roll_I", "roll_D", "roll_out",
    "pitch_sp", "pitch_act", "pitch_err", "pitch_P", "pitch_I", "pitch_D", "pitch_out",
    "yaw_sp", "yaw_act", "yaw_err", "yaw_P", "yaw_I", "yaw_D", "yaw_out",
]

RE_LINEA = re.compile(r"PID," + r"(-?[\d.]+),"*(len(COLUMNAS)-1) + r"(-?[\d.]+)")


def parsear_linea(linea):
    m = RE_LINEA.search(linea)
    if not m:
        return None
    return [float(x) for x in m.groups()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--segundos", type=float, default=20.0)
    ap.add_argument("--out", default="prueba_pid.csv")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    t0 = time.time()
    n_ok, n_desc = 0, 0

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNAS)
        print(f"Capturando {args.segundos}s de {args.port}... (mueve/prueba el dron ahora)")
        while time.time() - t0 < args.segundos:
            cruda = ser.readline().decode("utf-8", errors="ignore")
            if not cruda:
                continue
            datos = parsear_linea(cruda)
            if datos is not None:
                writer.writerow(datos)
                n_ok += 1
            elif cruda.strip():
                n_desc += 1

    ser.close()
    print(f"Listo: {n_ok} filas válidas guardadas en {args.out} ({n_desc} líneas descartadas)")


if __name__ == "__main__":
    main()

## python3 capturar_pid_csv.py --port /dev/ttyACM0 --segundos 20 --out prueba1.csv