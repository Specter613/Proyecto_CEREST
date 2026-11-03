"""
Grafica en TIEMPO REAL Roll, Pitch y Yaw: sensor (acelerómetro/magnetómetro)
contra la salida del Mahony, leyendo directo del puerto serie (USB CDC).

Requiere en el firmware una línea de debug así, impresa periódicamente
(ver ImprimirDebugTiempoReal() en App_Loop):
    RT,<tick_ms>,<rollSensor>,<pitchSensor>,<headingMag>,<rollEkf>,<pitchEkf>,<yawEkf>\r\n

Roll y Pitch "del sensor" salen del acelerómetro (tu atan2f en mpu6500.cpp).
Yaw "del sensor" sale del magnetómetro (heading), porque el acelerómetro
NO puede medir yaw — la gravedad no cambia al girar sobre el eje vertical.

Instalar dependencias:
    pip install pyserial matplotlib numpy --break-system-packages

Listar puertos disponibles:
    python3 graficar_tiempo_real.py --list

Uso:
    python3 graficar_tiempo_real.py --port COM5
    python3 graficar_tiempo_real.py --port /dev/ttyACM0 --ventana 15
"""

import argparse
import re
import threading
import queue
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

RE_LINEA = re.compile(
    r"RT,(\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
)


def parsear_linea(linea):
    """'RT,tick,rollS,pitchS,headingMag,rollE,pitchE,yawE' -> tupla de 7 números, o None."""
    m = RE_LINEA.search(linea)
    if not m:
        return None
    tick = int(m.group(1))
    vals = tuple(float(m.group(i)) for i in range(2, 8))
    return (tick, *vals)


def listar_puertos():
    if serial is None:
        print("Falta pyserial. Instala con: pip install pyserial --break-system-packages")
        return
    puertos = serial.tools.list_ports.comports()
    if not puertos:
        print("No se encontró ningún puerto serie.")
        return
    for p in puertos:
        print(f"  {p.device}  -  {p.description}")


class LectorSerial(threading.Thread):
    """Lee líneas del puerto serie en un hilo aparte, sin bloquear la gráfica."""

    def __init__(self, puerto, baud, cola):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.baud = baud
        self.cola = cola
        self._detener = threading.Event()

    def detener(self):
        self._detener.set()

    def run(self):
        try:
            ser = serial.Serial(self.puerto, self.baud, timeout=1)
        except Exception as e:
            print(f"No se pudo abrir {self.puerto}: {e}")
            self.cola.put(None)      # señal de error hacia el hilo principal
            return

        while not self._detener.is_set():
            try:
                cruda = ser.readline().decode("utf-8", errors="ignore")
            except Exception:
                continue
            if not cruda:
                continue
            datos = parsear_linea(cruda)
            if datos is not None:
                self.cola.put(datos)

        ser.close()


def wrap_deg(a):
    """Envuelve a (-180, 180], útil si algún día quieres comparar diferencias de yaw."""
    return (a + 180.0) % 360.0 - 180.0


def crear_figura_y_actualizador(cola, ventana_s, titulo_extra=""):
    """
    Arma la figura de 3 paneles y devuelve (fig, actualizar, estado), donde
    'actualizar(frame)' es la función que FuncAnimation debe llamar
    periódicamente. Separado así para poder probar 'actualizar' sin
    necesidad de un puerto serie real.
    """
    t_buf = deque()
    roll_s_buf, pitch_s_buf, yaw_s_buf = deque(), deque(), deque()
    roll_e_buf, pitch_e_buf, yaw_e_buf = deque(), deque(), deque()
    estado = {"t0": None, "conectado": True}

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    fig.suptitle(f"Tiempo real{titulo_extra}")

    l_roll_s,  = axes[0].plot([], [], color="tab:blue",   label="Sensor (accel)")
    l_roll_e,  = axes[0].plot([], [], color="tab:orange", label="Mahony")
    axes[0].set_title("Roll", loc="left", fontweight="bold")
    axes[0].set_ylabel("grados")

    l_pitch_s, = axes[1].plot([], [], color="tab:blue",   label="Sensor (accel)")
    l_pitch_e, = axes[1].plot([], [], color="tab:orange", label="Mahony")
    axes[1].set_title("Pitch", loc="left", fontweight="bold")
    axes[1].set_ylabel("grados")

    l_yaw_s,   = axes[2].plot([], [], color="tab:blue",   label="Sensor (magnetómetro / heading)")
    l_yaw_e,   = axes[2].plot([], [], color="tab:orange", label="Mahony")
    axes[2].set_title("Yaw", loc="left", fontweight="bold")
    axes[2].set_ylabel("grados (0-360)")
    axes[2].set_xlabel("Tiempo (s)")

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()

    lineas = (l_roll_s, l_roll_e, l_pitch_s, l_pitch_e, l_yaw_s, l_yaw_e)

    def actualizar(frame):
        hubo_error = False
        while True:
            try:
                item = cola.get_nowait()
            except queue.Empty:
                break
            if item is None:
                hubo_error = True
                continue
            tick, rs, ps, hmag, re_, pe, ye = item
            if estado["t0"] is None:
                estado["t0"] = tick
            t = (tick - estado["t0"]) / 1000.0
            t_buf.append(t)
            roll_s_buf.append(rs);   pitch_s_buf.append(ps);  yaw_s_buf.append(hmag)
            roll_e_buf.append(re_);  pitch_e_buf.append(pe);  yaw_e_buf.append(ye)

        if hubo_error and estado["conectado"]:
            estado["conectado"] = False
            fig.suptitle(f"Tiempo real{titulo_extra}  [SIN CONEXIÓN]", color="red")

        # Recorta lo que salga de la ventana visible (por tiempo, no por cantidad fija)
        if t_buf:
            limite = t_buf[-1] - ventana_s
            while t_buf and t_buf[0] < limite:
                t_buf.popleft()
                roll_s_buf.popleft();  pitch_s_buf.popleft();  yaw_s_buf.popleft()
                roll_e_buf.popleft();  pitch_e_buf.popleft();  yaw_e_buf.popleft()

        if not t_buf:
            return lineas

        t_arr = np.array(t_buf)
        l_roll_s.set_data(t_arr, roll_s_buf)
        l_roll_e.set_data(t_arr, roll_e_buf)
        l_pitch_s.set_data(t_arr, pitch_s_buf)
        l_pitch_e.set_data(t_arr, pitch_e_buf)
        l_yaw_s.set_data(t_arr, yaw_s_buf)
        l_yaw_e.set_data(t_arr, yaw_e_buf)

        xmax = t_arr[-1] if t_arr[-1] > t_arr[0] else t_arr[0] + 1
        for ax, series in ((axes[0], (roll_s_buf, roll_e_buf)),
                           (axes[1], (pitch_s_buf, pitch_e_buf)),
                           (axes[2], (yaw_s_buf, yaw_e_buf))):
            ax.set_xlim(t_arr[0], xmax)
            todos = np.concatenate([np.array(s) for s in series])
            if len(todos):
                margen = max(1.0, 0.1 * (todos.max() - todos.min()))
                ax.set_ylim(todos.min() - margen, todos.max() + margen)

        return lineas

    return fig, actualizar, estado


def main():
    ap = argparse.ArgumentParser(description="Gráfica en tiempo real: sensor vs Mahony (roll, pitch, yaw)")
    ap.add_argument("--port", help="Puerto serie, p.ej. COM5 o /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate (CDC lo ignora, pero pyserial lo exige)")
    ap.add_argument("--ventana", type=float, default=10.0, help="Segundos visibles en la gráfica (default 10)")
    ap.add_argument("--list", action="store_true", help="Lista los puertos disponibles y termina")
    args = ap.parse_args()

    if args.list:
        listar_puertos()
        return

    if serial is None:
        raise SystemExit("Falta pyserial. Instala con: pip install pyserial --break-system-packages")
    if not args.port:
        raise SystemExit("Falta --port. Usa --list para ver los puertos disponibles.")

    cola = queue.Queue()
    lector = LectorSerial(args.port, args.baud, cola)
    lector.start()

    fig, actualizar, _ = crear_figura_y_actualizador(cola, args.ventana, f" — {args.port}")
    anim = animation.FuncAnimation(fig, actualizar, interval=100, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        lector.detener()


if __name__ == "__main__":
    main()