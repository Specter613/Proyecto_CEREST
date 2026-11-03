"""
Panel de Control MAVLink - Interfaz Gráfica
=============================================
Requiere: pymavlink, matplotlib
Opcional (mapa con imágenes): tkintermapview
Instalar con:
    pip install pymavlink matplotlib tkintermapview --break-system-packages

Ejecutar con:
    python3 panel_mavlink_gui.py

Mensajes MAVLink que usa:
    ATTITUDE            -> pestaña IMU
    LOCAL_POSITION_NED  -> pestaña Posición (flujo óptico)
    GPS_RAW_INT         -> pestaña GPS          (stream EXTENDED_STATUS)
    SYS_STATUS          -> barra de batería     (stream EXTENDED_STATUS)
    PARAM_VALUE         -> pestaña PIDs
"""

import math
import queue
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox

from pymavlink import mavutil

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle, Polygon, Rectangle
import numpy as np
import os
# Mapa con imágenes (opcional): si no está instalado, se usa una gráfica lat/lon
try:
    from tkintermapview import TkinterMapView
    HAY_MAPA = True
except ImportError:
    HAY_MAPA = False

RUTA_MAPA_OFFLINE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapa_offline_PGG.db")
# ============================================================
# CONFIGURACIÓN
# ============================================================
CONEXION_MAVLINK = 'udpin:0.0.0.0:14550'

# Estructura de PIDs (debe coincidir con paramTable_ del firmware).
#   Bucle exterior (ángulo):    PID_AR_*, PID_AP_*      (yaw no tiene)
#   Bucle interior (velocidad): PID_RR_*, PID_RP_*, PID_RY_*
GRUPOS_PID = [
    ("Bucle exterior (ángulo)", [
        ("Roll",  "PID_AR"),
        ("Pitch", "PID_AP"),
    ]),
    ("Bucle interior (velocidad angular)", [
        ("Roll",  "PID_RR"),
        ("Pitch", "PID_RP"),
        ("Yaw",   "PID_RY"),
    ]),
]
TERMINOS = ("P", "I", "D")

# Lista plana con los 15 nombres, en el mismo orden que el firmware.
NOMBRES_PID = [
    f"{prefijo}_{t}"
    for _, ejes in GRUPOS_PID
    for _, prefijo in ejes
    for t in TERMINOS
]

# Puntos máximos de la trayectoria X-Y (a 10 Hz: 3000 = 5 minutos)
MAX_PUNTOS_TRAYECTORIA = 3000

# Puntos máximos del recorrido GPS (a 5 Hz: 3000 = 10 minutos)
MAX_PUNTOS_GPS = 3000

# Batería: voltaje por celda para los colores del indicador
CELDA_OK = 3.70      # V: arriba de esto, verde
CELDA_BAJA = 3.50    # V: entre BAJA y OK, naranja; debajo, rojo

# Servidores de imágenes del mapa
SERVIDORES_MAPA = {
    "Calles (OpenStreetMap)": ("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png", 19),
    "Satélite": ("https://mt0.google.com/vt/lyrs=s&hl=es&x={x}&y={y}&z={z}&s=Ga", 22),
}

NOMBRES_FIX = {
    0: "Sin GPS", 1: "Sin fix", 2: "2D", 3: "3D",
    4: "DGPS", 5: "RTK float", 6: "RTK fijo",
}


# ============================================================
# HILO DE COMUNICACIÓN MAVLINK
# ============================================================
class HiloMavlink(threading.Thread):
    """
    Corre en segundo plano, separado de la interfaz gráfica.

    - Mantiene la conexión abierta y procesa mensajes entrantes.
    - Guarda el último valor de actitud, posición, GPS, batería y de cada PID.
    - Recibe comandos desde la GUI a través de una cola thread-safe,
      para que nunca dos hilos toquen el socket MAVLink al mismo tiempo.
    """

    def __init__(self, direccion=CONEXION_MAVLINK):
        super().__init__(daemon=True)
        self.direccion = direccion
        self.master = None

        self.conectado = False
        self.lock = threading.Lock()

        self.actitud = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "tick": 0.0}
        self.posicion = {"x": 0.0, "y": 0.0, "z": 0.0, "nuevo": False}
        self.gps = {
            "lat": None, "lon": None, "alt": None, "fix": 0, "sats": 0,
            "hdop": None, "h_acc": None, "vel": None, "rumbo": None,
            "nuevo": False, "tick": 0.0,
        }
        self.bateria = {"voltaje": None, "corriente": None, "restante": None,
                        "mah_dron": None, "tick": 0.0}
        self.pids = {nombre: None for nombre in NOMBRES_PID}

        self.cola_comandos = queue.Queue()
        self._detener = threading.Event()

        # Callbacks opcionales que la GUI puede registrar.
        self.al_conectar = None  # función() -> None

    def enviar_comando(self, tipo, **kwargs):
        """La GUI llama esto — nunca manda MAVLink directamente ella
        misma, todo pasa por la cola."""
        self.cola_comandos.put((tipo, kwargs))

    def detener(self):
        self._detener.set()

    def run(self):
        try:
            self.master = mavutil.mavlink_connection(self.direccion)
            self.master.wait_heartbeat(timeout=15)
            with self.lock:
                self.conectado = True
            if self.al_conectar:
                self.al_conectar()
        except Exception as e:
            print(f"Error de conexión: {e}")
            return

        while not self._detener.is_set():
            self._procesar_comandos()

            msg = self.master.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue

            tipo = msg.get_type()

            if tipo == 'ATTITUDE':
                with self.lock:
                    self.actitud["roll"] = msg.roll
                    self.actitud["pitch"] = msg.pitch
                    self.actitud["yaw"] = msg.yaw
                    self.actitud["tick"] = time.time()

            elif tipo == 'LOCAL_POSITION_NED':
                with self.lock:
                    self.posicion["x"] = msg.x
                    self.posicion["y"] = msg.y
                    self.posicion["z"] = msg.z       # altura tal cual (positiva)
                    self.posicion["nuevo"] = True

            elif tipo == 'GPS_RAW_INT':
                # Unidades MAVLink: lat/lon en grados*1e7, alt en mm, eph en HDOP*100,
                # vel en cm/s, cog en grados*100, h_acc en mm. 65535 / 0 = desconocido.
                h_acc = getattr(msg, "h_acc", 0)
                with self.lock:
                    self.gps["fix"] = msg.fix_type
                    self.gps["sats"] = msg.satellites_visible
                    self.gps["lat"] = msg.lat / 1e7
                    self.gps["lon"] = msg.lon / 1e7
                    self.gps["alt"] = msg.alt / 1000.0
                    self.gps["hdop"] = msg.eph / 100.0 if msg.eph != 65535 else None
                    self.gps["h_acc"] = h_acc / 1000.0 if h_acc else None
                    self.gps["vel"] = msg.vel / 100.0 if msg.vel != 65535 else None
                    self.gps["rumbo"] = msg.cog / 100.0 if msg.cog != 65535 else None
                    self.gps["nuevo"] = True
                    self.gps["tick"] = time.time()

            elif tipo == 'SYS_STATUS':
                # Unidades MAVLink: voltaje en mV (65535 = desconocido),
                # corriente en cA (-1 = desconocida), restante en % (-1 = desconocido).
                with self.lock:
                    self.bateria["voltaje"] = (msg.voltage_battery / 1000.0
                                               if msg.voltage_battery != 65535 else None)
                    self.bateria["corriente"] = (msg.current_battery / 100.0
                                                 if msg.current_battery >= 0 else None)
                    self.bateria["restante"] = (msg.battery_remaining
                                                if msg.battery_remaining >= 0 else None)
                    self.bateria["tick"] = time.time()

            elif tipo == 'PARAM_VALUE':
                nombre = msg.param_id.rstrip('\x00').strip()
                if nombre in self.pids:
                    with self.lock:
                        self.pids[nombre] = msg.param_value

    def _pedir_stream(self, stream, hz):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            stream, hz, 1 if hz > 0 else 0)

    def _procesar_comandos(self):
        try:
            while True:
                tipo, kwargs = self.cola_comandos.get_nowait()
                if tipo == "pedir_pids":
                    self.master.mav.param_request_list_send(
                        self.master.target_system, self.master.target_component)

                elif tipo == "set_pid":
                    self.master.mav.param_set_send(
                        self.master.target_system, self.master.target_component,
                        kwargs["nombre"].encode('utf-8'), kwargs["valor"],
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

                elif tipo == "set_stream_imu":
                    self._pedir_stream(mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                                       kwargs["hz"])

                elif tipo == "set_stream_pos":
                    self._pedir_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                                       kwargs["hz"])

                elif tipo == "set_stream_estado":
                    # GPS_RAW_INT + SYS_STATUS (como en ArduPilot)
                    self._pedir_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                                       kwargs["hz"])
        except queue.Empty:
            pass

    def obtener_actitud(self):
        with self.lock:
            return dict(self.actitud)

    def obtener_posicion(self):
        """Devuelve la última posición y marca que ya se leyó."""
        with self.lock:
            pos = dict(self.posicion)
            self.posicion["nuevo"] = False
            return pos

    def obtener_gps(self):
        """Devuelve el último dato de GPS y marca que ya se leyó."""
        with self.lock:
            gps = dict(self.gps)
            self.gps["nuevo"] = False
            return gps

    def obtener_bateria(self):
        with self.lock:
            return dict(self.bateria)

    def obtener_pids(self):
        with self.lock:
            return dict(self.pids)


# ============================================================
# HUD DE ACTITUD (estilo Mission Planner)
# ============================================================
class IndicadorActitud:
    """HUD estilo Mission Planner: horizonte, escalera de pitch, arco de roll
    y cinta de rumbo, dibujado sobre un Axes de matplotlib."""

    # Signos del dibujo: solo afectan al HUD, no a los valores ni a la gráfica de líneas.
    SIGNO_ROLL = -1.0    # roll invertido respecto al valor recibido
    SIGNO_PITCH = -1.0   # pitch igual que en el horizonte original

    ESCALA_PITCH = 0.025   # unidades del dibujo por grado de pitch (±40° visibles)
    ESCALA_RUMBO = 0.02    # unidades del dibujo por grado en la cinta (±45° visibles)

    # Tamaño de textos y grosor de líneas (súbelos si agrandas el HUD)
    ESCALA_TEXTO = 1.5
    ESCALA_LINEAS = 1.4

    CIELO = '#3a7bd5'
    TIERRA = '#8b5a2b'

    def __init__(self, ax):
        self.ax = ax
        self.actualizar(0.0, 0.0, 0.0)

    @staticmethod
    def _rotar(u, v, angulo):
        """Rota los puntos (u, v) un ángulo (rad) alrededor del centro."""
        c, s = math.cos(angulo), math.sin(angulo)
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        return u * c - v * s, u * s + v * c

    @staticmethod
    def _envolver_180(grados):
        """Lleva un ángulo al rango (-180, 180]."""
        g = (grados + 180.0) % 360.0 - 180.0
        return 180.0 if g == -180.0 else g

    def actualizar(self, roll_rad, pitch_rad, yaw_rad=0.0):
        ax = self.ax
        ax.cla()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_aspect('equal')
        ax.axis('off')

        ft = self.ESCALA_TEXTO
        lw = self.ESCALA_LINEAS

        a = self.SIGNO_ROLL * roll_rad                     # giro del horizonte
        pitch_deg = self.SIGNO_PITCH * math.degrees(pitch_rad)
        desplaz = -pitch_deg * self.ESCALA_PITCH

        # ---------------- Cielo y tierra ----------------
        ax.add_patch(Rectangle((-1, -1), 2, 2, facecolor=self.CIELO, zorder=0))
        L = 4.0
        xr, yr = self._rotar([-L, L, L, -L], [desplaz, desplaz, desplaz - L, desplaz - L], a)
        ax.add_patch(Polygon(list(zip(xr, yr)), closed=True, facecolor=self.TIERRA, zorder=1))
        xr, yr = self._rotar([-L, L], [desplaz, desplaz], a)
        ax.plot(xr, yr, color='white', lw=2 * lw, zorder=2)

        # ---------------- Escalera de pitch ----------------
        for p in range(-90, 91, 5):
            if p == 0:
                continue
            v = desplaz + p * self.ESCALA_PITCH
            if abs(v) > 0.7:
                continue
            ancho = 0.22 if p % 10 == 0 else 0.10
            xr, yr = self._rotar([-ancho, ancho], [v, v], a)
            ax.plot(xr, yr, color='white', lw=1.2 * lw, zorder=3)
            if p % 10 == 0:
                for lado in (-1, 1):
                    tx, ty = self._rotar([lado * (ancho + 0.09)], [v], a)
                    ax.text(tx[0], ty[0], f"{abs(p)}", color='white', fontsize=7 * ft,
                            ha='center', va='center', rotation=math.degrees(a), zorder=3)

        # ---------------- Arco de roll ----------------
        R = 0.74
        ang = np.radians(np.linspace(30, 150, 60))
        ax.plot(R * np.cos(ang), R * np.sin(ang), color='white', lw=1.5 * lw, zorder=4)
        for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            th = math.radians(90 + t)
            largo = 0.08 if t % 30 == 0 else 0.05
            ax.plot([R * math.cos(th), (R + largo) * math.cos(th)],
                    [R * math.sin(th), (R + largo) * math.sin(th)],
                    color='white', lw=1.5 * lw, zorder=4)
        # Triángulo indicador: gira con el horizonte
        tx, ty = self._rotar([0.0, -0.035, 0.035], [R, R - 0.07, R - 0.07], a)
        ax.add_patch(Polygon(list(zip(tx, ty)), closed=True, facecolor='white', zorder=4))

        # ---------------- Símbolo del dron (fijo) ----------------
        for lado in (-1, 1):
            ax.plot([lado * 0.40, lado * 0.14, lado * 0.08], [0, 0, -0.06],
                    color='yellow', lw=3 * lw, solid_capstyle='round', zorder=6)
        ax.plot([0], [0], 'o', color='yellow', markersize=5 * lw, zorder=6)

        # ---------------- Cinta de rumbo (±180°) ----------------
        rumbo = self._envolver_180(math.degrees(yaw_rad))
        ax.add_patch(Rectangle((-1, 0.84), 2, 0.16, facecolor='black', alpha=0.5, zorder=7))
        inicio = int(math.floor(rumbo)) - 50
        inicio -= inicio % 5
        for h in range(inicio, int(math.ceil(rumbo)) + 55, 5):
            x = (h - rumbo) * self.ESCALA_RUMBO
            if abs(x) > 0.95:
                continue
            hn = self._envolver_180(h)
            alto = 0.05 if int(hn) % 10 == 0 else 0.03
            ax.plot([x, x], [0.84, 0.84 + alto], color='white', lw=1 * lw, zorder=8)
            if int(hn) % 10 == 0:
                ax.text(x, 0.94, f"{hn:.0f}", color='white', fontsize=7 * ft,
                        ha='center', va='center', zorder=8)
        # Caja central con el rumbo exacto y marca de referencia
        ax.add_patch(Rectangle((-0.13, 0.875), 0.26, 0.12, facecolor='black',
                               edgecolor='white', lw=1 * lw, zorder=9))
        ax.text(0, 0.935, f"{rumbo:+.0f}°", color='white', fontsize=9 * ft,
                fontweight='bold', ha='center', va='center', zorder=10)
        ax.add_patch(Polygon([(0, 0.84), (-0.03, 0.80), (0.03, 0.80)], closed=True,
                             facecolor='white', zorder=9))

        # ---------------- Lectura numérica ----------------
        ax.text(-0.97, -0.93,
                f"Roll {math.degrees(roll_rad):+5.1f}°   Pitch {math.degrees(pitch_rad):+5.1f}°",
                color='white', fontsize=8 * ft, ha='left', va='center', zorder=10,
                bbox=dict(facecolor='black', alpha=0.5, edgecolor='none', pad=2))
        
# ============================================================
# APLICACIÓN PRINCIPAL (GUI)
# ============================================================
class AppMavlink(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Panel de Control MAVLink")
        self.geometry("1050x680")

        self.hilo = HiloMavlink()
        self.hilo.al_conectar = self._on_conectado
        self.hilo.start()

        self.historial_t = []
        self.historial_roll = []
        self.historial_pitch = []
        self.historial_yaw = []
        self.tiempo_inicio = time.time()

        # Trayectoria X-Y (flujo óptico)
        self.tray_x = []
        self.tray_y = []

        # Recorrido GPS (lat, lon) y último dato
        self.ruta_gps = []
        self.ultimo_gps = None
        self.gps_centrado = False
        self.ultimo_dibujo_ruta = 0.0

        # Batería
        self.mah_consumidos = 0.0
        self.celdas = None
        self.tick_bateria_ant = None

        # PIDs enviados que aún no han sido confirmados por el firmware
        # {nombre: valor_enviado}
        self.pendientes = {}

        self._construir_interfaz()
        self.after(100, self._loop_actualizacion)

    # -------------------- Construcción de la interfaz --------------------
    def _construir_interfaz(self):
        barra = ttk.Frame(self)
        barra.pack(fill='x', padx=8, pady=4)
        self.lbl_estado = ttk.Label(barra, text="Conectando...", foreground="orange")
        self.lbl_estado.pack(side='left')

        # Batería: visible en todas las pestañas
        marco_bat = ttk.Frame(barra)
        marco_bat.pack(side='right')
        fuente_bat = ('', 11, 'bold')
        self.lbl_bat_v = ttk.Label(marco_bat, text="Batería: --- V", font=fuente_bat)
        self.lbl_bat_v.pack(side='left', padx=8)
        self.lbl_bat_celda = ttk.Label(marco_bat, text="(--- V/celda)", font=fuente_bat)
        self.lbl_bat_celda.pack(side='left', padx=4)
        self.lbl_bat_i = ttk.Label(marco_bat, text="--- A", font=fuente_bat)
        self.lbl_bat_i.pack(side='left', padx=8)
        self.lbl_bat_pct = ttk.Label(marco_bat, text="--- %", font=fuente_bat)
        self.lbl_bat_pct.pack(side='left', padx=8)
        self.lbl_bat_mah = ttk.Label(marco_bat, text="0 mAh", font=fuente_bat)
        self.lbl_bat_mah.pack(side='left', padx=8)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True, padx=8, pady=8)

        self.tab_pids = ttk.Frame(self.notebook)
        self.tab_imu = ttk.Frame(self.notebook)
        self.tab_pos = ttk.Frame(self.notebook)
        self.tab_gps = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_pids, text="PIDs")
        self.notebook.add(self.tab_imu, text="IMU")
        self.notebook.add(self.tab_pos, text="Posición")
        self.notebook.add(self.tab_gps, text="GPS")

        self._construir_tab_pids()
        self._construir_tab_imu()
        self._construir_tab_posicion()
        self._construir_tab_gps()

    def _construir_tab_pids(self):
        frame = self.tab_pids

        barra_botones = ttk.Frame(frame)
        barra_botones.pack(fill='x', pady=6)
        ttk.Button(barra_botones, text="Actualizar todos",
                   command=self._pedir_pids).pack(side='left', padx=4)

        self.entradas_pid = {}
        contenedor = ttk.Frame(frame)
        contenedor.pack(fill='both', expand=True, padx=4)

        fila = 0
        for titulo, ejes in GRUPOS_PID:
            ttk.Label(contenedor, text=titulo, font=('', 11, 'bold')).grid(
                row=fila, column=0, columnspan=8, sticky='w', padx=4, pady=(14, 2))
            fila += 1

            ttk.Label(contenedor, text="Eje", font=('', 9, 'bold')).grid(
                row=fila, column=0, sticky='w', padx=4)
            for j, t in enumerate(TERMINOS):
                ttk.Label(contenedor, text=f"{t}  (actual / nuevo)",
                          font=('', 9, 'bold')).grid(
                    row=fila, column=1 + 2 * j, columnspan=2, sticky='w', padx=4)
            fila += 1

            for etiqueta, prefijo in ejes:
                ttk.Label(contenedor, text=etiqueta, width=8).grid(
                    row=fila, column=0, sticky='w', padx=4, pady=3)

                for j, t in enumerate(TERMINOS):
                    nombre = f"{prefijo}_{t}"

                    lbl_valor = ttk.Label(contenedor, text="---", width=9)
                    lbl_valor.grid(row=fila, column=1 + 2 * j, sticky='w', padx=(4, 0))

                    entrada = ttk.Entry(contenedor, width=9)
                    entrada.grid(row=fila, column=2 + 2 * j, sticky='w', padx=(0, 10))
                    entrada.bind("<Return>", lambda ev, p=prefijo: self._aplicar_fila(p))

                    self.entradas_pid[nombre] = {"label": lbl_valor, "entry": entrada}

                ttk.Button(contenedor, text="Aplicar",
                           command=lambda p=prefijo: self._aplicar_fila(p)).grid(
                    row=fila, column=7, sticky='w', padx=4)
                fila += 1

        self.lbl_estado_pid = ttk.Label(frame, text="", foreground="green")
        self.lbl_estado_pid.pack(pady=6)

    def _construir_tab_imu(self):
        frame = self.tab_imu

        barra = ttk.Frame(frame)
        barra.pack(fill='x', pady=6)
        ttk.Label(barra, text="Frecuencia (Hz):").pack(side='left', padx=4)
        self.combo_hz = ttk.Combobox(barra, values=[0, 5, 10, 20, 30, 50],
                                      width=6, state='readonly')
        self.combo_hz.set(20)
        self.combo_hz.pack(side='left', padx=4)
        ttk.Button(barra, text="Aplicar frecuencia",
                   command=self._aplicar_hz_imu).pack(side='left', padx=4)

        contenedor = ttk.Frame(frame)
        contenedor.pack(fill='both', expand=True)

        # Proporción de anchos HUD : gráfica (súbelo para agrandar más el HUD)
        TAMANO_HUD = 3.0
        TAMANO_LINEAS = 2.0

        self.fig = plt.Figure(figsize=(9, 5))
        gs = self.fig.add_gridspec(1, 2, width_ratios=[TAMANO_HUD, TAMANO_LINEAS])
        self.ax_horizonte = self.fig.add_subplot(gs[0, 0])
        self.ax_lineas = self.fig.add_subplot(gs[0, 1])

        # Márgenes chicos: el HUD ocupa casi toda su columna
        self.fig.subplots_adjust(left=0.01, right=0.98, top=0.95, bottom=0.10, wspace=0.10)

        self.indicador = IndicadorActitud(self.ax_horizonte)

        self.ax_lineas.set_title("Roll / Pitch / Yaw (°)")
        self.ax_lineas.set_xlabel("Tiempo (s)")
        self.linea_roll, = self.ax_lineas.plot([], [], label='Roll', color='tab:blue')
        self.linea_pitch, = self.ax_lineas.plot([], [], label='Pitch', color='tab:orange')
        self.linea_yaw, = self.ax_lineas.plot([], [], label='Yaw', color='tab:green')
        self.ax_lineas.legend(loc='upper right', fontsize=8)
        self.ax_lineas.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=contenedor)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    def _construir_tab_posicion(self):
        frame = self.tab_pos

        barra = ttk.Frame(frame)
        barra.pack(fill='x', pady=6)
        ttk.Label(barra, text="Frecuencia (Hz):").pack(side='left', padx=4)
        self.combo_hz_pos = ttk.Combobox(barra, values=[0, 5, 10, 20, 30, 50],
                                            width=6, state='readonly')
        self.combo_hz_pos.set(10)
        self.combo_hz_pos.pack(side='left', padx=4)
        ttk.Button(barra, text="Aplicar frecuencia",
                command=self._aplicar_hz_pos).pack(side='left', padx=4)

        self.lbl_z = ttk.Label(frame, text="Z: --- m", font=('', 16, 'bold'))
        self.lbl_z.pack(pady=6)

        contenedor = ttk.Frame(frame)
        contenedor.pack(fill='both', expand=True)

        self.fig_pos = plt.Figure(figsize=(6, 5))
        self.ax_xy = self.fig_pos.add_subplot(1, 1, 1)
        self.ax_xy.set_title("Posición X vs Y")
        self.ax_xy.set_xlabel("X (m)")
        self.ax_xy.set_ylabel("Y (m)")
        self.ax_xy.set_aspect('equal', adjustable='box')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.plot([0], [0], 'k+', markersize=12)
        self.linea_tray, = self.ax_xy.plot([], [], color='tab:purple', linewidth=1)
        self.punto_actual, = self.ax_xy.plot([], [], 'o', color='tab:red', markersize=8)

        self.canvas_pos = FigureCanvasTkAgg(self.fig_pos, master=contenedor)
        self.canvas_pos.get_tk_widget().pack(fill='both', expand=True)

    def _construir_tab_gps(self):
        frame = self.tab_gps

        # --- Barra superior: frecuencia, tipo de mapa, centrar, seguir, borrar ---
        barra = ttk.Frame(frame)
        barra.pack(fill='x', pady=6)
        ttk.Label(barra, text="Frecuencia GPS/batería (Hz):").pack(side='left', padx=4)
        self.combo_hz_estado = ttk.Combobox(barra, values=[0, 1, 2, 5, 10],
                                            width=5, state='readonly')
        self.combo_hz_estado.set(2)
        self.combo_hz_estado.pack(side='left', padx=4)
        ttk.Button(barra, text="Aplicar frecuencia",
                   command=self._aplicar_hz_estado).pack(side='left', padx=4)

        if HAY_MAPA:
            ttk.Label(barra, text="Mapa:").pack(side='left', padx=(16, 4))
            self.combo_mapa = ttk.Combobox(barra, values=list(SERVIDORES_MAPA.keys()),
                                           width=22, state='readonly')
            self.combo_mapa.set("Satélite")
            self.combo_mapa.pack(side='left', padx=4)
            self.combo_mapa.bind("<<ComboboxSelected>>", lambda ev: self._cambiar_mapa())

        ttk.Button(barra, text="Centrar en el dron",
                   command=self._centrar_mapa).pack(side='left', padx=(16, 4))
        self.var_seguir = tk.BooleanVar(value=True)
        ttk.Checkbutton(barra, text="Seguir al dron",
                        variable=self.var_seguir).pack(side='left', padx=4)
        ttk.Button(barra, text="Borrar recorrido",
                   command=self._borrar_ruta_gps).pack(side='left', padx=4)

        cuerpo = ttk.Frame(frame)
        cuerpo.pack(fill='both', expand=True)

        # --- Panel de datos (izquierda) ---
        datos = ttk.Frame(cuerpo, width=230)
        datos.pack(side='left', fill='y', padx=8)
        datos.pack_propagate(False)

        self.lbl_gps = {}
        filas = [
            ("fix", "Fix"), ("sats", "Satélites"), ("h_acc", "Precisión horiz."),
            ("hdop", "HDOP"), ("lat", "Latitud"), ("lon", "Longitud"),
            ("alt", "Altura MSL"), ("vel", "Velocidad"), ("rumbo", "Rumbo"),
            ("edad", "Último dato"),
        ]
        for i, (clave, texto) in enumerate(filas):
            ttk.Label(datos, text=texto + ":", font=('', 10, 'bold')).grid(
                row=i, column=0, sticky='w', pady=3)
            lbl = ttk.Label(datos, text="---", font=('', 10))
            lbl.grid(row=i, column=1, sticky='w', padx=6, pady=3)
            self.lbl_gps[clave] = lbl

        # --- Mapa (derecha) ---
        contenedor_mapa = ttk.Frame(cuerpo)
        contenedor_mapa.pack(side='left', fill='both', expand=True)

        self.marcador_gps = None
        self.ruta_mapa = None

        if HAY_MAPA:
            self.mapa = TkinterMapView(contenedor_mapa, corner_radius=0,
                                       database_path=RUTA_MAPA_OFFLINE)
            self.mapa.pack(fill='both', expand=True)
            self._cambiar_mapa()
            self.mapa.set_position(20.6597, -103.3496)   # posición inicial cualquiera
            self.mapa.set_zoom(15)
        else:
            ttk.Label(contenedor_mapa,
                      text="Instala 'tkintermapview' para ver el mapa con imágenes. "
                           "Mientras tanto se dibuja el recorrido en lat/lon.",
                      foreground="gray").pack(pady=4)
            self.fig_gps = plt.Figure(figsize=(6, 5))
            self.ax_gps = self.fig_gps.add_subplot(1, 1, 1)
            self.ax_gps.set_title("Recorrido GPS")
            self.ax_gps.set_xlabel("Longitud (°)")
            self.ax_gps.set_ylabel("Latitud (°)")
            self.ax_gps.grid(True, alpha=0.3)
            self.linea_gps, = self.ax_gps.plot([], [], color='tab:purple', linewidth=1.5)
            self.punto_gps, = self.ax_gps.plot([], [], 'o', color='tab:red', markersize=8)
            self.canvas_gps = FigureCanvasTkAgg(self.fig_gps, master=contenedor_mapa)
            self.canvas_gps.get_tk_widget().pack(fill='both', expand=True)

    # -------------------- Callbacks de datos --------------------
    def _on_conectado(self):
        # Se llama desde el hilo de fondo — usamos after() para
        # asegurarnos de tocar los widgets de Tk solo desde el hilo
        # principal (obligatorio en tkinter).
        self.after(0, lambda: self.lbl_estado.config(text="Conectado", foreground="green"))
        # Al conectar: pedir PIDs y el stream de GPS + batería (la cola es thread-safe).
        self.hilo.enviar_comando("pedir_pids")
        self.hilo.enviar_comando("set_stream_estado", hz=2)

    # -------------------- Acciones de usuario --------------------
    def _pedir_pids(self):
        self.hilo.enviar_comando("pedir_pids")
        self.lbl_estado_pid.config(text="Solicitando parámetros...", foreground="orange")

    def _aplicar_fila(self, prefijo):
        """Envía las ganancias P/I/D de un eje que tengan un valor escrito.
        Los campos vacíos se ignoran (no se tocan en el firmware)."""
        cambios = {}
        for t in TERMINOS:
            nombre = f"{prefijo}_{t}"
            texto = self.entradas_pid[nombre]["entry"].get().strip().replace(",", ".")
            if not texto:
                continue
            try:
                cambios[nombre] = float(texto)
            except ValueError:
                messagebox.showerror("Valor inválido",
                                     f"'{texto}' no es un número válido ({nombre}).")
                return

        if not cambios:
            return

        for nombre, valor in cambios.items():
            self.hilo.enviar_comando("set_pid", nombre=nombre, valor=valor)
            self.pendientes[nombre] = valor
            self.entradas_pid[nombre]["entry"].delete(0, 'end')

        lista = ", ".join(f"{n}={v:g}" for n, v in cambios.items())
        self.lbl_estado_pid.config(text=f"Enviando {lista} ...", foreground="orange")

    def _aplicar_hz_imu(self):
        self.hilo.enviar_comando("set_stream_imu", hz=int(self.combo_hz.get()))

    def _aplicar_hz_pos(self):
        self.hilo.enviar_comando("set_stream_pos", hz=int(self.combo_hz_pos.get()))

    def _aplicar_hz_estado(self):
        self.hilo.enviar_comando("set_stream_estado", hz=int(self.combo_hz_estado.get()))

    def _cambiar_mapa(self):
        if not HAY_MAPA:
            return
        url, zoom_max = SERVIDORES_MAPA[self.combo_mapa.get()]
        self.mapa.set_tile_server(url, max_zoom=zoom_max)

    def _centrar_mapa(self):
        if self.ultimo_gps is None:
            return
        lat, lon = self.ultimo_gps
        if HAY_MAPA:
            self.mapa.set_position(lat, lon)
            self.mapa.set_zoom(19)

    def _borrar_ruta_gps(self):
        self.ruta_gps.clear()
        if HAY_MAPA and self.ruta_mapa is not None:
            self.ruta_mapa.delete()
            self.ruta_mapa = None

    # -------------------- Actualización de batería (todas las pestañas) --------------------
    def _actualizar_bateria(self):
        bat = self.hilo.obtener_bateria()
        v, i, pct = bat["voltaje"], bat["corriente"], bat["restante"]

        # mAh integrados por el panel (respaldo si el dron no manda BATTERY_STATUS)
        if i is not None and bat["tick"] > 0:
            if self.tick_bateria_ant is not None and bat["tick"] > self.tick_bateria_ant:
                dt = bat["tick"] - self.tick_bateria_ant
                if dt < 5.0:   # ignora huecos largos (desconexión)
                    self.mah_consumidos += i * dt * 1000.0 / 3600.0
            self.tick_bateria_ant = bat["tick"]

        if v is not None and v > 1.0:
            # Número de celdas: se detecta una vez (batería cargada: <= 4.3 V/celda)
            if self.celdas is None:
                self.celdas = max(1, math.ceil(v / 4.3))
            v_celda = v / self.celdas
            if v_celda >= CELDA_OK:
                color = "green"
            elif v_celda >= CELDA_BAJA:
                color = "orange"
            else:
                color = "red"
            self.lbl_bat_v.config(text=f"Batería: {v:.2f} V ({self.celdas}S)", foreground=color)
            self.lbl_bat_celda.config(text=f"({v_celda:.2f} V/celda)", foreground=color)
        else:
            self.lbl_bat_v.config(text="Batería: --- V", foreground="gray")
            self.lbl_bat_celda.config(text="(--- V/celda)", foreground="gray")

        self.lbl_bat_i.config(text=f"{i:.1f} A" if i is not None else "--- A")
        self.lbl_bat_pct.config(text=f"{pct} %" if pct is not None else "--- %")

        # mAh: los del dron si llegan (no dependen del enlace); si no, los del panel
        if bat["mah_dron"] is not None:
            self.lbl_bat_mah.config(text=f"{bat['mah_dron']} mAh")
        else:
            self.lbl_bat_mah.config(text=f"{self.mah_consumidos:.0f} mAh (panel)")

    # -------------------- GPS --------------------
    def _acumular_gps(self, gps):
        """Guarda el recorrido solo con fix 3D (sin fix, lat/lon no significan nada)."""
        if gps["nuevo"] and gps["fix"] >= 3 and gps["lat"] and gps["lon"]:
            punto = (gps["lat"], gps["lon"])
            self.ultimo_gps = punto
            self.ruta_gps.append(punto)
            if len(self.ruta_gps) > MAX_PUNTOS_GPS:
                self.ruta_gps.pop(0)

    def _dibujar_gps(self, gps):
        # --- Datos ---
        fix = gps["fix"]
        color_fix = "green" if fix >= 3 else ("orange" if fix == 2 else "red")
        self.lbl_gps["fix"].config(text=f"{fix} ({NOMBRES_FIX.get(fix, '?')})",
                                   foreground=color_fix)
        self.lbl_gps["sats"].config(text=str(gps["sats"]))
        self.lbl_gps["h_acc"].config(
            text=f"{gps['h_acc']:.2f} m" if gps["h_acc"] is not None else "---")
        self.lbl_gps["hdop"].config(
            text=f"{gps['hdop']:.2f}" if gps["hdop"] is not None else "---")
        if fix >= 2 and gps["lat"] is not None:
            self.lbl_gps["lat"].config(text=f"{gps['lat']:.7f}°")
            self.lbl_gps["lon"].config(text=f"{gps['lon']:.7f}°")
        else:
            self.lbl_gps["lat"].config(text="(sin fix)")
            self.lbl_gps["lon"].config(text="(sin fix)")
        self.lbl_gps["alt"].config(
            text=f"{gps['alt']:.1f} m" if (fix >= 3 and gps["alt"] is not None) else "---")
        self.lbl_gps["vel"].config(
            text=f"{gps['vel']:.2f} m/s" if gps["vel"] is not None else "---")
        self.lbl_gps["rumbo"].config(
            text=f"{gps['rumbo']:.0f}°" if gps["rumbo"] is not None else "---")
        if gps["tick"] > 0:
            edad = time.time() - gps["tick"]
            self.lbl_gps["edad"].config(text=f"hace {edad:.1f} s",
                                        foreground="green" if edad < 2 else "red")
        else:
            self.lbl_gps["edad"].config(text="sin datos", foreground="red")

        if self.ultimo_gps is None:
            return
        lat, lon = self.ultimo_gps

        # --- Mapa con imágenes ---
        if HAY_MAPA:
            if self.marcador_gps is None:
                self.marcador_gps = self.mapa.set_marker(lat, lon, text="Dron")
            else:
                self.marcador_gps.set_position(lat, lon)

            if not self.gps_centrado:
                self.mapa.set_position(lat, lon)
                self.mapa.set_zoom(19)
                self.gps_centrado = True
            elif self.var_seguir.get():
                self.mapa.set_position(lat, lon)

            # El recorrido se redibuja como máximo 1 vez por segundo (es lo más pesado)
            ahora = time.time()
            if len(self.ruta_gps) >= 2 and ahora - self.ultimo_dibujo_ruta > 1.0:
                self.ultimo_dibujo_ruta = ahora
                if self.ruta_mapa is None:
                    self.ruta_mapa = self.mapa.set_path(list(self.ruta_gps),
                                                        color="#ff3333", width=3)
                else:
                    try:
                        self.ruta_mapa.set_position_list(list(self.ruta_gps))
                    except AttributeError:
                        # Versiones viejas de tkintermapview: se vuelve a crear
                        self.ruta_mapa.delete()
                        self.ruta_mapa = self.mapa.set_path(list(self.ruta_gps),
                                                            color="#ff3333", width=3)

        # --- Gráfica lat/lon (sin tkintermapview) ---
        else:
            lats = [p[0] for p in self.ruta_gps]
            lons = [p[1] for p in self.ruta_gps]
            self.linea_gps.set_data(lons, lats)
            self.punto_gps.set_data([lon], [lat])
            # Misma escala en metros en ambos ejes: 1° de lon = cos(lat) · 1° de lat
            self.ax_gps.set_aspect(1.0 / math.cos(math.radians(lat)), adjustable='datalim')
            self.ax_gps.relim()
            self.ax_gps.autoscale_view()
            self.canvas_gps.draw_idle()

    # -------------------- Loop de refresco de la GUI --------------------
    def _loop_actualizacion(self):
        # Batería: siempre, en todas las pestañas
        self._actualizar_bateria()

        pids = self.hilo.obtener_pids()
        for nombre, valor in pids.items():
            if valor is not None:
                self.entradas_pid[nombre]["label"].config(text=f"{valor:.6g}")

        # Confirmación: el firmware responde con PARAM_VALUE tras cada PARAM_SET.
        for nombre, enviado in list(self.pendientes.items()):
            recibido = pids.get(nombre)
            if recibido is not None and math.isclose(recibido, enviado,
                                                     rel_tol=1e-5, abs_tol=1e-9):
                del self.pendientes[nombre]
                if not self.pendientes:
                    self.lbl_estado_pid.config(
                        text="✔ Ganancias confirmadas por el firmware",
                        foreground="green")

        # La trayectoria X-Y y el recorrido GPS se acumulan siempre,
        # aunque su pestaña no esté visible
        pos = self.hilo.obtener_posicion()
        if pos["nuevo"]:
            self.tray_x.append(pos["x"])
            self.tray_y.append(pos["y"])
            if len(self.tray_x) > MAX_PUNTOS_TRAYECTORIA:
                self.tray_x.pop(0)
                self.tray_y.pop(0)

        gps = self.hilo.obtener_gps()
        self._acumular_gps(gps)

        pestana = self.notebook.index(self.notebook.select())

        if pestana == 1:
            act = self.hilo.obtener_actitud()
            roll_deg = math.degrees(act["roll"])
            pitch_deg = math.degrees(act["pitch"])
            yaw_deg = math.degrees(act["yaw"])

            t = time.time() - self.tiempo_inicio
            self.historial_t.append(t)
            self.historial_roll.append(roll_deg)
            self.historial_pitch.append(pitch_deg)
            self.historial_yaw.append(yaw_deg)

            while self.historial_t and (t - self.historial_t[0]) > 15:
                self.historial_t.pop(0)
                self.historial_roll.pop(0)
                self.historial_pitch.pop(0)
                self.historial_yaw.pop(0)

            self.linea_roll.set_data(self.historial_t, self.historial_roll)
            self.linea_pitch.set_data(self.historial_t, self.historial_pitch)
            self.linea_yaw.set_data(self.historial_t, self.historial_yaw)
            if self.historial_t:
                self.ax_lineas.set_xlim(
                    self.historial_t[0], max(self.historial_t[-1], self.historial_t[0] + 1))
            self.ax_lineas.relim()
            self.ax_lineas.autoscale_view(scalex=False)

            self.indicador.actualizar(act["roll"], act["pitch"], act["yaw"])

            self.canvas.draw_idle()

        elif pestana == 2:
            self.lbl_z.config(text=f"Z: {pos['z']:.2f} m")
            if self.tray_x:
                self.linea_tray.set_data(self.tray_x, self.tray_y)
                self.punto_actual.set_data([self.tray_x[-1]], [self.tray_y[-1]])

                xmin, xmax = min(self.tray_x), max(self.tray_x)
                ymin, ymax = min(self.tray_y), max(self.tray_y)
                cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
                media = max(0.25, (xmax - xmin) / 2, (ymax - ymin) / 2) * 1.15
                self.ax_xy.set_xlim(cx - media, cx + media)
                self.ax_xy.set_ylim(cy - media, cy + media)

            self.canvas_pos.draw_idle()

        elif pestana == 3:
            self._dibujar_gps(gps)

        self.after(100, self._loop_actualizacion)  # ~10 FPS de refresco de GUI

    def destroy(self):
        self.hilo.detener()
        super().destroy()


if __name__ == "__main__":
    app = AppMavlink()
    app.protocol("WM_DELETE_WINDOW", app.destroy)
    app.mainloop()