"""
descargar_mapa.py — Descarga zonas del mapa para usarlas sin internet.
Ejecutar CON internet:
    python3 descargar_mapa.py

Se puede correr las veces que quieras: lo nuevo se agrega a la misma base
y lo que ya estaba descargado no se vuelve a bajar.
"""
import math
import os
from tkintermapview import OfflineLoader

# Base de datos local (la misma ruta que usa el panel)
RUTA_BD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapa_offline_CUCEI.db")

# Zonas a descargar: (nombre, (lat, lon) superior izquierda, (lat, lon) inferior derecha)
# Las coordenadas se sacan en Google Maps con clic derecho sobre el punto.
ZONAS = [
    ("Zona 1", (lat_sup, long_izq), (lat_inf, long_der)),
    # ("Zona 2", (lat_sup, lon_izq), (lat_inf, lon_der)),
]

# Zoom: 15 = barrio, 19 = se distingue un dron sobre una cancha.
# Cada nivel extra multiplica por 4 los cuadros.
ZOOM_MIN = 14
ZOOM_MAX = 19

# Servidores (los mismos que el panel). Comenta el que no quieras.
SERVIDORES = {
    "Satélite": "https://mt0.google.com/vt/lyrs=s&hl=es&x={x}&y={y}&z={z}&s=Ga",
    "Calles":   "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
}


def cuadro(lat, lon, zoom):
    """Número de cuadro (x, y) que contiene a (lat, lon) en ese zoom."""
    n = 2 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def contar_cuadros(sup_izq, inf_der):
    total = 0
    for z in range(ZOOM_MIN, ZOOM_MAX + 1):
        x1, y1 = cuadro(sup_izq[0], sup_izq[1], z)
        x2, y2 = cuadro(inf_der[0], inf_der[1], z)
        total += (abs(x2 - x1) + 1) * (abs(y2 - y1) + 1)
    return total


# --- Resumen antes de descargar ---
total = 0
for nombre, sup_izq, inf_der in ZONAS:
    n = contar_cuadros(sup_izq, inf_der)
    total += n
    print(f"{nombre}: {n} cuadros por servidor")
total *= len(SERVIDORES)
print(f"\nTotal: {total} cuadros ({len(SERVIDORES)} servidores), "
      f"aprox. {total * 20 / 1024:.0f} MB")

if input("¿Descargar? (s/n): ").strip().lower() != "s":
    raise SystemExit("Cancelado.")

# --- Descarga ---
for nombre_srv, url in SERVIDORES.items():
    cargador = OfflineLoader(path=RUTA_BD, tile_server=url)
    for nombre, sup_izq, inf_der in ZONAS:
        print(f"\nDescargando {nombre} ({nombre_srv})...")
        cargador.save_offline_tiles(sup_izq, inf_der, ZOOM_MIN, ZOOM_MAX)

print(f"\nListo: mapa guardado en {RUTA_BD}")
