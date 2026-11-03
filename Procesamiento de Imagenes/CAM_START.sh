#!/bin/sh

# Detener el script si ocurre cualquier error
set -e

# --- CONFIGURACIÓN DE VARIABLES ---
INTERFACE="wlxc8fe0f54a694"
CHANNEL="161"
KEY_PATH="/etc/wifibroadcast/drone.key"
STREAM_ID="7669206"

# --- VALIDACIÓN DE USUARIO ---
# Verifica si el script se está ejecutando como root/sudo
if [ "$EUID" -ne 0 ]; then
    echo "❌ Error: Este script necesita permisos de administrador."
    echo "Por favor, ejecútalo usando: sudo $0"
    exit 1
fi

# 2. Configurar la tarjeta en Modo Monitor
ip link set "$INTERFACE" down

iw dev "$INTERFACE" set type monitor

ip link set "$INTERFACE" up

iw dev "$INTERFACE" set channel "$CHANNEL"

# 3. Lanzar el receptor de Wifibroadcast
wfb_rx -c 127.0.0.1 -u 5600 -K "$KEY_PATH" -i "$STREAM_ID" "$INTERFACE"

