import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

PUERTO = "/dev/ttyACM0"
BAUD = 115200
NUM_MUESTRAS = 1000

ser = serial.Serial(PUERTO, BAUD, timeout=1)
time.sleep(2)

# 1. INICIAR STREAMING
ser.write(b"STREAMMAG\r\n")
time.sleep(0.1)

muestras = []
print(f"Recolectando {NUM_MUESTRAS} muestras...")
print("GIRA EL SENSOR LENTAMENTE en figura de 8...")

while len(muestras) < NUM_MUESTRAS:
    linea = ser.readline().decode(errors='ignore').strip()
    partes = linea.split(',')
    if len(partes) == 3:
        try:
            x, y, z = float(partes[0]), float(partes[1]), float(partes[2])
            muestras.append((x, y, z))
            if len(muestras) % 50 == 0:
                print(f"  {len(muestras)}/{NUM_MUESTRAS}")
        except ValueError:
            continue

# 2. DETENER STREAMING E INMEDIATAMENTE VACIAR BUFFERS
print("\nDeteniendo streaming...")
ser.write(b"x\r\n")
time.sleep(0.5)

# Vaciar cualquier residuo que haya quedado volando en el bus USB
ser.reset_input_buffer()
ser.reset_output_buffer()

# 3. CÁLCULO MATEMÁTICO DE CALIBRACIÓN
datos = np.array(muestras)
SCALA = 1000.0

x, y, z = datos[:,0]/SCALA, datos[:,1]/SCALA, datos[:,2]/SCALA
M = np.stack([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z], axis=1)
b = np.ones(len(x))
sol, *_ = np.linalg.lstsq(M, b, rcond=None)
A,B,C,D,E,F,G,H,I = sol

Q = np.array([[A,D,E],[D,B,F],[E,F,C]])
u = np.array([G,H,I])
offset = -np.linalg.solve(Q, u)

valores, vectores = np.linalg.eigh(Q)
raiz_Q = vectores @ np.diag(np.sqrt(np.abs(valores))) @ vectores.T
matriz = raiz_Q / np.mean(np.sqrt(np.abs(valores)))

corregidos = (matriz @ (np.stack([x,y,z],axis=1) - offset).T).T
radios = np.linalg.norm(corregidos, axis=1)
calidad = radios.std() / radios.mean()

print(f"\nOffset: X={offset[0]:.4f} Y={offset[1]:.4f} Z={offset[2]:.4f}")
print(f"Calidad del ajuste: {calidad*100:.2f}%")

# 4. ENVIAR COMANDO DE CALIBRACIÓN AL STM32
comando = f"SETCALMAG:{offset[0]:.4f},{offset[1]:.4f},{offset[2]:.4f}," \
          f"{matriz[0,0]:.4f},{matriz[0,1]:.4f},{matriz[0,2]:.4f}," \
          f"{matriz[1,0]:.4f},{matriz[1,1]:.4f},{matriz[1,2]:.4f}," \
          f"{matriz[2,0]:.4f},{matriz[2,1]:.4f},{matriz[2,2]:.4f}\r\n"

print(f"\nEnviando calibración al STM32...")
ser.write(comando.encode('utf-8'))
time.sleep(0.5)

# 5. LEER RESPUESTA DEL STM32
while ser.in_waiting:
    linea = ser.readline().decode(errors='ignore').strip()
    if linea:
        print(f"Respuesta STM32: {linea}")

ser.close()

# 6. MOSTRAR GRÁFICAS (Al final de todo para no bloquear la transmisión)
fig = plt.figure(figsize=(12, 5))

ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(datos[:,0], datos[:,1], datos[:,2], s=8, alpha=0.6)
ax1.set_title('Datos Crudos')

ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(corregidos[:,0], corregidos[:,1], corregidos[:,2], s=8, color='green', alpha=0.6)
ax2.set_title(f'Corregido ({calidad*100:.2f}%)')

plt.tight_layout()
plt.show()