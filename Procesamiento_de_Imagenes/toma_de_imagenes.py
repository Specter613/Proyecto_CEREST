"""

Codigo para captura de imágenes topográficas desde el stream de video del dron para reconstrucción 3D con COLMAP.
El visor multihilo permite atrapar imágenes a máxima velocidad sin bloquear la interfaz.
Se guardan imágenes en la carpeta 'dataset_colmap' con resolución reducida (480p) para ahorrar VRAM y espacio en disco.
Se requiere el uso de SW externo (COLMAP) para la reconstrucción 3D posterior. ('colmap gui' en la terminal)

--- Post-Procesamiento para reconstrucción de nube de puntos en 3D con COLMAP ---

"""

import cv2
import os
import threading
import time

# Silenciar advertencias
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,udp,rtp|fflags;nobuffer|flags;low_delay"

# Crear la carpeta donde se guardarán las fotos topográficas
carpeta_datos = "dataset_1"
if not os.path.exists(carpeta_datos):
    os.makedirs(carpeta_datos)

class VisorRover:
    def __init__(self, fuente):
        print("Iniciando motor de recepción FPV para mapeo...")
        self.cap = cv2.VideoCapture(fuente, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            print("Error fatal: No se pudo abrir la cámara.")
            os._exit(1)
            
        self.ret, self.frame = self.cap.read()
        self.corriendo = True
        
        self.hilo = threading.Thread(target=self._actualizar, args=())
        self.hilo.daemon = True
        self.hilo.start()

    def _actualizar(self):
        while self.corriendo:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                self.ret = ret

    def leer(self):
        return self.ret, self.frame

    def detener(self):
        self.corriendo = False
        self.hilo.join()
        self.cap.release()

# --- BUCLE PRINCIPAL ---
if __name__ == '__main__':
    camara = VisorRover("dron.sdp")
    time.sleep(1)

    imagenes_guardadas = 0
    # Queremos tomar 1 foto cada 0.5 segundos (2 FPS)
    intervalo_segundos = 0.4 
    tiempo_ultimo_guardado = time.time()
    
    print(f"Recolector Topográfico iniciado. Guardando en ./{carpeta_datos}/")
    print("Mueve el dron/rover alrededor del objetivo. Presiona 'q' para detener la captura.")
    
    while True:
        ret, frame = camara.leer()
        
        if ret and frame is not None:
            # Ventana para que el piloto vea por dónde va
            cv2.imshow('MODULAR - Captura de dataset', frame)
            
            tiempo_actual = time.time()
            if tiempo_actual - tiempo_ultimo_guardado >= intervalo_segundos:
                # REDUCCIÓN TÁCTICA
                # frame_reducido = cv2.resize(frame, (854, 480))
                frame_reducido = frame  # Mantener resolución original para COLMAP

                # Guardamos con formato img_0001.jpg, img_0002.jpg...
                nombre_archivo = f"{carpeta_datos}/img_{imagenes_guardadas:04d}.jpg"
                cv2.imwrite(nombre_archivo, frame_reducido)
                
                print(f"Captura exitosa: {nombre_archivo}")
                imagenes_guardadas += 1
                tiempo_ultimo_guardado = tiempo_actual
                
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    print(f"Misión completada. {imagenes_guardadas} imágenes listas para reconstrucción.")
    camara.detener()
    cv2.destroyAllWindows()
