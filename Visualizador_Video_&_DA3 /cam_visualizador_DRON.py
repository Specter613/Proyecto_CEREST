"""

Codigo meramente de prueba para visualizar el stream de video del dron en Linux usando OpenCV y FFmpeg.
El visor multihilo permite atrapar imágenes a máxima velocidad sin bloquear la interfaz.

--- Visualizador de video en tiempo real desde el stream del dron ---

"""


import cv2
import os
import threading
import time

# Silenciar advertencias
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,udp,rtp|fflags;nobuffer|flags;low_delay"

class VisorRover:
    def __init__(self, fuente):
        print("Iniciando motor de recepción FPV (Modo FFmpeg)...")
        
        # Le pasamos la variable 'fuente' (tu archivo dron.sdp) y usamos FFmpeg
        self.cap = cv2.VideoCapture(fuente, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            print("Error fatal: No se pudo abrir la cámara. Revisa wfb_rx.")
            os._exit(1)
            
        self.ret, self.frame = self.cap.read()
        self.corriendo = True
        
        # Arrancamos el trabajador en segundo plano
        self.hilo = threading.Thread(target=self._actualizar, args=())
        self.hilo.daemon = True
        self.hilo.start()

    def _actualizar(self):
        # El bucle infinito del hilo que atrapa imágenes a máxima velocidad
        while self.corriendo:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                self.ret = ret

    def leer(self):
        # Nos entrega la imagen más reciente al instante
        return self.ret, self.frame

    def detener(self):
        self.corriendo = False
        self.hilo.join()
        self.cap.release()

# --- BUCLE PRINCIPAL ---
if __name__ == '__main__':
    # Inicializamos nuestra cámara multihilo apuntando a tu archivo SDP
    camara = VisorRover("dron.sdp")
    
    # Le damos 1 segundo al hilo para atrapar el primer I-Frame
    time.sleep(1)

    print("Desplegando interfaz. Presiona 'q' para salir.")
    
    while True:
        ret, frame = camara.leer()
        
        if ret and frame is not None:
            # Aquí la imagen fluye como mantequilla
            cv2.imshow(' MODULAR - Vision de Dron', frame)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    print("Cerrando comunicaciones...")
    camara.detener()
    cv2.destroyAllWindows()