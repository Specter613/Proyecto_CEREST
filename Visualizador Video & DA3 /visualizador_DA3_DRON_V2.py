"""
Mapa de calor de profundidad con Depth Anything 3 sobre el stream de video del dron.
- Hilo receptor: atrapa frames del stream (SDP/FFmpeg) a máxima velocidad.
- Hilo de profundidad: toma el frame más reciente e infiere con DA3.
- Hilo principal: muestra video y profundidad lado a lado en una sola ventana.
Presiona 'q' para salir.
"""

import os

# Silenciar advertencias y configurar FFmpeg ANTES de importar cv2
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,udp,rtp|fflags;nobuffer|flags;low_delay"

import cv2
import numpy as np
import torch
import threading
import time
from contextlib import nullcontext

from depth_anything_3.api import DepthAnything3

# Ruta al SDP relativa a este script
SDP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dron.sdp")

# Tamaño al que se encoge el frame antes de pasarlo al modelo
INFER_SIZE = (426, 320)

# Alto de cada panel en la ventana (el ancho se ajusta manteniendo proporción)
DISPLAY_HEIGHT = 480

WINDOW_NAME = "MODULAR - Video | Profundidad DA3"


class VisorRover:
    def __init__(self, fuente):
        print("Iniciando motor de recepción FPV (Modo FFmpeg)...")
        self.cap = cv2.VideoCapture(fuente, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            print(f"Error fatal: No se pudo abrir el stream ({fuente}). Revisa wfb_rx.")
            os._exit(1)

        self.lock = threading.Lock()
        self.frame = None
        self.frame_id = 0
        self.fps = 0.0
        self.corriendo = True

        self.hilo = threading.Thread(target=self._actualizar, daemon=True)
        self.hilo.start()

    def _actualizar(self):
        conteo = 0
        t_ref = time.time()
        while self.corriendo:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.frame_id += 1
                conteo += 1
                ahora = time.time()
                if ahora - t_ref >= 1.0:
                    self.fps = conteo / (ahora - t_ref)
                    conteo = 0
                    t_ref = ahora
            else:
                time.sleep(0.005)

    def leer(self):
        with self.lock:
            if self.frame is None:
                return None, self.frame_id
            return self.frame.copy(), self.frame_id

    def detener(self):
        self.corriendo = False
        self.hilo.join(timeout=2)
        self.cap.release()


class TrabajadorProfundidad:
    def __init__(self, camara, model, amp_ctx):
        self.camara = camara
        self.model = model
        self.amp_ctx = amp_ctx

        self.lock = threading.Lock()
        self.depth_colored = None
        self.fps = 0.0
        self.corriendo = True

        self.hilo = threading.Thread(target=self._bucle, daemon=True)
        self.hilo.start()

    def _bucle(self):
        ultimo_id = -1
        while self.corriendo:
            frame, frame_id = self.camara.leer()
            if frame is None or frame_id == ultimo_id:
                time.sleep(0.002)
                continue
            ultimo_id = frame_id

            t0 = time.time()
            small_frame = cv2.resize(frame, INFER_SIZE)
            rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

            with torch.no_grad():
                with self.amp_ctx():
                    prediction = self.model.inference([rgb_frame])

            depth = prediction.depth[0]
            if torch.is_tensor(depth):
                depth = depth.float().cpu().numpy()

            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)

            with self.lock:
                self.depth_colored = depth_colored
                self.fps = 1.0 / (time.time() - t0)

    def leer(self):
        with self.lock:
            return self.depth_colored, self.fps

    def detener(self):
        self.corriendo = False
        self.hilo.join(timeout=5)


def a_alto(img, alto):
    # Redimensiona manteniendo proporción a un alto fijo
    h, w = img.shape[:2]
    return cv2.resize(img, (int(w * alto / h), alto))


def main():
    use_cuda = torch.cuda.is_available()
    device = torch.device('cuda' if use_cuda else 'cpu')
    print(f"Iniciando inferencia en: {device}")

    if not use_cuda:
        # DA3 llama a torch.cuda.is_bf16_supported(), que truena sin GPU
        torch.cuda.is_bf16_supported = lambda *args, **kwargs: True
        print("AVISO: PyTorch no detecta GPU, corriendo en CPU (será lento).")

    model_name = "depth-anything/DA3-BASE" if use_cuda else "depth-anything/DA3-SMALL"
    print(f"Cargando modelo {model_name}...")
    model = DepthAnything3.from_pretrained(model_name).to(device).eval()

    amp_ctx = (lambda: torch.autocast(device_type='cuda', dtype=torch.float16)) if use_cuda else nullcontext

    camara = VisorRover(SDP_PATH)

    print("Esperando primer frame del dron...")
    t0 = time.time()
    while camara.leer()[0] is None:
        if time.time() - t0 > 10:
            print("No llegó ningún frame en 10 s. Revisa el enlace.")
            camara.detener()
            return
        time.sleep(0.05)

    profundidad = TrabajadorProfundidad(camara, model, amp_ctx)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("Desplegando interfaz. Presiona 'q' para salir.")

    while True:
        frame, _ = camara.leer()
        if frame is not None:
            panel_rgb = a_alto(frame, DISPLAY_HEIGHT)

            depth_colored, fps_da3 = profundidad.leer()
            if depth_colored is not None:
                panel_depth = cv2.resize(depth_colored, (panel_rgb.shape[1], panel_rgb.shape[0]))
            else:
                panel_depth = np.zeros_like(panel_rgb)
                cv2.putText(panel_depth, "Calculando profundidad...", (10, DISPLAY_HEIGHT // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.putText(panel_rgb, f"Stream: {camara.fps:.1f} FPS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(panel_depth, f"DA3: {fps_da3:.1f} FPS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            combinado = cv2.hconcat([panel_rgb, panel_depth])
            cv2.imshow(WINDOW_NAME, combinado)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Cerrando comunicaciones...")
    profundidad.detener()
    camara.detener()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()