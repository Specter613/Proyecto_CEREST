# Guía de acondicionamiento de estación en tierra

Esta pequeña guía fue aplicada en la distro **Ubuntu 24.04 LTS Noble**, con **Python 3.12.3**, por lo que el uso de alguna otra versión puede significar una alteración en el funcionamiento del visualizador o en la compilación de dependencias.

---

## PRERREQUISITOS DEL SISTEMA

Antes de configurar el entorno de Python, es fundamental instalar dependencias a nivel de sistema que Nerfstudio y el adaptador de red requerirán:

1. **Controladores del adaptador de red:** Asegúrate de tener instalados los drivers DKMS para el adaptador RTL8812AU, necesarios para que los scripts puedan interactuar correctamente con la interfaz y recibir la transmisión del Eachine Sphere Link VTX.
2. **Instalación de COLMAP:** Nerfstudio utiliza HLOC y COLMAP para generar la nube de puntos inicial y calcular la pose de las cámaras. Instálalo en tu sistema ejecutando:
   ```bash
   sudo apt update && sudo apt install colmap
   ```

---

## INSTALACIÓN

Para la reconstrucción 3D de nuestro entorno se requiere realizar una serie de pasos previos, estos mismos tienen como objetivo obtener Nerfstudio y las herramientas necesarias para su correcta aplicación, por lo que se pide atentamente ejecutarlas en el orden que se brinda.

**1.** Comenzaremos por crear un espacio de trabajo en donde estaremos almacenando nuestras reconstrucciones y entrenamientos:
```bash
mkdir reconstruccion_3D
```

**2.** Accedemos a la carpeta que creamos:
```bash
cd reconstruccion_3D/
```

**3.** Posteriormente, se crea un entorno virtual en donde aislaremos las herramientas de Python que estaremos utilizando:
```bash
python3 -m venv nerfstudio-env
```

**4.** Activamos nuestro entorno virtual (Este paso se debe realizar cada vez que querramos ejecutar Nerfstudio):
```bash
source nerfstudio-env/bin/activate
```

**5.** Instalamos las dependencias necesarias para Nerfstudio:
```bash
pip install --upgrade pip setuptools wheel ninja
pip install torch==2.1.2 torchvision==0.16.2 --extra-index-url https://download.pytorch.org/whl/cu118
```

**6.** Instalamos Nerfstudio:
```bash
pip install nerfstudio
```
> **Nota de compatibilidad (tiny-cuda-nn):** Dado que Ubuntu 24.04 utiliza compiladores GCC recientes que pueden entrar en conflicto con CUDA 11.8, si este paso falla al compilar `tinycudann`, puede que necesites forzar el uso de una versión anterior de `g++` (como `g++-11`) en tu sistema antes de reinstalar.

**7.** Comprobamos que la instalación fue exitosa:
```bash
ns-train --help
```

---

## APLICACIÓN

Una vez inicializado nuestro entorno virtual (paso 4), con apoyo del script de Python `toma_de_imagenes.py` anexo en este mismo directorio, generamos un dataset con la cámara a bordo en el dron. 

Para ello, haremos uso de un adaptador de red RTL8812AU mediante USB, y lo inicializaremos con apoyo de los scripts `.sh` / `.zsh`. Si se tiene terminal BASH usa el `CAM_START.sh`, si es ZSH usa `CAM_START.zsh`. Se deben dar permisos de ejecución a estos scripts:
```bash
chmod +x CAM_START.sh
```
> **Nota:** Dichos scripts también están anexos en este directorio y pueden requerir de un cambio en la variable `INTERFACE` según el nombre de la interfaz de la antena que se utilice. Aseguran que la interfaz esté lista para interceptar el video.

Una vez lista nuestra estación en tierra, dejaremos la terminal abierta para que el proceso no se detenga. 

En seguida ejecutaremos el script `toma_de_imagenes.py`. Este creará una carpeta llamada `dataset_1`, aquí se guardarán las imágenes adquiridas por el dron. *(Asegúrate de que la conexión sea estable para evitar la pérdida de paquetes UDP, ya que la nitidez de los fotogramas es crucial para una buena reconstrucción).*

Con nuestro dataset creado, daremos instrucciones a Nerfstudio para el procesamiento de tal dataset. Para ello indicaremos la dirección de la carpeta con estas imágenes y la dirección de la carpeta destino; en este caso, esta última se creará y se le asignará el nombre de `dataset_1_procesado`:

```bash
ns-process-data images --data /home/{TU_USUARIO}/reconstruccion_3D/dataset_1 --output-dir /home/{TU_USUARIO}/reconstruccion_3D/dataset_1_procesado --sfm-tool hloc 
```

Una vez terminado este proceso, usaremos esta nueva carpeta para el entrenamiento del modelo *Splatfacto*:

```bash
ns-train splatfacto --data /home/{TU_USUARIO}/reconstruccion_3D/dataset_1_procesado
```
	
Al ejecutarse, podremos visualizar el entrenamiento en tiempo real en nuestro navegador; dicha dirección es otorgada por la salida en la terminal del entrenamiento, que generalmente es la siguiente:
```text
http://0.0.0.0:7007/
```