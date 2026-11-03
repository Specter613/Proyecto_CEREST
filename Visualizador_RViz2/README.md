# Uso de visualizador de RViz2

## Preparación del entorno
Para el correcto funcionamiento del visualizador, se requiere el uso de herramientas como ROS 2 Jazzy Jaliso y el mismo RViz2, por lo que su previa instalación es indispensable, este proceso lo pueden encontrar dentro de la documentación oficial de ROS 2
[ROS2 Documentation](https://docs.ros.org/en/jazzy/Installation.html)

Después, conectaremos nuestra PC a la red generada por nuestra emisora, en nuestro caso una hecha por ExpressLRS, esto con el fin de poder recibir los datos de telemetría vía MavLink

**Nota:**Este trabajo es 100% compatible con Ubuntu 24.04 Noble LTS.


## Ejecución

Con nuestras herrameintas listas, compliaremos nuestro paquete desde la carpeta visrviz_ws

   ```bash
   colcon build
   ```

Posteriormente, añadimos el PATH de este directorio

   ```bash
   source install/setup.zsh #Reemplaza .zsh por .bash si usas BASH

   ```
Por útlimo, ejecutaremos el archvio de lanzamiento de nuestro paquete
   ```bash
   ros2 launch dron_sim_pkg dron_sim.launch.py

   ```

