import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'dron_sim_pkg'

    # 2. Obtener las rutas de los archivos dentro del paquete instalado
    pkg_share = get_package_share_directory(package_name)
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'dron_urdf.urdf')
    
    # Opcional: Ruta a un archivo de configuración de RViz para guardar la vista
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'drone.rviz')

    # 3. Leer el contenido del URDF para pasarlo como parámetro
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Nodo A: robot_state_publisher
    # Se encarga de publicar la estructura del robot basada en el URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Nodo B: Tu script puente de MAVLink a TF
    mavlink_bridge_node = Node(
        package=package_name,
        executable='mavlink_tf_bridge', # El nombre del ejecutable definido en tu setup.py
        name='mavlink_tf_bridge',
        output='screen'
    )

    # Nodo C: RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_path] # Descomenta esto cuando hayas guardado tu configuración en RViz
    )

    # Retornar la descripción del lanzamiento con los 3 nodos
    return LaunchDescription([
        robot_state_publisher_node,
        mavlink_bridge_node,
        rviz2_node
    ])