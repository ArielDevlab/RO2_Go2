# Instalación del modelo en Gazebo Ignition 
Es necesario, iniciar el ambiente en ROS2 antes de cualquier modificacion o abrir algun archivo para su visualización

```
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

```

## 1. Instalar el puente de comunicación de Ignition
Primero, vamos a descargar los paquetes que conectan ROS 2 con la nueva versión de Gazebo.

```
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge -y
```

## 2. Crear el nuevo lanzador de Python
Vamos a crear un archivo específico para arrancar la nueva versión del simulador.

```
nano ~/go2_ws/src/GO2_URDF/launch/ignition.launch.py
```

Copia y pega el siguiente código. Este script hace lo mismo que el anterior, pero usando los nodos modernos de ros_gz_sim y soltando al robot a 0.5 metros de altura (-z 0.5) para que veas cómo cae y la física interactúa:

```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')

    # Convertir Xacro a URDF
    robot_desc = Command(['xacro ', xacro_file])

    # Nodo publicador (Esqueleto matemático)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Iniciar Gazebo Ignition (cargando un mundo vacío con luz y suelo)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # Inyectar el modelo en Ignition
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'go2', '-topic', 'robot_description', '-z', '0.5'],
        output='screen',
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn
    ])
```

## 3. Actualizar el CMakeLists.txt
   Como agregamos un archivo nuevo, le damos un repaso rápido a colcon para que lo reconozca.

```
cd ~/go2_ws
colcon build --packages-select go2_description --symlink-install
```
   
## 4. Actualizar el package.xml
   Esta es la parte vital. Ignition es estricto con la ubicación de los archivos 3D. Antes de lanzar el comando, debemos exportar una variable de entorno para decirle exactamente dónde está la "piel" del robot.

Ejecuta estas tres líneas juntas:

```
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=~/go2_ws/install/go2_description/share:$IGN_GAZEBO_RESOURCE_PATH
```

## 5. ¡Y ahora, el momento de la verdad! Lanza tu nuevo archivo:

```
ros2 launch go2_description ignition.launch.py
```

  
