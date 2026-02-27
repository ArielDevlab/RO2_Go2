# Instalación del modelo en RViz 
Es necesario, iniciar el ambiente en ROS2 antes de cualquier modificacion o abrir algun archivo para su visualización

```
source /opt/ros/humble/setup.bash
```

## 1. Obtención y estructura de los archivos
Los modelos oficiales del Go2 se pueden obtener a través de los repositorios de GitHub de Unitree (como unitree_ros2) o mediante adaptaciones de la comunidad basadas en los SDK oficiales para ROS 2 (Humble, Jazzy, etc.).

Para que la visualización funcione, el paquete descargado depende de dos directorios clave:
* Carpeta meshes/: Contiene los archivos .stl o .dae que representan la geometría 3D de cada eslabón del cuadrúpedo (chasis, cadera, muslo y pantorrilla).
* Carpeta urdf/: Contiene el archivo go2.urdf (o .xacro). Este archivo actúa como el esqueleto matemático; utiliza etiquetas <mesh filename="package://..."/> para mandar a llamar a los archivos .stl y los enlaza definiendo los ejes de rotación y los límites de cada motor.

## 2. Configuración en el Workspace
Para integrar estos modelos en tu entorno, es necesario compilar el paquete de descripción dentro de tu espacio de trabajo (workspace) para que ROS 2 pueda indexar las rutas de las mallas:

Clona el repositorio que contiene la descripción del Go2 dentro del directorio src de tu workspace (por ejemplo, ~/ros2_ws/src).

Regresa a la raíz del workspace (cd ~/ros2_ws) e instala cualquier dependencia faltante mediante rosdep.

Compila el paquete. El uso de enlaces simbólicos es muy útil aquí para no tener que recompilar si modificas el URDF:

* Crear el nuevo Workspace y la carpeta src
Abre una terminal y ejecuta el siguiente comando. El parámetro -p le indica al sistema que cree tanto la carpeta principal como la subcarpeta src de una sola vez:

```
mkdir -p ~/go2_ws/src
```

* Mover tu paquete al Workspace
Ahora debes colocar la carpeta descomprimida GO2_URDF (la que contiene los archivos .xacro, mallas, etc.) exactamente dentro de la carpeta src que acabas de crear.

Puedes hacerlo arrastrando la carpeta gráficamente desde tu explorador de archivos, o usando la terminal:

```
# Asumiendo que GO2_URDF está en tu carpeta de Descargas:
mv ~/Downloads/GO2_URDF ~/go2_ws/src/
```
Compilar el nuevo espacio de trabajo
Una vez que el paquete está en su lugar, debes compilar este nuevo workspace para que el sistema reconozca los ejecutables y las rutas de las mallas 3D. Ve a la raíz de tu nuevo espacio:

```
cd ~/go2_ws
```
Dependiendo de la versión que vayas a utilizar para levantar este paquete, ejecuta el comando de compilación:

Si usas ROS 1 (Noetic):

```
catkin_make
```
Si usas ROS2 (Humble/Jazzy, tras adaptar los CMakeLists y launch):

```
colcon build
```
Para que colcon build compile el paquete con éxito, necesitamos "traducir" dos archivos en la raíz de tu carpeta GO2_URDF (o go2_description): el ***CMakeLists.txt*** y el ***package.xml***.

1. Actualizar el CMakeLists.txt
   Abre el archivo CMakeLists.txt de tu paquete. Borra todo su contenido (que está buscando catkin) y reemplázalo con esta estructura estándar de ROS 2 para paquetes de descripción (que no requieren compilar código en C++, solo instalar archivos de visualización):

```
cmake_minimum_required(VERSION 3.8)
project(go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscamos el sistema de construcción de ROS 2
find_package(ament_cmake REQUIRED)

# Instruimos al sistema que instale todas las carpetas clave en el workspace
install(
  DIRECTORY config dae launch meshes urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```
   
3. Actualizar el package.xml
   Abre el archivo package.xml. Este archivo describe las dependencias. Busca cualquier línea que diga <buildtool_depend>catkin</buildtool_depend> y cámbiala por ament_cmake.

Tu archivo debería quedar similar a esto (presta especial atención a la etiqueta <export> al final):

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>go2_description</name>
  <version>0.0.0</version>
  <description>Paquete URDF del Unitree Go2 migrado a ROS 2</description>
  <maintainer email="tu_correo@email.com">AxXt</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
3. Crear el archivo de lanzamiento en Python
   Para abrir los modelos, hay un detalle importante: en la estructura original que descargaste, los archivos de la carpeta launch (go2_rviz.launch y gazebo.launch) están escritos en XML, que era el formato de ROS 1. En ROS 2, los archivos de lanzamiento se escriben en Python (.launch.py) para darte mucho más control sobre los nodos.

   * Navega a la carpeta de lanzadores de tu paquete:
```
cd ~/go2_ws/src/GO2_URDF/launch
```

   * Crea un archivo llamado **display.launch.py**:
```
nano display.launch.py
```

   * Pega el siguiente código en Python. Este script inicializa el **robot_state_publisher** (que traduce el URDF), el **joint_state_publisher_gui** (la ventana de los deslizadores) y rviz2:

```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Buscamos la ruta de tu paquete compilado
    pkg_share = get_package_share_directory('go2_description')
    
    # Ruta exacta al archivo xacro maestro
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')
    
    # Comando para convertir xacro a URDF en tiempo real
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Nodo 1: Publicador del estado del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Nodo 2: Interfaz gráfica para mover las articulaciones
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    
    # Nodo 3: RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
```
   
5. Recompilar
   Una vez guardados los archivos, regresa a la terminal en la raíz de tu workspace (~/go2_ws) y vuelve a intentar la compilación:

```
colcon build --packages-select go2_description
```
(Nota: Asegúrate de que el nombre del directorio dentro de src coincida con el nombre en los archivos, en este caso go2_description o ajusta el nombre en el CMakeLists a GO2_URDF).

* "Conectar" tu terminal al Workspace (Sourcing)
Este es el paso más importante y el que más errores suele causar. Para que comandos como roslaunch o ros2 launch encuentren tu paquete GO2_URDF, tu terminal actual debe "saber" que este nuevo workspace existe.

Desde la raíz de tu workspace (~/go2_ws), ejecuta:
Para ROS 1:
```
source devel/setup.bash
```
Para ROS 2: 
```
source install/setup.bash
```
¡Listo! A partir de este momento, en esa terminal ya puedes ejecutar los archivos .launch del Unitree Go2 y visualizar los modelos en RViz o simular la física en Gazebo usando este entorno dedicado. Cada vez que abras una terminal nueva para trabajar con este robot, solo recuerda ejecutar el comando source. 

## 3. Ejecución y renderizado en RViz
A diferencia de visualizar una sola pieza estática, un robot con múltiples grados de libertad requiere que el sistema calcule las transformaciones espaciales (TF) de cada eslabón en tiempo real para que RViz ensamble los .stl correctamente. Esto se logra mediante dos nodos esenciales:

* robot_state_publisher: Lee el archivo URDF y publica la posición estática de las articulaciones.

* joint_state_publisher_gui: Despliega una ventana con controles deslizantes que te permitirá mover virtualmente cada articulación del Go2 y ver cómo reaccionan las mallas de las patas.

Por lo general, el paquete incluye un archivo de lanzamiento (launch file) que automatiza la apertura de RViz con estos nodos. El comando de ejecución suele ser similar a este:

```
ros2 launch go2_description display.launch.py
```
** (El nombre del paquete o del archivo .launch.py puede variar ligeramente dependiendo de la rama específica del repositorio que estés utilizando). **
