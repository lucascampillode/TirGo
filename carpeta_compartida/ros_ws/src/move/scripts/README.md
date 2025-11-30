# Navegación y Uso de run_all.sh

Script orquestador en **Bash** encargado de levantar el sistema mínimo de navegación del robot TIAGo para el proyecto.

Automatiza la carga secuencial de **RViz**, el **Servidor de Mapas** y el nodo de **Lógica de Navegación** (`checkpointfollower.py`), gestionando los tiempos de espera necesarios entre procesos para asegurar una carga correcta y evitar conflictos.

---

## 1. Ubicación en el Paquete

Este script se encuentra dentro de la carpeta `scripts/` del paquete `move`. Asume la siguiente estructura de archivos relativa:

```
text
move/
├── launch/
│   └── rviz.launch              ← Configuración de visualización
├── maps/
│   ├── Mapa_aula_mod_1.0.yaml   ← Metadatos del mapa
│   └── Mapa_aula_mod_1.0.pgm    ← Imagen del mapa
└── scripts/
    ├── run_all.sh               ← ESTE SCRIPT
    └── checkpointfollower.py    ← Nodo de lógica de movimiento
```

## 2.Qué hace este script

El script ejecuta una secuencia lineal de arranque diseñada para evitar condiciones de carrera (race conditions):

   1. Carga el Workspace: Hace source del entorno de trabajo local (devel/setup.bash).

   2. Lanza RViz: Ejecuta roslaunch move rviz.launch en segundo plano (background) y guarda su PID.

   3. Espera (4s): Da tiempo a que la interfaz gráfica de RViz cargue completamente.

   4. Carga el Mapa: Lanza el map_server apuntando al archivo .yaml definido.

   5. Espera (5s): Asegura que el mapa esté publicado y disponible en el topic /map antes de continuar, este tiempo puede ser ajustado pero con menos tiempo no carga correctamente.

   6. Lanza la Lógica: Ejecuta el nodo de control checkpointfollower.py.

   7. Gestión de Procesos: Mantiene el script vivo (wait) mientras los nodos hijos sigan corriendo.
## 3.Dependencias del Sistema

Para que este script funcione sin errores, necesitas:

ROS Noetic instalado.
Paquete map_server:
  ``` Bash
  sudo apt-get install ros-noetic-map-server
  ```
    * Tener compilado el workspace (catkin_make) para que devel/setup.bash exista.

    * Los archivos del mapa (.yaml y .pgm) deben existir en la ruta especificada dentro de src/move/maps/.

## 4. Uso
 ```bash
    cd carpeta_compartida/ros_ws/src/move/scripts/
    ./run_all.sh
 ```
 ## 5. Nodos gestionados

Este script es responsable de levantar los siguientes componentes:

   * RViz (rviz): Visualización del estado del robot, mapa y sensores.

   * Map Server (map_server): Publica el mapa estático del aula en /map.

   * Checkpoint Follower (checkpointfollower.py): Cliente Python que envía al robot a las posiciones objetivo.
    
