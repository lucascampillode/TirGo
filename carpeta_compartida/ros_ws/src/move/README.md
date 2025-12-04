
````markdown
# Paquete `move` – Navegación TIAGo + Checkpoint Follower

Este paquete contiene la **lógica mínima de navegación** del robot TIAGo para el proyecto **TirGo-Pharma**. Su función principal es gestionar la navegación autónoma a través de una lista de puntos predefinidos.

El sistema permite:
- Lanzar **RViz** con una configuración predefinida.
- Cargar el **mapa estático** del aula.
- Ejecutar el nodo de navegación `checkpointfollower.py`, que envía al robot a una lista de checkpoints.

---

## 1. Estructura del paquete

```text
move/
├── CMakeLists.txt
├── package.xml
├── configs/
│   └── rviz_configs.rviz       # Configuración visual de RViz
├── launch/
│   └── rviz.launch             # Lanzador de visualización
├── maps/
│   ├── Mapa_aula.yaml          # Metadatos del mapa
│   └── Mapa_aula.pgm           # Imagen del mapa
├── scripts/
│   ├── run_all.sh              # Script de ejecución principal
│   └── run_test.sh             # Script para pruebas
└── src/
    └── move/
        ├── checkpointfollower.py   # Lógica principal de navegación
        ├── publish_initial_pose.py # Publicador de pose inicial
        ├── comunicacion_test.py    # Test de topics
        └── test_puntos.py          # Validación de coordenadas
````

-----

## 2\. Requisitos

  * **ROS 1 Noetic** (o distribución compatible).
  * Workspace configurado con el paquete `move` compilado.
  * Acceso a TIAGo real o simulación (Gazebo) funcionando.
  * Dependencias Python:
      * `rospy`
      * `numpy`
      * `tf.transformations`

-----

## 3\. Uso

### 3.1. Preparación

Asegúrate de que la simulación o el robot real están encendidos y con `roscore` activo. Realiza el *source* de tu entorno habitual.

### 3.2. Ejecución del sistema

El script principal se encarga de levantar RViz, el servidor de mapas y el nodo de navegación en orden.

```bash
cd ~/carpeta_compartida/ros_ws
source devel/setup.bash

# Ejecutar el script de orquestación
roscd move/scripts
./run_all.sh
```

El robot debería aparecer en RViz localizado en el mapa y comenzar la ruta de checkpoints automáticamente.

-----

## 4\. Documentación detallada

Para más detalles sobre cada componente, consulta los README específicos:

  * [Scripts y ejecución](./scripts/README.md)
  * [Launch files](./launch/README.md)
  * [Nodos y código fuente](./src/move/README.md)
  * [Configuración](./configs/README.md)

<!-- end list -->

```
