<div align="center">

# move

Paquete de **navegación del robot TIAGo** para el sistema **TirGoPharma**.

Implementa la **lógica mínima y controlada de movimiento autónomo**
necesaria para la demo: desplazamiento entre puntos clave
(dispensador ↔ paciente) y publicación de **hitos de navegación**
para el coordinador de misión.

Compatible con **ROS 1 (Noetic)**.

</div>

---

## Visión general

El paquete **`move`** encapsula la navegación del robot TIAGo
mediante una lista de **checkpoints predefinidos** sobre un mapa estático.

Su objetivo **no es** desarrollar un stack completo de navegación,
sino proporcionar un **comportamiento fiable, reproducible y trazable**
para la demo end-to-end de TirGoPharma.

Este paquete se encarga de:

- Lanzar **RViz** con una configuración predefinida
- Cargar el **mapa estático** del entorno
- Ejecutar el nodo de navegación `checkpointfollower.py`
- Publicar flags ROS que indican el progreso de la navegación
  (llegada al dispensador, llegada al paciente, etc.)

El coordinador de misión (`tirgo_mission_server`) utiliza estos flags
para avanzar su **máquina de estados**.

---

## 1. Estructura del paquete

```text
move/
├── CMakeLists.txt
├── package.xml
├── configs/
│   └── rviz_configs.rviz           # Configuración visual de RViz
├── launch/
│   └── rviz.launch                 # Lanzador de RViz
├── maps/
│   ├── Mapa_aula.yaml              # Metadatos del mapa
│   └── Mapa_aula.pgm               # Imagen del mapa
├── scripts/
│   ├── run_all.sh                  # Script principal de ejecución
│   └── run_test.sh                 # Script de pruebas
└── src/
    └── move/
        ├── checkpointfollower.py   # Lógica principal de navegación
        ├── publish_initial_pose.py # Publicador de pose inicial
        ├── comunicacion_test.py    # Test de comunicación por topics
        └── test_puntos.py          # Validación de coordenadas
````

---

## 2. Rol dentro de TirGoPharma

Dentro del sistema global, el paquete **`move`** actúa como
**módulo de ejecución de navegación**.

* **Recibe órdenes implícitas** desde el coordinador de misión
* Ejecuta el desplazamiento físico del robot
* **Publica flags ROS** que indican la finalización de cada tramo

Ejemplos de flags publicados (orientativos):

* `/tirgo/tiago/arrived`
* `/tirgo/tiago/at_patient`

Estos flags permiten que la misión avance de forma **determinista y trazable**.

---

## 3. Requisitos

* **ROS 1 Noetic** (o distribución compatible)
* Workspace con el paquete `move` compilado
* TIAGo real o simulación funcionando (Gazebo)
* Dependencias Python:

  * `rospy`
  * `numpy`
  * `tf.transformations`

---

## 4. Uso

### 4.1 Preparación

Antes de ejecutar el paquete:

1. Asegúrate de que el robot o la simulación están activos
2. Lanza `roscore` si no está ya en ejecución
3. Carga el entorno de tu workspace

```bash
cd ~/carpeta_compartida/ros_ws
source devel/setup.bash
```

---

### 4.2 Ejecución del sistema

El script principal **orquesta todo el flujo de navegación**:
RViz, mapa y nodo de control.

```bash
roscd move/scripts
./run_all.sh
```

Comportamiento esperado:

* RViz se abre con el mapa del aula
* El robot aparece localizado en el mapa
* El nodo `checkpointfollower.py` comienza a recorrer los checkpoints
* Se publican los flags de llegada correspondientes

---

## 5. Documentación detallada por componente

Este README da la visión global del paquete.
Para profundizar en cada parte, consulta:

* **Scripts y ejecución**
  [`scripts/README.md`](./scripts/README.md)

* **Launch files**
  [`launch/README.md`](./launch/README.md)

* **Nodos y código fuente**
  [`src/move/README.md`](./src/move/README.md)

* **Configuración y RViz**
  [`configs/README.md`](./configs/README.md)

---

## 6. Notas de diseño

* La navegación se basa en **checkpoints fijos**, no en planificación dinámica
* Esto reduce incertidumbre y hace la demo **más estable y reproducible**
* El objetivo es **integración de sistema**, no optimización de navegación

---

## 7. Resumen

* `move` implementa la navegación autónoma mínima necesaria para TirGoPharma
* Publica hitos de navegación consumidos por `tirgo_mission_server`
* Está diseñado para ser **simple, fiable y fácil de depurar**
* Puede ejecutarse tanto en robot real como en simulación

Este paquete permite que el robot **se mueva cuando debe moverse**
y que el resto del sistema **sepa exactamente cuándo ha llegado**.

