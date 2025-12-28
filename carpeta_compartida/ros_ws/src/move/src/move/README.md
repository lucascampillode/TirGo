## Carpeta Move 
````markdown

# Módulo `move` (Python) – Nodos de Navegación

Este directorio contiene el código fuente en Python que implementa la lógica de navegación, secuenciación de objetivos y pruebas del robot TIAGo.

---

## 1. Archivos del Paquete

```text
src/move/
├── checkpointfollower.py   # Nodo principal de navegación
├── publish_initial_pose.py # Herramienta de localización
├── comunicacion_test.py    # Test de conectividad
└── test_puntos.py          # Validación de coordenadas
````

-----

## 2\. Descripción de los Nodos

### 2.1. `checkpointfollower.py`

Es el **nodo principal** ("Director de Misión"). Su función es guiar al robot a través de una ruta preestablecida.

  * **Funcionamiento:**
    1.  Lee una lista de coordenadas (checkpoints) definida en el código.
    2.  Envía cada punto como un objetivo (`goal`) al topic `/move_base/goal`.
    3.  Monitoriza el estado del robot y espera a que llegue al destino antes de enviar el siguiente punto.
  * **Mensajes clave:** `move_base_msgs/MoveBaseActionGoal`.

### 2.2. `publish_initial_pose.py`

Un script auxiliar para ayudar al sistema de localización (`amcl`) a situarse.

  * **Funcionamiento:** Publica una estimación de la posición inicial del robot en el topic `/initialpose`. Esto es útil para "resetear" la ubicación del robot en el mapa al inicio de la ejecución sin usar la interfaz gráfica de RViz.


### 2.3. `communication_move.py`

Es el **coordinador de la misión** y actúa como una máquina de estados asíncrona. Gestiona todo el ciclo de vida del proceso de dispensación, desde que se recibe la orden hasta que el medicamento se entrega al paciente.

* **Funcionamiento:**
1. **Espera de misión:** Permanece en estado `IDLE` hasta recibir un mensaje en `/tirgo/mission/start`.
2. **Orquestación de navegación:** Utiliza la clase `Follower` (de `checkpointfollower.py`) para mover al robot entre dos puntos clave: el **paciente** y el **dispensador**.
3. **Gestión de eventos (Callbacks):** El nodo no solo mueve al robot, sino que escucha eventos externos de hardware y lógica para avanzar de fase (ej. cuando el dispensador está listo o el brazo ha recogido el envase).


* **Fases del Proceso:**
* `MOVING_TO_DISPENSER`: Trayecto hacia el módulo de carga.
* `DISPENSING`: Espera a que el sistema mecánico libere el medicamento.
* `PICKING_CONTAINER`: Fase de recogida por parte del robot.
* `RETURNING_TO_PATIENT`: Trayecto de vuelta al origen.
* `DELIVERING_TO_PATIENT` / `CLOSING_INTERACTION`: Entrega final y despedida.


* **Topics Clave:**
* **Subscripciones:** `/tirgo/mission/start`, `/tirgo/dispense/ready`, `/tirgo/tiago/picked`, `/tirgo/tiago/delivered`.
* **Publicaciones:** `/tirgo/tiago/arrived` (llegada al dispensador), `/tirgo/tiago/at_patient` (llegada al paciente).


* **Uso:**
Para iniciar una misión manualmente desde la terminal una vez el nodo esté activo:
```bash
rostopic pub /tirgo/mission/start std_msgs/String "data: 'go'" -1

```

-----

## 3\. Dependencias

Para que estos scripts funcionen, el entorno debe tener acceso a las siguientes librerías de ROS y Python:

  * `rospy`
  * `geometry_msgs`
  * `move_base_msgs`
  * `tf.transformations` (para el manejo de cuaterniones y orientación)
  * `numpy`

<!-- end list -->

```

