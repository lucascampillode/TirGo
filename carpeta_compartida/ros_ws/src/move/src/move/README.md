<div align="center">

# Módulo `move` (Python)

Nodos de **navegación y orquestación de movimiento** del robot TIAGo
para el sistema **TirGoPharma**.

Este directorio contiene la lógica que permite al robot desplazarse
entre puntos clave (dispensador ↔ paciente) y publicar los hitos de navegación
utilizados por el coordinador de misión.

</div>

---

## Visión general

La carpeta `src/move/` agrupa los nodos Python responsables de:

- Navegación basada en **checkpoints**
- Orquestación del movimiento dentro del **flujo de misión**
- Publicación de la **pose inicial** para localización
- Pruebas de comunicación y validación

Estos nodos trabajan conjuntamente con:

- El stack de navegación (`move_base`)
- El mapa estático del entorno (`map_server`)
- El coordinador de misión (`tirgo_mission_server`)

---

## 1. Archivos del módulo

```text
src/move/
├── comunication_move.py     # Orquestador del movimiento dentro del flujo de misión
├── checkpointfollower.py    # Lógica base de navegación por checkpoints
├── publish_initial_pose.py  # Publicador de pose inicial (/initialpose)
├── comunicacion_test.py     # Test de conectividad y comunicación (auxiliar)
└── test_puntos.py           # Validación de coordenadas y puntos (auxiliar)
````

---

## 2. Descripción de los nodos

### 2.1 `checkpointfollower.py`

Es el **nodo base de navegación**.
Su responsabilidad es mover al robot a través de una secuencia fija de puntos.

#### Funcionamiento

1. Define una lista de **checkpoints** (coordenadas en el mapa).
2. Envía cada punto como un objetivo al stack de navegación:

   * `/move_base/goal`
3. Espera a que el robot alcance el objetivo antes de continuar.
4. Detecta la llegada al punto antes de avanzar al siguiente.

Este nodo **no toma decisiones de alto nivel**;
simplemente ejecuta movimiento de forma determinista.

#### Mensajes clave

* `move_base_msgs/MoveBaseActionGoal`

---

### 2.2 `comunication_move.py`

Es el **orquestador de movimiento para la demo** y el nodo
que se utiliza habitualmente en el flujo integrado del sistema.

#### Rol dentro del sistema

* Escucha el inicio de la misión (p. ej. `/tirgo/mission/start`)
* Coordina el desplazamiento del robot a los puntos clave del flujo:

  * dispensador
  * paciente
* Publica **hitos de navegación** consumidos por el coordinador de misión:

  * `/tirgo/tiago/arrived`
  * `/tirgo/tiago/at_patient`

Este nodo no implementa navegación de bajo nivel,
sino que **encapsula cuándo y hacia dónde debe moverse el robot**
dentro del proceso completo de dispensación.

> En la demo integrada, este nodo se lanza desde `scripts/run_all.sh`.

#### Flujo simplificado

1. Espera evento de inicio de misión.
2. Navega al dispensador.
3. Publica `/tirgo/tiago/arrived`.
4. Espera confirmaciones del proceso (dispensación / recogida).
5. Navega al paciente.
6. Publica `/tirgo/tiago/at_patient`.

---

### 2.3 `publish_initial_pose.py`

Script auxiliar para facilitar la **localización inicial** del robot.

#### Funcionamiento

* Publica una estimación de la pose inicial del robot en:

  * `/initialpose`
* Permite reiniciar o ajustar la localización de `amcl`
  sin necesidad de usar RViz manualmente.

Es especialmente útil:

* al iniciar la demo
* tras mover el robot manualmente
* en pruebas repetidas

---

### 2.4 `comunicacion_test.py`

Nodo de **pruebas y verificación** de comunicación.

#### Uso principal

* Validar que los topics relevantes están activos
* Comprobar que el robot responde a mensajes de navegación
* Detectar problemas de conexión o configuración

Este nodo **no forma parte del flujo final de producción**,
pero es útil durante el desarrollo y la depuración.

---

### 2.5 `test_puntos.py`

Script de **validación de coordenadas**.

Permite:

* Comprobar que los checkpoints están bien definidos
* Verificar que las posiciones son alcanzables en el mapa
* Evitar errores de navegación por puntos mal configurados

---

## 3. Integración con la misión

Los nodos de este directorio **no gestionan la misión completa**,
pero sí proporcionan los **eventos físicos de movimiento**.

En concreto:

* Publican hitos como:

  * `/tirgo/tiago/arrived`
  * `/tirgo/tiago/at_patient`
* Estos hitos son consumidos por:

  * `tirgo_mission_server`

De este modo, la misión avanza únicamente cuando
el robot **ha llegado físicamente al punto esperado**.

---

## 4. Dependencias

Para ejecutar estos nodos es necesario disponer de:

### ROS

* `rospy`
* `geometry_msgs`
* `move_base_msgs`

### Python

* `numpy`
* `tf.transformations`
  (manejo de cuaterniones y orientación)

---

## 5. Uso típico

Este módulo **no suele lanzarse nodo a nodo manualmente**.

Forma parte del flujo iniciado mediante:

* `scripts/run_all.sh`

Esto garantiza que el mapa, la localización, RViz y la navegación
se inician en el orden correcto.

---

## 6. Resumen

* `src/move/` contiene la **implementación real del movimiento**
* `checkpointfollower.py` ejecuta navegación determinista de bajo nivel
* `comunication_move.py` orquesta el movimiento dentro del flujo de misión
* Los nodos auxiliares facilitan localización y pruebas

Este directorio es donde el sistema
**deja de ser lógico y empieza a moverse de verdad**
