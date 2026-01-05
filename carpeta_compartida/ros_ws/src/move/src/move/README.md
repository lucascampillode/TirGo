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
- Publicación de la **pose inicial** para localización
- Pruebas de comunicación y validación
- Coordinación del movimiento dentro del flujo de misión

Estos nodos trabajan conjuntamente con:

- El stack de navegación (`move_base`)
- El mapa estático del entorno
- El coordinador de misión (`tirgo_mission_server`)

---

## 1. Archivos del módulo

```text
src/move/
├── checkpointfollower.py      # Lógica principal de navegación por checkpoints
├── publish_initial_pose.py    # Publicador de pose inicial (/initialpose)
├── comunicacion_test.py       # Test de conectividad y comunicación
└── test_puntos.py             # Validación de coordenadas y puntos
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
4. Notifica cuando un punto ha sido alcanzado.

Este nodo **no toma decisiones de alto nivel**;
simplemente ejecuta movimiento de forma determinista.

#### Mensajes clave

* `move_base_msgs/MoveBaseActionGoal`

---

### 2.2 `publish_initial_pose.py`

Script auxiliar para facilitar la **localización inicial** del robot.

#### Funcionamiento

* Publica una estimación de la pose inicial del robot en:

  * `/initialpose`
* Permite reiniciar o ajustar la localización de `amcl`
  sin necesidad de usar RViz manualmente.

Es especialmente útil:

* Al iniciar la demo
* Tras mover el robot manualmente
* En pruebas repetidas

---

### 2.3 `comunicacion_test.py`

Nodo de **pruebas y verificación** de comunicación.

#### Uso principal

* Validar que los topics relevantes están activos
* Comprobar que el robot responde a mensajes de navegación
* Detectar problemas de conexión o configuración

Este nodo **no forma parte del flujo final de producción**,
pero es clave durante el desarrollo.

---

### 2.4 `test_puntos.py`

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

* Publican flags como:

  * `/tirgo/tiago/arrived`
  * `/tirgo/tiago/at_patient`
* Estos flags son consumidos por:

  * `tirgo_mission_server`

De este modo, la misión avanza solo cuando
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

Este módulo **no suele lanzarse directamente** nodo a nodo.

Forma parte del flujo iniciado mediante:

* `scripts/run_all.sh`
* o los launch files del paquete `move`

Esto garantiza que el mapa, RViz y la navegación
se inician en el orden correcto.

---

## 6. Resumen

* `src/move/` contiene la **implementación real del movimiento**
* `checkpointfollower.py` ejecuta navegación determinista
* Los nodos auxiliares facilitan localización y pruebas
* El módulo publica eventos que sincronizan la misión completa

Este directorio es el punto donde el sistema
**deja de ser lógico y empieza a moverse de verdad** 🤖🚶‍♂️
