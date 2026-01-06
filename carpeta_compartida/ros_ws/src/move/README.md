<div align="center">

# Módulo `move` (Python)

Nodos de **navegación y orquestación de movimiento** del robot TIAGo
para el sistema **TirGoPharma**.

Este directorio contiene la lógica que permite al robot desplazarse
entre puntos clave (dispensador ↔ paciente) y publicar hitos de navegación
utilizados por el coordinador de misión.

</div>

---

## Visión general

La carpeta `src/move/` agrupa los nodos Python responsables de:

- Orquestación del movimiento dentro del flujo de misión (demo)
- Envío de objetivos al stack de navegación
- Publicación de pose inicial para localización
- Pruebas auxiliares y validación

Estos nodos trabajan conjuntamente con:

- El stack de navegación (p. ej. `move_base`)
- El mapa estático del entorno (`map_server`)
- El coordinador de misión (`tirgo_mission_server`)

---

## 1. Archivos del módulo

```text
src/move/
├── comunication_move.py     # Orquestador: movimiento dispensador↔paciente + hitos ROS
├── checkpointfollower.py    # Utilidad navegación por goals/checkpoints
├── publish_initial_pose.py  # Publicador de pose inicial (/initialpose)
├── comunicacion_test.py     # Test de conectividad y comunicación (auxiliar)
└── test_puntos.py           # Validación de coordenadas y puntos (auxiliar)
````

---

## 2. Descripción de los nodos

### 2.1 `comunication_move.py`

Es el **orquestador de movimiento para la demo**.

#### Rol

* Escucha el inicio de la misión (p. ej. `/tirgo/mission/start`)
* Ejecuta desplazamientos a puntos clave del flujo (dispensador y paciente)
* Publica hitos consumidos por el coordinador de misión:

  * `/tirgo/tiago/arrived`
  * `/tirgo/tiago/at_patient`

> Este nodo es el que normalmente se lanza en el flujo integrado mediante `scripts/run_all.sh`.

---

### 2.2 `checkpointfollower.py`

Es una **utilidad de navegación** orientada a mover el robot por una secuencia de puntos.

#### Funcionamiento (alto nivel)

* Construye objetivos de navegación
* Publica goals para que el stack de navegación ejecute el movimiento
* Permite recorridos deterministas por checkpoints

Este nodo se usa como base/soporte de navegación por puntos (según el escenario de uso).

---

### 2.3 `publish_initial_pose.py`

Script auxiliar para facilitar la **localización inicial** del robot.

* Publica una estimación de pose inicial del robot en `/initialpose`
* Permite reiniciar o ajustar la localización sin necesidad de intervenir desde RViz

Útil:

* al iniciar la demo
* tras mover el robot manualmente
* en pruebas repetidas

---

### 2.4 `comunicacion_test.py`

Nodo de **pruebas** (auxiliar).

Uso típico:

* validar conectividad y que topics del sistema están activos
* detectar problemas de configuración durante desarrollo

No es el flujo principal de demo, pero ayuda a depurar.

---

### 2.5 `test_puntos.py`

Script de validación de coordenadas (auxiliar).

Permite:

* comprobar que las posiciones de referencia son coherentes
* evitar errores por puntos mal definidos en el entorno

---

## 3. Integración con la misión

Los nodos de este directorio no gestionan la misión completa,
pero sí proporcionan los **eventos físicos de movimiento**.

En particular:

* Publican hitos como:

  * `/tirgo/tiago/arrived`
  * `/tirgo/tiago/at_patient`
* Consumidos por:

  * `tirgo_mission_server`

Así la misión avanza solo cuando el robot ha llegado físicamente al punto esperado.

---

## 4. Dependencias

### ROS

* `rospy`
* `geometry_msgs`
* `move_base_msgs`

### Python

* `numpy`
* `tf.transformations`

---

## 5. Uso típico

Normalmente no se lanzan los nodos “a mano” uno a uno.
El flujo estándar está orquestado mediante:

* `scripts/run_all.sh`

Esto garantiza que RViz, mapa y movimiento se inician en orden.

---

## 6. Resumen

* `src/move/` contiene la **implementación real del movimiento**
* `comunication_move.py` orquesta el movimiento en la demo y publica hitos
* Nodos auxiliares facilitan localización y pruebas

Este directorio es donde el sistema **deja de ser lógico y empieza a moverse de verdad**
