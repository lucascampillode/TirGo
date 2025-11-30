# `tirgo_msgs`

Paquete de **mensajes y acciones ROS 1** para el ecosistema **TirgoPharma**.

Aquí se definen los tipos compartidos entre:

- La interfaz web (`tirgo_ui`)
- El Action Server de misión (`tirgo_mission_server`)
- Los nodos de TIAGo y del dispensador Raspberry Pi

Actualmente contiene la acción principal:

- `TirgoMission.action` → describe toda la misión de dispensación y entrega para el robot TIAGo.

Está pensado para **ROS 1 (Noetic)** y se integra con `actionlib`.

---

## 1. Estructura del paquete

```text
tirgo_msgs/
├── package.xml
├── CMakeLists.txt
└── action/
    └── TirgoMission.action      ← definición de la acción /tirgo/mission
````

---

## 2. Acción `TirgoMission.action`

Esta acción modela la misión completa de TirgoPharma: desde que la web lanza un pedido hasta que TIAGo entrega el medicamento y se despide.

Contenido:

```action
# ===== GOAL =====
# Identificador del paciente (hash del DNI)
string patient_id

# Identificador de la cubeta física en el dispensador
int32 med_id

---
# ===== RESULT =====
# Éxito global de la misión
bool success

# Código de error en caso de fallo
string error_code

# Mensaje de error más descriptivo (para logs/UI)
string error_message

---
# ===== FEEDBACK =====
# Estado textual de la misión (para la UI)
string state

# Progreso aproximado 0.0 - 1.0
float32 progress
```

Resumen de cada bloque:

* **Goal**

  * `patient_id`: hash del DNI del paciente. La web nunca pasa el DNI en claro.
  * `med_id`: identificador lógico del medicamento / cubeta del dispensador.

* **Result**

  * `success`: `true` si la misión ha terminado correctamente.
  * `error_code`: código simbólico de error (`TIMEOUT_ARRIVE`, `TIMEOUT_READY`, etc.).
  * `error_message`: texto más descriptivo para logs y para la UI web.

* **Feedback**

  * `state`: estado textual de la máquina de estados (ej. `GOING_TO_DISPENSER`).
  * `progress`: número en `[0.0, 1.0]` que aproxima el progreso total de la misión.

---

## 3. Integración con otros paquetes

### 3.1 `tirgo_mission_server`

El paquete `tirgo_mission_server` implementa el **Action Server** `/tirgo/mission`:

* Usa `TirgoMissionGoal.patient_id` y `med_id` para iniciar la misión.
* Va publicando `TirgoMissionFeedback.state` y `progress`.
* Rellena `TirgoMissionResult.success`, `error_code` y `error_message` al terminar.

### 3.2 `tirgo_ui`

El paquete `tirgo_ui` implementa un **Action Client** en `rosio.py`:


* Cuando el usuario confirma un pedido, construye un `TirgoMissionGoal`.
* Envía el goal al Action Server `/tirgo/mission`.
* Muestra en la web el `feedback.state`, `progress` y el `result` final.

---

## 4. Reconstrucción tras cambios en la acción

Cada vez que se cambie el contenido de `TirgoMission.action`, es recomendable limpiar y recompilar:

```bash
cd ~/carpeta_compartida/ros_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

Después:

```bash
rosmsg show tirgo_msgs/TirgoMissionAction
```

para verificar que el sistema está viendo la versión actualizada.

---

## 5. Resumen

* `tirgo_msgs` centraliza los tipos que comparten la web, el Action Server y los nodos de hardware.
* `TirgoMission.action` encapsula toda la lógica de alto nivel de una misión de TirgoPharma.
* `tirgo_mission_server` y `tirgo_ui` dependen directamente de este paquete, por lo que cualquier cambio en la acción debe ir acompañado de recompilación y tests.

Este paquete es la base para mantener **consistencia de tipos** en todo el sistema TirgoPharma.
