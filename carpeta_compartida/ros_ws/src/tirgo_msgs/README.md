<div align="center">

# tirgo_msgs

Paquete de **mensajes y acciones ROS 1** para el ecosistema **TirgoPharma**.

Define los **tipos compartidos** entre la interfaz web, el coordinador de misión
y los nodos de ejecución robótica y hardware, garantizando **consistencia de
interfaces** en todo el sistema.

Compatible con **ROS 1 (Noetic)** y basado en **actionlib**.

</div>

---

## Visión general

El paquete **`tirgo_msgs`** centraliza todas las **definiciones de mensajes y acciones**
utilizadas por los distintos módulos de TirgoPharma.

Es un paquete **puramente declarativo**:  
no contiene lógica de ejecución, solo contratos de comunicación claros y estables.

Actualmente define la acción principal del sistema:

- **`TirgoMission.action`** → describe la misión completa de dispensación y entrega
  realizada por el robot TIAGo.

Este paquete es utilizado por:

- La **interfaz web** (`tirgo_ui`) como *Action Client*
- El **coordinador de misión** (`tirgo_mission_server`) como *Action Server*
- Los nodos de **navegación, manipulación y dispensación**

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

La acción **`TirgoMission`** modela la misión completa de TirgoPharma:
desde que la web lanza una solicitud válida hasta que TIAGo entrega
el medicamento y finaliza la interacción.

### Definición completa

```action
# ===== GOAL =====
# Identificador del paciente (hash del DNI)
string patient_id

# Identificador lógico del medicamento / cubeta del dispensador
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

---

### Significado de cada bloque

#### Goal

* `patient_id`
  Hash del DNI del paciente.
  La identidad nunca se transmite en claro por ROS.

* `med_id`
  Identificador lógico del medicamento, que corresponde
  con la cubeta física (`bin_id`) del dispensador.

#### Result

* `success`
  `true` si la misión ha finalizado correctamente.

* `error_code`
  Código simbólico de error (`TIMEOUT_ARRIVE`, `TIMEOUT_READY`, etc.).

* `error_message`
  Mensaje descriptivo para logs y visualización en la UI web.

#### Feedback

* `state`
  Estado textual de la máquina de estados del coordinador
  (por ejemplo: `GOING_TO_DISPENSER`, `WAITING_DISPENSE`, etc.).

* `progress`
  Valor en el rango `[0.0, 1.0]` que aproxima el progreso total
  de la misión para la interfaz web.

---

## 3. Integración con otros paquetes

### 3.1 `tirgo_mission_server`

El paquete **`tirgo_mission_server`** implementa el **Action Server**
asociado a `/tirgo/mission`:

* Recibe `patient_id` y `med_id` en el `Goal`
* Publica `state` y `progress` como `Feedback`
* Devuelve `success`, `error_code` y `error_message` en el `Result`

La **máquina de estados** del sistema está directamente reflejada
en los campos de `Feedback`.

---

### 3.2 `tirgo_ui`

El paquete **`tirgo_ui`** implementa un **Action Client** (en `rosio.py`):

* Construye un `TirgoMissionGoal` tras validar la solicitud del usuario
* Envía el goal al Action Server `/tirgo/mission`
* Muestra en la web el `feedback.state`, `feedback.progress`
  y el `result` final de la misión

---

## 4. Reconstrucción tras cambios en la acción

Cada vez que se modifique el archivo `TirgoMission.action`,
es necesario **recompilar el workspace** para regenerar los mensajes:

```bash
cd ~/carpeta_compartida/ros_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

Para verificar que la definición activa es la correcta:

```bash
rosmsg show tirgo_msgs/TirgoMissionAction
```

---

## 5. Resumen

* `tirgo_msgs` centraliza los **contratos de comunicación** del sistema TirgoPharma.
* `TirgoMission.action` encapsula toda la lógica de alto nivel de una misión.
* La web, el coordinador y los nodos de ejecución dependen directamente de este paquete.
* Cualquier cambio en la acción requiere recompilación y validación mediante tests.

Este paquete es la base para mantener **consistencia de tipos, trazabilidad
y robustez** en todo el sistema TirgoPharma.
