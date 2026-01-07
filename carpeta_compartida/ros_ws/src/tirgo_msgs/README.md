<div align="center">

# tirgo_msgs

Paquete de mensajes y acciones ROS 1 para el ecosistema TirgoPharma.

Define los tipos compartidos entre la interfaz web, el coordinador de misión
y los nodos de ejecución robótica y hardware, garantizando consistencia de
interfaces en todo el sistema.

Compatible con ROS 1 (Noetic) y basado en actionlib.

</div>

---

## Visión general

El paquete `tirgo_msgs` centraliza todas las definiciones de mensajes y acciones
utilizadas por los distintos módulos de TirgoPharma.

Es un paquete puramente declarativo:
no contiene lógica de ejecución, únicamente contratos de comunicación
que deben mantenerse estables entre productores y consumidores.

Actualmente define la acción principal del sistema:

- `TirgoMission.action`: describe la misión completa de dispensación y entrega
  realizada por el robot TIAGo.

Este paquete es utilizado por:

- `tirgo_ui` como Action Client
- `tirgo_mission_server` como Action Server
- Nodos de navegación, manipulación y dispensación que observan el estado de la misión

---

## 1. Estructura del paquete

```text
tirgo_msgs/
├── package.xml
├── CMakeLists.txt
└── action/
    └── TirgoMission.action
````

---

## 2. Acción `TirgoMission.action`

La acción `TirgoMission` modela la misión completa de TirgoPharma:
desde que la interfaz web lanza una solicitud válida hasta que el robot
finaliza la entrega y la interacción con el paciente.

### Definición completa

```action
# ===== GOAL =====

# Identificador del paciente (hash del DNI u otro identificador opaco)
string patient_id

# Identificador físico de cubeta en el dispensador (bin_id)
int32 med_id

---
# ===== RESULT =====

# Éxito global de la misión
bool success

# Código simbólico de error
string error_code

# Mensaje descriptivo de error (logs / UI)
string error_message

---
# ===== FEEDBACK =====

# Estado textual de la misión
string state

# Progreso aproximado en rango [0.0, 1.0]
float32 progress
```

---

## 3. Semántica y contrato operativo

### 3.1 Goal

* `patient_id`
  Identificador opaco del paciente.
  La identidad real nunca se transmite en claro por ROS.

* `med_id`
  Identificador físico de la cubeta (`bin_id`) del dispensador.

  Si el sistema maneja identificadores lógicos de medicación en base de datos,
  la traducción medicamento → `bin_id` debe realizarse en `tirgo_ui`
  antes de enviar el goal al Action Server.

---

### 3.2 Feedback

El campo `state` refleja directamente el estado de la máquina de estados
del coordinador de misión.

Estados esperados (no exhaustivos):

* `GOING_TO_DISPENSER`
* `WAITING_DISPENSE`
* `PICKING_UP`
* `GOING_TO_PATIENT`
* `AT_PATIENT`
* `FAREWELL`
* `DONE`

El campo `progress` representa un progreso aproximado de la misión.
No se garantiza linealidad estricta, pero se recomienda:

* valores crecientes
* coherentes con el avance por fases
* adecuados para visualización en la UI

---

### 3.3 Result

* `success`
  Indica si la misión ha finalizado correctamente.

* `error_code`
  Código simbólico que permite a la UI y a los tests distinguir el motivo del fallo.

  Valores típicos:

  * `BAD_GOAL`
  * `TIMEOUT_ARRIVE`
  * `TIMEOUT_READY`
  * `TIMEOUT_PICK`
  * `TIMEOUT_PATIENT`
  * `TIMEOUT_DELIVER`
  * `TIMEOUT_FAREWELL`
  * `PREEMPTED`

* `error_message`
  Texto descriptivo orientado a logs y depuración.

---

## 4. Integración con otros paquetes

### 4.1 `tirgo_mission_server`

`tirgo_mission_server` implementa el Action Server asociado a `/tirgo/mission`:

* Consume `patient_id` y `med_id` del goal
* Publica feedback (`state`, `progress`) durante la ejecución
* Devuelve el result final de la misión

La FSM del sistema está directamente reflejada en los campos de feedback.

---

### 4.2 `tirgo_ui`

`tirgo_ui` implementa un Action Client:

* Valida la solicitud del usuario
* Traduce datos de negocio (medicación → bin_id)
* Construye y envía un `TirgoMissionGoal`
* Visualiza feedback y resultado en la interfaz web

---

## 5. Verificación en runtime

Comandos útiles para inspeccionar la acción en ejecución:

```bash
rostopic list | grep /tirgo/mission
rostopic echo /tirgo/mission/feedback
rostopic echo /tirgo/mission/result
```

Para inspeccionar la definición compilada:

```bash
rosmsg show tirgo_msgs/TirgoMissionAction
```



---


## 6. Resumen

* `tirgo_msgs` define los contratos de comunicación del sistema TirgoPharma.
* `TirgoMission.action` encapsula la misión completa a nivel de interfaz.
* UI, coordinador y nodos dependen directamente de este paquete.


Este paquete es la base para mantener consistencia de tipos,
trazabilidad de estados y robustez en todo el sistema TirgoPharma.
