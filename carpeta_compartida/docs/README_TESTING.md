<div align="center">

# Testing — TirGoPharma (ROS 1 Noetic)
Guía de pruebas del repositorio: **qué cubre cada suite**, **cómo ejecutarla** y **qué ejecutar según el cambio**.

</div>

---

## Mapa global de tests

| Área | Dónde | Tipo | Requiere ROS master | Requiere HW |
|---|---|---|:---:|:---:|
| Web UI | `tirgo_ui/tests/` | pytest | ❌ | ❌ |
| STT (Vosk) | `stt_vosk/tests/` | pytest | ❌ | ❌ |
| Mission Server | `tirgo_mission_server/test/` | pytest + rostest | ✅ *(solo rostest)* | ❌ |
| Dispensador | `servo_dispenser/test/` | pytest + rostest | ✅ *(solo rostest)* | ⚠️ *(flow en RPi)* |
| Move (manual) | `move/src/move/*test*.py` | scripts | ✅ | depende |

### Diagrama: cobertura por capa

```mermaid
flowchart TB
  subgraph Unit["UNIT (rápido, sin ROS real)"]
    UI["tirgo_ui (pytest)\n- rutas y sesión\n- storage Mongo\n- helpers rosio"]
    STT["stt_vosk (pytest)\n- hotword\n- partials\n- errores de modelo"]
    MSU["tirgo_mission_server (pytest)\n- flags + _wait_flag\n- validación de goal"]
    SDU["servo_dispenser (pytest)\n- mapeo bin_id\n- límites PWM\n- inválidos"]
  end

  subgraph Integration["INTEGRACIÓN (ROS real)"]
    MSI["tirgo_mission_server (rostest)\n- ActionServer /tirgo/mission\n- estados + timeouts + preempt"]
    SDI["servo_dispenser (rostest)\n- /request -> /ready con nodo real"]
  end

  subgraph Manual["VALIDACIÓN MANUAL"]
    MV["move scripts\n- smoke tests\n- ruta por puntos"]
  end
````

---

## Setup (una vez por terminal)

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/ros_ws/devel/setup.bash
```

---

## Ejecutar tests

### Pytest (rápido)

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_ui && pytest -q
cd ../stt_vosk && pytest -q
cd ../tirgo_mission_server && pytest -q
cd ../servo_dispenser && pytest -q
```

### Rostest (integración ROS)

```bash
rostest tirgo_mission_server test_mission_flow.test
rostest servo_dispenser servo_dispenser_flow.test
```

> ⚠️ `servo_dispenser_flow` está pensado para Raspberry Pi (nodo real + pigpio):

```bash
sudo pigpiod
rostest servo_dispenser servo_dispenser_flow.test
```

---

## Selección de tests por impacto del cambio

> Objetivo: ejecutar **la mínima batería** que te da confianza real según lo que has tocado.

### Cambios en UI (rutas, validaciones, sesión)

Ejecuta:

```bash
cd .../tirgo_ui && pytest -q
```

**Por qué existen estos tests:** asegurar que la UI no rompe en cambios de plantilla/rutas y que las validaciones (DNI/nombre/stock) siguen siendo consistentes.
**Qué validan (a alto nivel):**

* navegación sin sesión / con sesión
* validación de input (DNI/nombre)
* mensajes de error y redirecciones correctas

---

### Cambios en acceso a datos (Mongo storage: meds/stock/dispenses/pacientes)

Ejecuta:

```bash
cd .../tirgo_ui && pytest -q tests/test_storage_mongo_unit.py tests/test_storage_stock_dispenses.py
```

**Por qué existen:** evitar “bugs silenciosos” de stock/recetas/registro de dispensación al tocar queries o estructura de documentos.
**Qué validan:**

* lookup y filtros
* decremento de stock sin negativos
* logging de dispensación OK/ERROR

---

### Cambios en el puente web → misión (ROSIO / start_mission_async)

Ejecuta:

```bash
cd .../tirgo_ui && pytest -q tests/test_web_ros_integration.py tests/test_rosio_helpers_unit.py
```

**Por qué existen:** proteger el punto más crítico del proyecto: que desde la UI se dispare correctamente la misión (sin depender de ROS real).
**Qué validan:**

* que se llama a `start_mission_async` con bin_id correcto
* que errores en esa llamada se manejan sin tumbar la UI

---

### Cambios en STT (hotword, parciales, control del bucle, modelo)

Ejecuta:

```bash
cd .../stt_vosk && pytest -q
```

**Por qué existen:** el STT es muy sensible a “pequeños cambios” (publicar de más, no publicar, romper con modelo inexistente).
**Qué validan:**

* hotword se emite una vez
* parciales solo cuando se habilitan
* si falta el modelo → falla de forma controlada

---

### Cambios en lógica del Mission Server (flags, timeouts, validación del goal)

Ejecuta:

```bash
cd .../tirgo_mission_server && pytest -q
```

**Por qué existen:** asegurar la máquina de estados interna sin levantar ROS: callbacks y control de timeouts/preempt.
**Qué validan:**

* callbacks ponen flags correctos
* `_wait_flag` (success/timeout/preempt)
* BAD_GOAL (patient_id vacío o med_id inválido)

---

### Cambios en tópicos/estados del flujo E2E de misión

Ejecuta (imprescindible):

```bash
rostest tirgo_mission_server test_mission_flow.test
```

**Por qué existe:** es el guardarraíl del proyecto: verifica que el ActionServer `/tirgo/mission` funciona de verdad en ROS y que la coreografía de tópicos sigue viva.
**Qué valida:**

* happy path completo
* timeouts por fase (arrive/ready/pick/patient/deliver/farewell)
* cancelación (preempt)

> Nota: `test_mission_flow.py` NO va con pytest; se ejecuta con `rostest`.

---

### Cambios en lógica del dispensador (mapeo bin_id, PWM, validaciones)

Ejecuta:

```bash
cd .../servo_dispenser && pytest -q
```

**Por qué existen:** proteger la lógica sin depender de hardware (stub de pigpio), evitando que un refactor rompa el mapeo de bins.
**Qué validan:**

* límites de PWM (clamp)
* bin válido mueve servo correcto y publica ready
* bin inválido no publica ready

---

### Cambios en el flujo ROS del dispensador (`/request` → `/ready`)

Ejecuta (en RPi con pigpiod):

```bash
sudo pigpiod
rostest servo_dispenser servo_dispenser_flow.test
```

**Por qué existe:** validar integración real en ROS con el nodo ejecutándose como en demo.
**Qué valida:**

* bin válido → ready una vez
* bin inválido → nunca ready

---

### Cambios en navegación / checkpoints / comunicación de `move` (validación manual)

Ejecuta:

```bash
rosrun move comunicacion_test.py
rosrun move test_puntos.py
```

**Por qué existen:** smoke tests para comprobar rápido que el pipeline de navegación y puntos responde (sin convertirlo aún en suite CI).
**Qué validan:**

* ruta por puntos (Follower)
* coreografía de comunicación y estados en tiempo real

---

## Inventario de tests (todos los ficheros)

### `tirgo_ui/tests/`

* `test_main_routes.py` — smoke de index con/sin sesión
* `test_consultar.py` — validación DNI/nombre y flujo consultar
* `test_leer.py` — selección medicación + checks de stock
* `test_diagnostico.py` — diagnóstico/estado + cierre de sesión
* `test_session_unit.py` — start/end/current session
* `test_storage_mongo_unit.py` — acceso a datos (unit)
* `test_storage_stock_dispenses.py` — stock + logging de dispensación
* `test_rosio_helpers_unit.py` — helpers rosio (contrato básico)
* `test_web_ros_integration.py` — web→misión (mock)
* `conftest.py` — resumen de pytest

### `stt_vosk/tests/`

* `test_stt_vosk_node.py` — hotword/partials/model missing
* `conftest.py`

### `tirgo_mission_server/test/`

* `test_tirgo_mission.py` — unit sin ROS real (flags/timeout/preempt/BAD_GOAL)
* `test_mission_flow.test` + `test_mission_flow.py` — integración ROS (ActionServer)
* `conftest.py` — ignora `test_mission_flow.py` en pytest

### `servo_dispenser/test/`

* `test_servo_dispenser.py` — unit sin HW (stub pigpio)
* `servo_dispenser_flow.test` + `test_servo_dispenser_flow.py` — integración ROS (nodo real)
* `conftest.py`

### `move/src/move/` (manual)

* `comunicacion_test.py`
* `test_puntos.py`
