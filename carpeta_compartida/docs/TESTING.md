<div align="center">

# Testing — TirGoPharma (ROS 1 Noetic)
Guía **definitiva** de pruebas del repositorio: **qué cubre cada suite**, **cómo ejecutarla** (Docker o nativo) y **qué lanzar según el cambio**.

</div>

---

## Principios (qué se prueba y qué NO)

- El foco de los tests es **proteger la demo end-to-end**: contratos entre módulos, validaciones biomédicas y coordinación de misión.
- Hay tres niveles:
  1) **Unit (pytest)**: rápido, sin ROS master (se mockea `rospy/actionlib` cuando hace falta).
  2) **Integración ROS (rostest)**: levanta nodos reales y verifica flujos.
  3) **Manual/smoke**: scripts simples para navegación (`move`) que no están integrados como suite CI.

> Nota: dentro de `carpeta_compartida/gallium/` hay ficheros de terceros (dependencias/overlay). **No forman parte** del plan de tests del proyecto.

---

## Mapa global de tests (por paquete)

| Área | Paquete | Dónde | Tipo | Requiere ROS master | Requiere HW |
|---|---|---|---|:---:|:---:|
| Web UI (Flask + negocio) | `tirgo_ui` | `tirgo_ui/tests/` | `pytest` | ❌ | ❌ |
| STT (Vosk) | `stt_vosk` | `stt_vosk/tests/` | `pytest` | ❌ | ❌ |
| Mission Server (unit) | `tirgo_mission_server` | `tirgo_mission_server/test/test_tirgo_mission.py` | `pytest` | ❌ | ❌ |
| Mission Server (E2E ROS) | `tirgo_mission_server` | `tirgo_mission_server/test/test_mission_flow.test` | `rostest` | ✅ | ❌ |
| Dispensador (unit sin HW) | `servo_dispenser` | `servo_dispenser/test/test_servo_dispenser.py` | `pytest` | ❌ | ❌ |
| Dispensador (ROS real) | `servo_dispenser` | `servo_dispenser/test/servo_dispenser_flow.test` | `rostest` | ✅ | ⚠️ *(recomendado RPi / pigpio)* |
| Navegación (manual) | `move` | `move/src/move/*test*.py` | scripts | ✅ | depende |

---

## Diagrama: cobertura por capa

```mermaid
---
config:
  layout: elk
---
flowchart TB
 subgraph Unit["UNIT<br>(rápido, sin ROS master)"]
        UI["tirgo_ui (pytest)<br>- rutas/flujo UI<br>- validaciones DNI/nombre<br>- storage Mongo (fakes)<br>- puente ROS (mocks)"]
        STT["stt_vosk (pytest)<br>- hotword<br>- parciales<br>- modelo ausente (error controlado)"]
        MSU["tirgo_mission_server (pytest)<br>- flags/callbacks<br>- _wait_flag (success/timeout/preempt)<br>- validación de goal (BAD_GOAL)"]
        SDU["servo_dispenser (pytest)<br>- mapeo bin_id -&gt; GPIO<br>- clamp PWM (MIN/MAX)<br>- inválidos no publican ready<br>- stub pigpio"]
  end
 subgraph Integration["INTEGRACIÓN<br>(ROS real con rostest)"]
        MSI["tirgo_mission_server (rostest)<br>- ActionServer /tirgo/mission<br>- happy path por hitos<br>- timeouts por fase<br>- cancelación (preempt)"]
        SDI["servo_dispenser (rostest)<br>- nodo real<br>- /request -&gt; /ready<br>- latencias/tempos reales"]
  end
 subgraph Manual["MANUAL / SMOKE (no CI)"]
        MV["move scripts<br>- comunicacion_test.py<br>- test_puntos.py"]
  end
````

---

## Dónde ejecutar los tests (Docker vs nativo)

### Opción recomendada: ejecutar tests dentro del contenedor ROS (PC)

Es lo más reproducible porque ya tienes ROS Noetic y dependencias cargadas.

1. Levanta el entorno (stack normal):

```bash
./tirgo_ALL.sh
```

2. Entra al contenedor:

```bash
docker exec -it "$(docker compose ps -q ros1_rob_tirgo)" bash
```

3. Dentro del contenedor, carga entorno ROS y workspace:

```bash
source /opt/ros/noetic/setup.bash
source ~/carpeta_compartida/ros_ws/devel/setup.bash
```

> Si estás trabajando sin `devel/` aún, compila primero dentro del contenedor:

```bash
cd ~/carpeta_compartida/ros_ws && catkin_make
source devel/setup.bash
```

### Opción nativa (Ubuntu 20.04 + ROS Noetic)

Igual que arriba, pero sin `docker exec`. Necesitas ROS Noetic instalado y el workspace compilado.

---

## Ejecutar tests (comandos oficiales)

> Todos los paths de esta guía asumen el workspace montado en:
> `~/carpeta_compartida/ros_ws/src/`

### 1) Pytest (rápido)

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_ui && pytest -q
cd ../stt_vosk && pytest -q
cd ../tirgo_mission_server && pytest -q
cd ../servo_dispenser && pytest -q
```

**Qué hace realmente**:

* `tirgo_ui`: tests de rutas/validaciones + storage Mongo con fakes + mocks del puente ROS.
* `stt_vosk`: tests con dobles (audio/vosk/rospy) para validar publicaciones y errores.
* `tirgo_mission_server`: **solo unit** (el flow ROS se excluye por `collect_ignore`).
* `servo_dispenser`: unit sin HW, con stub de `pigpio`.

### 2) Rostest (integración ROS)

#### Mission Server (ActionServer `/tirgo/mission`)

```bash
rostest tirgo_mission_server test_mission_flow.test
```

* Lanza `tirgo_mission_server.py` con timeouts pequeños (params en el `.test`).
* Ejecuta `test_mission_flow.py` como test ROS real.

#### Dispensador (flujo ROS nodo real)

```bash
rostest servo_dispenser servo_dispenser_flow.test
```

⚠️ Recomendado en Raspberry Pi (o entorno con `pigpio` disponible).
En RPi, asegúrate de tener el demonio:

```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
# o:
sudo pigpiod
```

---

## Selección de tests por impacto del cambio (guía práctica)

Objetivo: ejecutar **la mínima batería** que aporta confianza real según lo tocado.

### Cambios en UI (rutas, validaciones, sesión)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_ui && pytest -q
```

Cubre:

* navegación con/sin sesión,
* validaciones DNI/nombre,
* mensajes de error/redirecciones,
* flujo “consultar / leer / diagnóstico”.

---

### Cambios en acceso a datos (Mongo storage)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_ui && \
pytest -q tests/test_storage_mongo_unit.py tests/test_storage_stock_dispenses.py
```

Cubre:

* queries/filtros,
* decremento de stock sin negativos,
* logging de dispensación OK/ERROR (estructura esperada).

---

### Cambios en puente Web → ROS (rosio / start_mission_async)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_ui && \
pytest -q tests/test_web_ros_integration.py tests/test_rosio_helpers_unit.py
```

Cubre:

* que se construye la llamada a misión con `med_id/bin_id` coherente,
* manejo de error sin tumbar la UI,
* helpers y contrato mínimo del puente (mock).

---

### Cambios en STT (publicación, hotword, parciales, modelo)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/stt_vosk && pytest -q
```

Cubre:

* hotword (emisión y control),
* parciales (cuando procede),
* fallo controlado si falta el modelo.

---

### Cambios internos en Mission Server (flags, timeouts, validación goal)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/tirgo_mission_server && pytest -q
```

Cubre:

* callbacks → flags correctos,
* `_wait_flag` (success/timeout/preempt),
* goals inválidos (BAD_GOAL).

---

### Cambios en tópicos/estados del flujo E2E de misión (lo más crítico)

Ejecuta:

```bash
rostest tirgo_mission_server test_mission_flow.test
```

Cubre:

* ActionServer real,
* avance por fases/hitos,
* timeouts por fase,
* cancelación/preempt.

> Importante: `test_mission_flow.py` **no se ejecuta con pytest** (es `rostest`).

---

### Cambios en lógica del dispensador (mapping bin_id, PWM, validaciones)

Ejecuta:

```bash
cd ~/carpeta_compartida/ros_ws/src/servo_dispenser && pytest -q
```

Cubre:

* clamp PWM (`MIN_US`, `MAX_US`, `NEUTRAL`),
* bin válido → “mueve servo” (captura de llamadas pigpio fake),
* bin inválido → no confirma ready.

---

### Cambios en el flujo ROS del dispensador (`/request` → `/ready`)

Ejecuta (ideal en RPi):

```bash
sudo pigpiod
rostest servo_dispenser servo_dispenser_flow.test
```

Cubre:

* bin válido → ready,
* bin inválido → nunca ready (según test).

---

### Cambios en navegación / checkpoints (`move`) — validación manual

Ejecuta:

```bash
rosrun move comunicacion_test.py
rosrun move test_puntos.py
```

Objetivo:

* smoke test rápido de comunicación y ruta por puntos.
* No es suite automatizada CI (aún).

---

## Inventario exacto de tests (ficheros reales en el repo)

### `tirgo_ui/tests/`

* `test_main_routes.py` — smoke de index con/sin sesión (plantillas fake)
* `test_consultar.py` — validación DNI/nombre y flujo consultar
* `test_leer.py` — selección medicación + checks de stock (plantillas fake)
* `test_diagnostico.py` — diagnóstico/estado + logout/cierre
* `test_session_unit.py` — helpers de sesión (start/end/current)
* `test_storage_mongo_unit.py` — acceso a datos con fakes (pacientes/recetas/meds)
* `test_storage_stock_dispenses.py` — stock + logging de dispensación
* `test_rosio_helpers_unit.py` — helpers del puente ROS (contrato básico)
* `test_web_ros_integration.py` — web → misión (mock de start_mission_async)
* `conftest.py` — resumen de resultados al final (output amigable)

### `stt_vosk/tests/`

* `test_stt_vosk_node.py` — hotword/partials/model missing con dobles de audio/vosk/rospy
* `conftest.py`

### `tirgo_mission_server/test/`

* `test_tirgo_mission.py` — unit sin ROS real (mock de rospy/actionlib/std_msgs/tirgo_msgs)
* `test_mission_flow.test` — launch de integración (levanta server con timeouts pequeños)
* `test_mission_flow.py` — test ROS (rostest + unittest)
* `conftest.py` — `collect_ignore = ["test_mission_flow.py"]` para que pytest no lo ejecute

### `servo_dispenser/test/`

* `test_servo_dispenser.py` — unit sin HW (stub de pigpio + asserts de publicaciones y mapping)
* `servo_dispenser_flow.test` — launch de integración (nodo real + test)
* `test_servo_dispenser_flow.py` — test ROS (rostest)
* `conftest.py`

### `move/src/move/` (manual)

* `comunicacion_test.py`
* `test_puntos.py`

---

## Troubleshooting de testing (lo típico)

* **ImportError de paquetes ROS**: asegúrate de haber hecho `catkin_make` y de haber `source devel/setup.bash`.
* **Rostest no encuentra nodos**: revisa que el paquete está en el workspace correcto (`~/carpeta_compartida/ros_ws/src`) y recompila.
* **servo_dispenser_flow falla en PC**: es normal si no tienes `pigpio`/acceso HW. Ejecútalo en RPi o en un entorno que lo soporte.
* **pytest “pasa” pero demo falla**: ejecuta `rostest tirgo_mission_server test_mission_flow.test`
