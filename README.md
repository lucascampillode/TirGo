<div align="center">
  <img src="carpeta_compartida/ros_ws/src/tirgo_ui/static/logo.png" alt="TirGoPharma Logo" width="220"/>

  # TirGoPharma

  **Dispensación y entrega de medicación con TIAGo** usando **ROS 1 (Noetic)**, **Web (Flask)**, **MongoDB** y un **dispensador físico por servos (Raspberry Pi 3B)**.  
  **Demo end-to-end reproducible** (infra + datos semilla + scripts de arranque).

  <p>
    <a href="http://wiki.ros.org/noetic"><img alt="ROS Noetic" src="https://img.shields.io/badge/ROS-Noetic-22314E?style=flat&logo=ros"></a>
    <a href="https://releases.ubuntu.com/20.04/"><img alt="Ubuntu 20.04" src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=flat&logo=ubuntu&logoColor=white"></a>
    <a href="https://www.python.org/"><img alt="Python 3.8+" src="https://img.shields.io/badge/Python-3.8%2B-3776AB?style=flat&logo=python"></a>
    <a href="https://www.docker.com/"><img alt="Docker Compose" src="https://img.shields.io/badge/Docker-Compose-2496ED?style=flat&logo=docker"></a>
    <a href="https://www.mongodb.com/"><img alt="MongoDB" src="https://img.shields.io/badge/MongoDB-Database-47A248?style=flat&logo=mongodb"></a>
    <img alt="Status" src="https://img.shields.io/badge/Status-Released-blue?style=flat">

  </p>

  <p>
    Proyecto desarrollado en el contexto de la asignatura <b>Robótica aplicada a servicios biomédicos</b> (Universidad de Deusto).<br/>
  </p>
</div>

---

## Tabla de contenidos
1. [Estado del proyecto](#estado-del-proyecto)
2. [Explicación general del proyecto](#explicación-general-del-proyecto)
3. [Demo](#demo)
4. [Qué hace y por qué importa](#qué-hace-y-por-qué-importa)
   - [Features clave](#features-clave)
5. [Quickstart (demo “de verdad”)](#quickstart-demo-de-verdad)
6. [Vista de alto nivel](#vista-de-alto-nivel)
7. [Arquitectura](#arquitectura)
   - [Diagrama completo](#diagrama-completo)
   - [Secuencia “misión de dispensación”](#secuencia-misión-de-dispensación)
   - [Despliegue (Docker + DB stack)](#despliegue-docker--db-stack)
8. [Estructura de carpetas](#estructura-de-carpetas)
9. [Paquetes ROS](#paquetes-ros)
10. [Interfaces ROS](#interfaces-ros)
11. [Requisitos](#requisitos)
12. [Instalación](#instalación)
13. [Configuración](#configuración)
14. [Uso detallado](#uso-detallado)
    - [Casos típicos](#casos-típicos)
    - [Ejemplos, flags y endpoints](#ejemplos-flags-y-endpoints)
    - [Inputs/outputs y formato](#inputsoutputs-y-formato)
15. [Base de datos (Mongo)](#base-de-datos-mongo)
16. [Testing](#testing)
17. [Desarrollo local](#desarrollo-local)
18. [Compatibilidad](#compatibilidad)
19. [Autores](#autores)

---

## Estado del proyecto
- **Active development / WIP**: diseñado para funcionar como demo integrada y, al mismo tiempo, poder iterar por módulos sin romper el conjunto.
- **Stack principal**: **ROS 1 Noetic**. Se recomienda Docker para asegurar reproducibilidad (mismas dependencias, mismo entorno).
- **Arquitectura modular**: UI, DB, coordinación, ejecución y hardware están desacoplados para poder sustituir/iterar componentes.
- **Entrada oficial de la demo final**: `./tirgo_ALL.sh` (script de raíz).

---

## Explicación general del proyecto
En un entorno hospitalario, dispensar y entregar medicación no es solo “mover un robot”: hay reglas (receta / stock / restricciones), trazabilidad y coordinación entre varios subsistemas. En la práctica, gran parte del proceso es repetitivo y exige consistencia para minimizar errores.

**TirGoPharma** modela ese escenario como un flujo completo, reproducible y con capas claras:

1. **Interacción con el usuario (Web + voz)**  
   El usuario solicita medicación desde una **interfaz web**. Si se habilita voz, un módulo de STT captura audio y lo transforma en texto para apoyar la interacción.  
   Importante: la web no es solo “pantallas”: guía el flujo y aplica reglas típicas del dominio (identificación cuando corresponde, validación, etc.).

2. **Datos consistentes (MongoDB con init + seed)**  
   La información de pacientes/medicación/condiciones se almacena en **MongoDB** y se inicializa con scripts y datos semilla.  
   Esto hace que la demo sea **reproducible**: no depende de “preparar cosas a mano” cada vez y facilita depurar sin tocar el robot.

3. **Misión robótica end-to-end (ROS)**  
   Cuando la solicitud es válida, se dispara una **misión** y el sistema coordina:
   - desplazamiento del TIAGo al punto de dispensación,
   - activación del dispensador físico (servos),
   - recogida/entrega (secuencias del brazo),
   - retorno al paciente,
   - cierre de interacción (p. ej., despedida).

4. **Reproducibilidad y guion de demo (scripts)**  
   El repo está organizado para que cualquier persona nueva pueda:
   - levantar la infraestructura (DB + contenedor ROS),
   - compilar y ejecutar con un “botón rojo” (`tirgo_ALL.sh`),
   - localizar rápido dónde está cada cosa,
   - profundizar módulo a módulo leyendo los READMEs específicos.

**¿Qué te vas a encontrar en el repo?**
- Carpeta de **infra** con base de datos y datos semilla.
- Workspace ROS con paquetes.
- Scripts de arranque (especialmente `tirgo_ALL.sh`) que actúan como guion de ejecución.
- Documentación distribuida: README general + READMEs por paquete.

---

## Demo
- **Demo completa (YouTube/Drive):** `[ENLACE_AQUI]`  
- **Demo corta (30–60s):** `[ENLACE_AQUI]`

**GIF recomendado:** `docs/demo.gif` (10–15s, sin audio)
![Demo GIF](docs/demo.gif)

---

## Qué hace y por qué importa
**Problema:** la dispensación/entrega de medicación requiere validación, trazabilidad y coordinación entre interfaz, datos, robótica y hardware.  
**Solución:** TirGoPharma separa el sistema en módulos claros (UI, DB, coordinación, ejecución, hardware) y los conecta con un flujo controlado end-to-end.  
**Por qué importa (académica y técnicamente):** es un ejemplo de robótica biomédica: el robot está integrado con una capa de negocio y una base de datos reproducible, no como un script aislado.

### Features clave
- **Web (Flask)** como interfaz + capa de negocio (flujo guiado y validaciones).
- **MongoDB con init/seed** para demo reproducible y depuración consistente.
- **Coordinación central de misión** para sincronizar pasos y manejar errores/cancelación.
- **Dispensador físico por servos** (Raspberry Pi 3B) integrado en el flujo.
- **Voz opcional** con STT offline para interacción más natural.
- **Paquetes desacoplados + documentación por módulo** para localizar y ampliar rápido.
- **Testing enfocado al flujo** (lo que suele romper demos end-to-end).

---

## Quickstart
> **Punto de entrada oficial (presentación final):** **`./tirgo_ALL.sh`**  
> Este script es el “guion” del sistema: levanta la BD (infra), levanta el contenedor ROS, compila el workspace y lanza los nodos principales.

### Requisitos mínimos
- Linux recomendado.
- Docker + Docker Compose v2.
- Red/entorno de laboratorio según el escenario (TIAGo/RPi).

### Pasos

```bash
cp .env.example .env
# (opcional) ajusta variables en .env
./tirgo_ALL.sh
```

### Smoke test

* **MongoDB + mongo-express**

  * `docker ps | grep tirgo_mongo`
  * mongo-express suele estar en `http://127.0.0.1:8081`

* **UI Web**

  * suele estar en `http://localhost:9001`
  
* **ROS (dentro del contenedor)**

  ```bash
  docker exec -it "$(docker compose ps -q ros1_rob_tirgo)" bash -lc 'rostopic list | grep -E "tirgo|stt"'
  ```

---

## Vista de alto nivel

El repo se entiende mejor como capas. Si solo quieres orientarte, aquí está el mapa mental:

* **Experiencia/negocio (Web):** `tirgo_ui`
  UI web + flujo + validación + puente con ROS.
* **Datos (DB):** `infra/tirgo_db_stack`
  MongoDB + mongo-express + init/seed.
* **Coordinación (misión):** `tirgo_mission_server`
  Orquesta el proceso end-to-end (quién hace qué y en qué orden).
* **Ejecución robótica:** `move`, `tirgo_tiago_arm_seq`, `tiago_speech_node`
  Movimiento, manipulación y cierre de interacción.
* **Hardware (dispensación):** `servo_dispenser`
  Control del dispensador por servos (RPi 3B).
* **HRI opcional:** `stt_vosk`
  Voz → texto para apoyar el flujo.

---

## Arquitectura

### Diagrama completo

```mermaid
---
config:
  layout: elk
---
flowchart LR
    U["Usuario"] -- Web --> UI["tirgo_ui<br/>(Flask + integración ROS)"]
    U -- Voz (opcional) --> STT["stt_vosk<br/>(STT offline)"]
    STT --> UI

    UI -- Validación / consultas --> DB[("MongoDB")]
    UI -- Lanza misión --> MS["tirgo_mission_server<br/>(coordinador de misión)"]

    MS -- Orden de movimiento --> MV["move<br/>(navegación)"]
    MS -- Solicita dispensación --> SD["servo_dispenser<br/>(RPi 3B + servos)"]
    SD -- Envase listo --> ARM["tirgo_tiago_arm_seq<br/>(secuencias brazo)"]

    MV -- Hitos de movimiento --> MS
    ARM -- Hitos de manipulación --> MS

    ARM -- Entrega completada --> SP["tiago_speech_node<br/>(cierre/despedida)"]
    SP --> MS

    MS -- Estado/resultado --> UI
```

### Secuencia “misión de dispensación”

```mermaid
sequenceDiagram
  autonumber
  participant UI as tirgo_ui
  participant MS as tirgo_mission_server
  participant MV as move
  participant SD as servo_dispenser
  participant ARM as tirgo_tiago_arm_seq
  participant SP as tiago_speech_node

  UI->>MS: Inicia misión (solicitud válida)
  MS->>MV: Ir al dispensador
  MV-->>MS: Llegada confirmada

  MS->>SD: Dispensar medicación (compartimento/bin)
  SD-->>MS: Dispensación completada

  ARM-->>MS: Recogida completada
  MV-->>MS: Vuelta al paciente confirmada
  ARM-->>MS: Entrega completada
  SP-->>MS: Cierre de interacción completado

  MS-->>UI: Resultado final (OK / error / cancelación)
```

### Despliegue (Docker + DB stack)

```mermaid
---
config:
  layout: elk
---
flowchart TB
    Host["Máquina host"] -- "./tirgo_ALL.sh" --> ALL["Script de arranque completo"]
    ALL -- docker compose --> ROSC["Contenedor ROS Noetic"] & DBStack["Stack DB (infra/tirgo_db_stack)"]
    DBStack --> MONGO[("MongoDB")] & ME["mongo-express<br>(solo localhost)"]
    ROSC -- acceso --> MONGO
    ROSC -- montaje --> WS["carpeta_compartida/ros_ws"]
```

---

## Estructura de carpetas

> Si llegas nuevo/a: aquí está el “GPS” del repo.

```text
TirGo/
├── Dockerfile
├── docker-compose.yml                  # contenedor ROS Noetic (recomendado)
├── .env / .env.example                 # configuración general del sistema
├── tirgo_stack.sh                      # levanta/reutiliza la DB (infra)
├── tirgo_ALL.sh                        # arranque completo (demo final): DB + contenedor + build + launch
├── infra/
│   └── tirgo_db_stack/                 # Mongo + mongo-express + init/seed
└── carpeta_compartida/
    └── ros_ws/                         # catkin workspace
        └── src/                        # paquetes ROS (cada uno con su README)
```

**Dónde buscar según lo que quieras hacer**

* **Guion de demo / arranque:** `tirgo_ALL.sh`
* **Infra y datos semilla:** `infra/tirgo_db_stack/`
* **Código ROS (nodos/launches):** `carpeta_compartida/ros_ws/src/`
* **UI web y assets:** `carpeta_compartida/ros_ws/src/tirgo_ui/`

---

## Paquetes ROS

> Ruta: `carpeta_compartida/ros_ws/src`
> Este README es la visión global. Los READMEs de cada paquete te dan el “cómo lo corro” y “cómo lo configuro” en detalle.

| Paquete                | Qué vas a encontrar                                                                              | README                                                                                           |
| ---------------------- | ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------ |
| `tirgo_ui`             | Servidor web Flask, rutas, sesión, validación de negocio y puente con ROS. Incluye assets de UI. | [`tirgo_ui/README.md`](carpeta_compartida/ros_ws/src/tirgo_ui/README.md)                         |
| `tirgo_mission_server` | Coordinador end-to-end: orquesta el flujo y sincroniza pasos.                                    | [`tirgo_mission_server/README.md`](carpeta_compartida/ros_ws/src/tirgo_mission_server/README.md) |
| `tirgo_msgs`           | Interfaces compartidas del sistema (acción principal).                                           | [`tirgo_msgs/README.md`](carpeta_compartida/ros_ws/src/tirgo_msgs/README.md)                     |
| `move`                 | Movimiento/navegación y publicación de hitos del proceso.                                        | [`move/README.md`](carpeta_compartida/ros_ws/src/move/README.md)                                 |
| `servo_dispenser`      | Dispensador por servos (RPi 3B): recibe solicitud y acciona el compartimento correspondiente.    | [`servo_dispenser/README.md`](carpeta_compartida/ros_ws/src/servo_dispenser/README.md)           |
| `stt_vosk`             | STT offline: captura audio y publica texto para UI/HRI.                                          | [`stt_vosk/README.md`](carpeta_compartida/ros_ws/src/stt_vosk/README.md)                         |
| `tirgo_tiago_arm_seq`  | Secuencias del brazo para recogida y entrega durante la demo.                                    | *(ver carpeta del paquete)*                                                                      |
| `tirgo_bringup`        | Launchers “top-level” para arrancar conjuntos coherentes de nodos.                               | *(ver carpeta del paquete)*                                                                      |

---

## Interfaces ROS

> **Nivel conceptual (alto nivel):** una misión principal + sincronización de hitos entre módulos.

* La **UI** inicia una misión y recibe estado/resultado.
* El **coordinador** guía el flujo y espera confirmaciones de cada parte del sistema.
* Los módulos de **ejecución** (movimiento/dispensación/brazo) reportan hitos para mantener el proceso trazable.

📌 Para el detalle fino (definiciones exactas y comportamiento):

* [`tirgo_mission_server/README.md`](carpeta_compartida/ros_ws/src/tirgo_mission_server/README.md)
* [`tirgo_msgs/README.md`](carpeta_compartida/ros_ws/src/tirgo_msgs/README.md)

> Nota conceptual importante: en el sistema se maneja la correspondencia entre “medicación” y “compartimento/bin” del dispensador para poder conectar la lógica de negocio con la ejecución física en la demo.

---

## Requisitos

### Opción recomendada (Docker)

* Docker + Docker Compose v2
* Linux con X11 si necesitas interfaces gráficas
* Audio en host si usas STT (micrófono)

### Opción nativa

* Ubuntu 20.04 + ROS 1 Noetic
* Dependencias por paquete (ver READMEs específicos)
* MongoDB (o usar el stack de `infra/`)

### Hardware (demo completa)

* TIAGo (o equivalente)
* Raspberry Pi 3B + servos (dispensador)
* Micrófono (opcional)

---

## Instalación

```bash
git clone <URL_DEL_REPO>
cd TirGo

cp .env.example .env
# edita si hace falta:
nano .env
```

(Optativo) Levantar solo DB:

```bash
./tirgo_stack.sh
```

---

## Configuración

La configuración se divide en dos niveles:

1. **Sistema (raíz):** `.env`
   credenciales de DB, secretos web y parámetros generales del stack.
2. **Infra DB:** `infra/tirgo_db_stack/`
   docker compose de DB + scripts de init/seed.

Referencias:

* `.env.example`
* `infra/tirgo_db_stack/docker-compose.yml`
* `infra/tirgo_db_stack/mongo-init/`

---

## Uso detallado

### Casos típicos

#### 1) Dispensación completa (modo demo)

1. Arranca todo con `./tirgo_ALL.sh`
2. Accede a la web (UI)
3. Sigue el flujo de solicitud/validación
4. Lanza misión y observa el proceso end-to-end

➡️ Detalle real de pantallas y flujo:

* [`tirgo_ui/README.md`](carpeta_compartida/ros_ws/src/tirgo_ui/README.md)

#### 2) Interacción por voz (opcional)

Con el stack levantado, el módulo STT permite capturar voz → texto para apoyar el flujo.

➡️ Configuración de dispositivos y STT:

* [`stt_vosk/README.md`](carpeta_compartida/ros_ws/src/stt_vosk/README.md)

#### 3) Depuración sin hardware

Se puede validar por módulos (UI+DB, coordinación de misión, etc.) sin depender del dispensador físico real.

➡️ Coordinación, ejecución y troubleshooting:

* [`tirgo_mission_server/README.md`](carpeta_compartida/ros_ws/src/tirgo_mission_server/README.md)

---

### Ejemplos, flags y endpoints

* **Script demo oficial:** `./tirgo_ALL.sh`
* **Stack DB:** `./tirgo_stack.sh` o `infra/tirgo_db_stack/`
* **Rutas/endpoints web (código):** `carpeta_compartida/ros_ws/src/tirgo_ui/routes/`
* **Launchers:** `carpeta_compartida/ros_ws/src/tirgo_bringup/` *(si aplica)*

📌 Para el detalle exacto de endpoints y navegación:

* [`tirgo_ui/README.md`](carpeta_compartida/ros_ws/src/tirgo_ui/README.md)

---

### Inputs/outputs y formato

**Inputs típicos**

* Identificación del paciente (cuando aplica)
* Selección de medicación / petición desde UI
* Confirmación del usuario (web/voz)

**Outputs típicos**

* Progreso/estado visible en la UI
* Resultado final de misión (éxito/error/cancelación)
* Evidencias: logs/monitorización (y vídeos si se añaden)

---

## Base de datos (Mongo)

La DB aterriza el componente biomédico: el robot actúa tras validación, no “porque sí”.

En `infra/tirgo_db_stack/` encontrarás:

* Stack Mongo + mongo-express
* Scripts de init (usuarios/estructura/índices)
* Seed de datos para demo reproducible

Dónde mirar:

* `infra/tirgo_db_stack/docker-compose.yml`
* `infra/tirgo_db_stack/mongo-init/`

### ⚠️ Nota de consistencia (importante para que no pete la demo)

El init de Mongo crea un usuario/credenciales concretas (definidas por los scripts de `mongo-init` y/o variables del stack).
Si la web no conecta a Mongo, lo más típico es que el `mongo_uri` que está usando la UI no coincida con el usuario/clave reales creados en la inicialización.

**Solución recomendada:** alinea las credenciales en `.env` con las del stack DB y usa un `mongo_uri` coherente.

Ejemplo (si tu usuario es `tirgo_user` y tu password coincide con `TIRGO_DB_PASSWORD`):

```bash
roslaunch tirgo_ui web.launch mongo_uri:=mongodb://tirgo_user:tirgo_pass_cambia@127.0.0.1:27017/tirgo?authSource=tirgo
```

> Si estás usando el stack completo con `./tirgo_ALL.sh`, lo ideal es que este ajuste quede reflejado en `.env`/config para no depender de “parches manuales” el día de demo.

---

## Testing

El testing se enfoca en lo que suele romper demos: el flujo end-to-end y la coordinación.

Referencias:

* [`tirgo_mission_server/README.md`](carpeta_compartida/ros_ws/src/tirgo_mission_server/README.md)
* [`stt_vosk/README.md`](carpeta_compartida/ros_ws/src/stt_vosk/README.md) *(si aplica)*

---

## Desarrollo local

Ruta recomendada para modificar el sistema sin perderte:

1. Lee este README (mapa global).
2. Entra al README del paquete que vayas a tocar.
3. Valida integración con `./tirgo_ALL.sh`.

**Regla de oro (debug):** `.env` → DB/seed → `tirgo_ALL.sh` → README del paquete.

---

## Compatibilidad

| Componente   | Recomendado                                 |
| ------------ | ------------------------------------------- |
| OS           | Linux (Ubuntu)                              |
| ROS          | ROS 1 Noetic                                |
| DB           | MongoDB (via `infra/tirgo_db_stack`)        |
| Contenedores | Docker + docker compose                     |
| Dispensador  | Raspberry Pi 3B + pigpio (si hardware real) |

---

## Autores

Proyecto desarrollado como parte de la asignatura **Robótica aplicada a servicios biomédicos** (Universidad de Deusto).

**Equipo TirGoPharma**

* Katrin Muñoz Errasti
* Lucas Campillo de Andrés
* Ángel Romero de la Maza

**Institución:** Universidad de Deusto (contexto académico / laboratorio)
