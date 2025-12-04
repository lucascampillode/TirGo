# tirgo_ui

Interfaz web de **TirgoPharma** integrada en **ROS 1 (Noetic)**.

Este paquete expone una aplicación **Flask** dentro de un paquete ROS para poder lanzarla con `rosrun`/`roslaunch` y comunicarse con el resto del sistema mediante **ROS**. La web sirve como capa de interacción con el usuario y permite:

- Desbloquear la interfaz por **voz** (hotword) escuchando el tópico de STT.
- Consultar pacientes, recetas y medicamentos almacenados en **MongoDB**.
- Lanzar una **misión de dispensación** a través del **Action Server** `/tirgo/mission`.
- Mostrar el estado de la misión a partir del feedback/result del Action y de tópicos ROS de estado.
- Probar el sistema en **modo desarrollo** (simular la hotword) sin necesidad de disponer del robot.

Está pensada para funcionar junto al paquete `stt_vosk` (reconocimiento de voz), con `tirgo_mission_server` (servidor de acciones de misión) y con una instancia de MongoDB.

---

## 1. Estructura del paquete

```text
tirgo_ui/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── web.launch            ← lanzamiento típico (dev / despliegue)
│   └── ...                   ← otros launch de pruebas o legado
├── scripts/
│   └── tirgo_web_server      ← entrypoint ROS que arranca Flask
├── src/
│   └── tirgo_ui/
│       ├── app.py            ← crea la app Flask y registra blueprints
│       ├── config.py         ← puerto, hotword, tópico STT, etc.
│       ├── rosio.py          ← Action client + pubs/subs ROS
│       ├── storage_mongo.py  ← acceso a MongoDB
│       ├── services.py       ← lógica auxiliar (actualmente mínima/legado)
│       ├── session.py        ← control de sesión activa en la UI
│       └── routes/           ← blueprints Flask
│           ├── main.py       ← pantalla principal + simular hotword
│           ├── consultar.py  ← búsqueda de paciente / recetas
│           ├── leer.py       ← confirmar y lanzar la misión
│           └── diagnostico.py← flujo guiado de diagnóstico
├── templates/                ← HTML (Jinja2)
└── static/                   ← CSS, JS, imágenes de medicamentos
````

---

## 2. Diagrama general del sistema

```mermaid
flowchart TB
  subgraph Usuario["Usuario"]
    U["Personal / Paciente"]
  end

  subgraph Web["tirgo_ui (Flask + ROS)"]
    UI["Interfaz web<br>routes + templates"]
    ROSIO["rosio.py<br>Action client + pubs/subs"]
    MONGOHELP["storage_mongo.py"]
  end

  subgraph ROS["ROS 1 Master"]
    STT["/stt/text<br>(nodo STT externo)"]

    subgraph MISSION["tirgo_mission_server"]
      ACT["/tirgo/mission<br>TirgoMissionAction"]
    end

    UISTATE["/tirgo/ui/state<br>/tirgo/ui/status"]
    SAY["/tirgo/say"]
  end

  subgraph Robot["TIAGo / Dispensador"]
    RBT["Navegación / Recogida"]
    DSP["Dispensador físico<br>(servo_dispenser)"]
  end

  subgraph DB["MongoDB (BD 'tirgo')"]
    MEDS[("medicamentos")]
    PAC[("pacientes")]
    REC[("recetas")]
    LOGS[("dispensaciones / logs")]
  end

  %% Flujo web
  U -- navegador --> UI
  UI --> ROSIO
  UI --> MONGOHELP
  MONGOHELP --> MEDS & PAC & REC & LOGS

  %% Voz
  STT -- texto con hotword --> ROSIO

  %% Acción de misión
  ROSIO -- goal (patient_id, med_id) --> ACT
  ACT -- feedback/result --> ROSIO

  %% Estado UI / voz robot
  ROSIO -- pub estado --> UISTATE
  ROSIO -- pub texto --> SAY

  %% Backend misión (resumen)
  ACT --> RBT
  ACT --> DSP
```

---

## 3. Dependencias

* **ROS 1 Noetic**:

  * `rospy`
  * `actionlib`
  * `std_msgs`
  * `tirgo_msgs` (definición de `TirgoMissionAction`)
* **Python 3** dentro del entorno ROS
* **Flask**
* **pymongo**
* **MongoDB** accesible (local o en contenedor)
* (Opcional pero recomendado) **stt_vosk** publicando en `/stt/text`
* **tirgo_mission_server** proporcionando el Action Server `/tirgo/mission`

---

## 4. Variables de entorno importantes

* `PORT` → puerto HTTP de la web.
  Valor por defecto: `9001`.

* `MONGO_URI` → cadena de conexión a MongoDB, por ejemplo:

  ```bash
  # Con autenticación (ejemplo)
  export MONGO_URI="mongodb://tirgo_app:tirgo@127.0.0.1:27017/tirgo?authSource=tirgo"

  # Sin autenticación (modo pruebas)
  export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
  ```

* `TIRGO_HOTWORD` → palabra que debe llegar desde el STT para desbloquear la interfaz.
  Ejemplo: `hola tirgo`.

* `TIRGO_STT_TOPIC` → tópico desde el que se lee el texto del STT:

  ```bash
  export TIRGO_STT_TOPIC="/stt/text"
  ```

* `TIRGO_DEV` → si vale `1`, activa endpoints de prueba (por ejemplo, simular hotword).

* `FLASK_SECRET_KEY` → clave de Flask para la sesión.

* `FLASK_DEBUG` → modo debug de Flask (`1`/`0`).

Estas variables se suelen establecer desde `launch/web.launch`.

---

## 5. Instalación

```bash
cd ~/carpeta_compartida/ros_ws/src
# aquí se ubica el paquete tirgo_ui
cd ..
catkin_make
source devel/setup.bash
```

Asegúrate de que la máquina / contenedor tiene acceso a la instancia de MongoDB.

---

## 6. Puesta en marcha

### 6.1. Lanzar STT y Action Server de misión

```bash
# Nodo de reconocimiento de voz (opcional pero recomendado)
roslaunch stt_vosk stt_vosk.launch

# Servidor de acciones de misión (coordinación TIAGo + dispensador)
roslaunch tirgo_mission_server tirgo_mission_server.launch
```

### 6.2. Lanzar la web con `web.launch`

El launch `web.launch` está preparado para dos modos:

#### a) Modo desarrollo (por defecto)

```bash
roslaunch tirgo_ui web.launch
```

Este modo típicamente:

* Expone la web en `PORT=9001`.
* Activa `TIRGO_DEV=1` (endpoints de prueba).
* Suele arrancar Flask con `FLASK_DEBUG=1`.

#### b) Modo despliegue (configuración más estricta)

```bash
roslaunch tirgo_ui web.launch \
  tirgo_dev:=0 \
  flask_debug:=0 \
  flask_secret_key:=<cadena-secreta-larga> \
  mongo_uri:=mongodb://tirgo_app:<password>@<HOST_MONGO>:27017/tirgo?authSource=tirgo
```

En este modo se desactivan las facilidades de desarrollo y se emplea una configuración más adecuada para demostraciones controladas.

### 6.3. Lanzar a mano (sin launch)

```bash
export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
export TIRGO_STT_TOPIC="/stt/text"
export TIRGO_DEV=1
rosrun tirgo_ui tirgo_web_server
```

---

## 7. Flujo típico de uso

1. **Llega la voz**
   El nodo de voz (`stt_vosk`) publica en `/stt/text` una cadena que contiene la hotword configurada (por ejemplo, `"hola tirgo"`).

2. **La web se desbloquea**
   `tirgo_ui` está suscrito a ese tópico, detecta la hotword a través de `rosio.py` y cambia del estado “esperando voz” al **menú principal**.

3. **Menú con 3 opciones principales**

   1. **Leer**

      * La web muestra los **medicamentos disponibles** (activos en Mongo).
      * Si el medicamento **no requiere receta**, permite solicitarlo directamente.
      * Si **requiere receta**, solicita identificación y comprueba en Mongo si existe **receta activa**.
      * Si hay receta activa y stock suficiente → se prepara la misión de dispensación.

   2. **Consultar**

      * Solicita identificación del paciente.
      * Muestra **todas las recetas activas** asociadas en Mongo.
      * Desde esa lista se puede seleccionar una receta concreta para lanzar una misión de dispensación.

   3. **Diagnosticar**

      * Presenta un breve cuestionario.
      * En función de las respuestas, propone uno de los medicamentos disponibles o recomienda descanso / consulta médica.
      * Si se acepta la propuesta, se sigue el mismo flujo de validación (receta/stock) y lanzamiento de misión.

4. **Validación técnica y lanzamiento de la misión**

   * La web verifica en MongoDB:

     * Que el medicamento existe y tiene `stock > 0`.
     * Que existe una receta activa, en el caso de medicamentos sujetos a prescripción.
   * Registra la operación en la colección de dispensaciones / logs.
   * Construye un goal `TirgoMissionGoal` con `patient_id` y `med_id` y lo envía al Action Server `/tirgo/mission` mediante `rosio.py`.

5. **Seguimiento de la misión**

   * `tirgo_ui` recibe el feedback y el resultado del Action:

     * Muestra el estado (por ejemplo, “yendo al dispensador”, “esperando dispensación”, “entregando al paciente”, etc.).
     * Indica el resultado final (éxito o código de error).
   * Además, publica mensajes de estado en `tirgo/ui/state` y `tirgo/ui/status` y puede enviar mensajes de texto a `/tirgo/say` para que TIAGo los pronuncie.

---

## 8. ROS: acción y tópicos utilizados

### 8.1. Acción de misión

* **Nombre**: `/tirgo/mission`
* **Tipo**: `tirgo_msgs/TirgoMissionAction`

`tirgo_ui` actúa como **cliente** de esta acción, enviando goals de dispensación y recibiendo feedback/result para actualizar la interfaz.

### 8.2. Tópicos publicados

* `tirgo/ui/state` (`std_msgs/String`)
  Estados de alto nivel de la interfaz (`IDLE`, `WAITING_HOTWORD`, `READY`, `DISPENSING`, `ERROR`, etc.).

* `tirgo/ui/status` (`std_msgs/String`)
  Información de estado más detallada, habitualmente en texto o JSON.

* `tirgo/ui/error` (`std_msgs/String`)
  Mensajes de error orientados a diagnóstico.

* `/tirgo/say` (`std_msgs/String`)
  Mensajes de texto que el nodo de voz de TIAGo puede convertir en audio.

### 8.3. Tópicos suscritos

* `TIRGO_STT_TOPIC` (por defecto `/stt/text`) (`std_msgs/String`)
  Texto reconocido por el sistema de STT. Se utiliza para detectar la hotword y, en su caso, otras órdenes de voz.

---

## 9. Base de datos (MongoDB)

La aplicación utiliza `storage_mongo.py` para acceder a la base **`tirgo`** (definida por `MONGO_URI`). Esquema conceptual:

1. ### `medicamentos`

   Catálogo que la web presenta al usuario:

   | id  | nombre             | tipo | bin_id | stock |
   | --- | ------------------ | ---- | ------ | ----- |
   | 101 | Paracetamol 1g     | L    | 1      | 9     |
   | 202 | Ibuprofeno 400 mg  | L    | 2      | 6     |
   | 303 | Amoxicilina 500 mg | R    | 3      | 0     |
   | 404 | Omeprazol 20 mg    | L    | 4      | 10    |

   * `tipo`: “libre” (`L`) o “receta” (`R`).
   * `bin_id`: cajetín físico del dispensador asociado.
   * `stock`: unidades disponibles.

2. ### `pacientes`

   Información de los pacientes:

   * `nombre`, `apellidos`
   * versiones normalizadas (`nombre_norm`, `apellidos_norm`) para búsqueda
   * `dni_hash`: hash del DNI (no se almacena el identificador en claro)

3. ### `recetas`

   Asociación paciente–medicamento–estado:

   * `dni_hash`
   * `medicamento_id`
   * `activa` (true/false)
   * campos adicionales (fechas, comentarios, etc.)

4. ### `dispensaciones` / `logs`

   Registro histórico de las operaciones de dispensación:

   * `ts` (timestamp)
   * `medicamento_id`
   * `dni_hash` (cuando aplica)
   * información sobre el resultado de la misión (éxito, código de error, etc.)

### Resumen de uso

1. El usuario selecciona una de las opciones (leer, consultar, diagnosticar).
2. Al solicitar un medicamento concreto, la web:

   * Comprueba en `medicamentos` su existencia, stock y `bin_id`.
   * En caso necesario, comprueba en `recetas` que exista una receta activa para ese paciente.
   * Si la validación es correcta, registra la operación en la colección de dispensaciones/logs.
   * Lanza el goal correspondiente al Action `/tirgo/mission`.

---

## 10. Modo desarrollo (simulación de voz)

Si se define:

```bash
export TIRGO_DEV=1
```

la web expone un endpoint que simula la llegada de la hotword (nombre exacto según configuración), por ejemplo:

```bash
curl -X POST http://localhost:9001/simulate_hola
```

Este mecanismo permite desbloquear la interfaz sin necesidad de ejecutar el nodo de STT, útil para pruebas y desarrollo.

---

## 11. Comandos útiles

```bash
# Compilación del workspace
cd ~/carpeta_compartida/ros_ws
catkin_make
source devel/setup.bash

# Lanzar STT (si se desea entrada por voz)
roslaunch stt_vosk stt_vosk.launch

# Lanzar el servidor de acciones de misión
roslaunch tirgo_mission_server tirgo_mission_server.launch

# Lanzar la web en modo desarrollo (valores por defecto de web.launch)
roslaunch tirgo_ui web.launch

# Lanzar la web de forma manual
export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
export TIRGO_STT_TOPIC="/stt/text"
export TIRGO_DEV=1
rosrun tirgo_ui tirgo_web_server
```
