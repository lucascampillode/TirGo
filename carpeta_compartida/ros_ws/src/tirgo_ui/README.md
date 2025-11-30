# tirgo_ui

Interfaz web de **TirgoPharma** integrada en **ROS 1 (Noetic)**.

Este paquete expone una aplicación **Flask** dentro de un paquete ROS para poder lanzarla con `rosrun`/`roslaunch` y hablar con el resto del sistema por **topics ROS**. La web permite:

- Desbloquear la interfaz por **voz** (hotword) escuchando el topic de STT.
- Consultar pacientes, recetas y medicamentos guardados en **MongoDB**.
- Lanzar una **operación de dispensación** publicando órdenes al robot/dispensador.
- Mostrar el estado de la misión leyendo topics ROS.
- Probar todo en **modo desarrollo** (simular la hotword) sin tener el robot encendido.

Está pensada para funcionar junto al paquete `stt_vosk` (reconocimiento de voz) y con una instancia de MongoDB.

---

## 1. Estructura del paquete

```text
tirgo_ui/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── web.launch            ← lanzamiento típico
│   └── ...                   ← otros launch de pruebas
├── scripts/
│   └── tirgo_web_server      ← entrypoint ROS que arranca Flask
├── src/
│   └── tirgo_ui/
│       ├── app.py            ← crea la app Flask y carga todo
│       ├── config.py         ← puerto, hotword, topic STT, etc.
│       ├── rosio.py          ← pubs/subs ROS
│       ├── storage_mongo.py  ← acceso a MongoDB
│       ├── services.py       ← lógica de dispensación
│       ├── session.py        ← control de sesión activa
│       └── routes/           ← blueprints Flask
│           ├── main.py       ← pantalla principal + simular hola
│           ├── consultar.py  ← buscar paciente / recetas
│           ├── leer.py       ← confirmar y dispensar
│           └── diagnostico.py← flujo guiado
├── templates/                ← HTML
└── static/                   ← CSS, JS, imágenes de medicamentos
````

---

## 2. Diagrama general del sistema

```mermaid
flowchart TB
 subgraph Usuario["Usuario"]
        U["Personal / Paciente"]
  end
 subgraph Web[" "]
        UI["Interfaz web<br>routes + templates"]
        ROSIO["rosio.py<br>pub/sub ROS"]
        MONGOHELP["storage_mongo.py"]
  end
 subgraph ROS["ROS 1 Master"]
        STT["stt/text<br>(nodo STT externo)"]
        MISSION["/tirgo/mission/start"]
        DISPREQ["/tirgo/dispense/request"]
        UISTATE["/tirgo/ui/state"]
        TIAGOARR["/tirgo/tiago/arrived"]
        DISPREADY["/tirgo/dispense/ready"]
        TIAGOPICKED["/tirgo/tiago/picked"]
  end
 subgraph Robot["TIAGo / Dispensador"]
        RBT["Navegación / Recogida"]
        DSP["Dispensador físico"]
  end
 subgraph DB["MongoDB"]
        MEDS[("medicamentos")]
        PAC[("pacientes")]
        REC[("recetas")]
        LOGS[("dispenses")]
  end
    U -- navegador --> UI
    UI --> ROSIO & MONGOHELP
    ROSIO -- pub --> MISSION & DISPREQ & UISTATE
    MONGOHELP --> MEDS & PAC & REC & LOGS
    STT -- texto con hotword --> ROSIO
    MISSION --> RBT
    DISPREQ --> DSP
    RBT -- /tirgo/tiago/arrived --> TIAGOARR
    RBT -- /tirgo/tiago/picked --> TIAGOPICKED
    DSP -- /tirgo/dispense/ready --> DISPREADY
    TIAGOARR --> ROSIO
    TIAGOPICKED --> ROSIO
    DISPREADY --> ROSIO
```

---

## 3. Dependencias

* **ROS 1 Noetic** (`rospy`, `std_msgs`)
* **Python 3** dentro del entorno ROS
* **Flask**
* **pymongo**
* **MongoDB** accesible (local o en contenedor)
* (Opcional pero recomendado) **stt_vosk** publicando en `stt/text`

---

## 4. Variables de entorno importantes

* `PORT` → puerto HTTP de la web. **Por defecto: 9001**.
* `MONGO_URI` → cadena de conexión a Mongo:

  * Con auth:

    ```bash
    export MONGO_URI="mongodb://tirgoAdmin:tirgo@127.0.0.1:27017/tirgo?authSource=tirgo"
    ```
  * Sin auth (modo pruebas):

    ```bash
    export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
    ```
* `TIRGO_HOTWORD` → palabra que debe llegar desde el STT para desbloquear. Ej: `hola tirgo`
* `TIRGO_STT_TOPIC` → topic donde escucha el texto:

  ```bash
  export TIRGO_STT_TOPIC="stt/text"
  ```

* `TIRGO_DEV` → si vale `1`, activa endpoints de prueba (simular hola).
* `FLASK_SECRET_KEY` → clave de Flask para la sesión.

---

## 5. Instalación

```bash
cd ~/carpeta_compartida/ros_ws/src
# aquí copias tirgo_ui
cd ..
catkin_make
source devel/setup.bash
```

Asegúrate de que el contenedor / máquina tiene acceso a MongoDB.

---

## 6. Puesta en marcha

### 6.1 Lanzar el STT 


```bash
roslaunch stt_vosk stt_vosk.launch
```

Eso te publicará en `stt/text`.

### 6.2 Lanzar la web

```bash
roslaunch tirgo_ui web.launch
```

Esto:

* exporta las variables,
* arranca el script `tirgo_web_server`,
* levanta Flask en el puerto indicado.

### 6.3 Lanzar a mano (sin launch)

```bash
export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
export TIRGO_STT_TOPIC="stt/text"
export TIRGO_DEV=1
rosrun tirgo_ui tirgo_web_server
```

---

## 7. Flujo típico de uso

1. **Llega la voz**
   El nodo de voz (`stt_vosk`) publica en `stt/text` algo que contiene la hotword, por ejemplo **"hola tirgo"**.

2. **La web se desbloquea**
   `tirgo_ui` está suscrito a ese topic, detecta la hotword y pasa del estado “esperando voz” al **menú principal**.

3. **Menú con 3 opciones**
   En la pantalla salen estas tres:

   1. **Leer**

      * La web te enseña los **medicamentos disponibles** (los que están activos en Mongo).
      * Si el medicamento **no necesita receta**, te deja pedirlo.
      * Si **sí necesita receta**, te pide identificarte y comprueba en Mongo si tienes **receta activa**.
      * Si hay receta y hay stock → pasa a la fase de dispensación.

   2. **Consultar**

      * Primero te identificas.
      * Luego te muestra **todas tus recetas activas** que hay en Mongo.
      * Desde esa lista puedes pedir la que quieras → y entonces dispara la dispensación.

   3. **Diagnosticar**

      * Te hace un mini test.
      * Según lo que respondas, te propone **una de las 4 opciones de medicamentos que tienes en la base** o, si no cuadra, te dice **descanso / ir al médico**.

4. **Cuando el usuario pide algo** la web hace la parte “técnica”:

   * comprueba en Mongo **stock** y **receta**,
   * registra la dispensación,
   * publica en **`/tirgo/mission/start`** lo que debe hacer el robot,
   * publica en **`/tirgo/dispense/request`** el `bin_id`/cajetín que toca.

5. **El hardware responde**
   El robot o el dispensador mandan los topics de vuelta (`/tirgo/dispense/ready`, `/tirgo/tiago/arrived`, `/tirgo/tiago/picked`) y la web los muestra como estado.

---

## 8. Topics ROS usados

**Publica:**

* `tirgo/ui/state` (`std_msgs/String`)
  Estados de la UI: `IDLE`, `HOTWORD:hola tirgo`, `DISPENSING`, etc.
* `tirgo/ui/status` (`std_msgs/String`)
  Mensajes más detallados (a veces en JSON).
* `tirgo/ui/error` (`std_msgs/String`)
* `/tirgo/mission/start` (`std_msgs/String`)
  Para decir al robot “empieza la misión X”.
* `/tirgo/dispense/request` (`std_msgs/Int32`)
  Para decir al dispensador qué bandeja/casilla abrir.

**Suscribe:**

* `stt/text` (o `/stt/text`) (`std_msgs/String`)
  Texto reconocido por el STT. **Este es el importante.**
* `/tirgo/tiago/arrived` (`std_msgs/Bool`)
* `/tirgo/dispense/ready` (`std_msgs/Bool`)
* `/tirgo/tiago/picked` (`std_msgs/Bool`)

> Nota: si tu nodo de voz publica en `stt/text` sin `/`, define `TIRGO_STT_TOPIC=stt/text` y listo.

---

## 9. Base de datos (MongoDB)

La aplicación usa una base de datos MongoDB usa `storage_mongo.py` para acceder a la base llamada **`tirgo`** (la que configures en `MONGO_URI`). 

1. ### `medicamentos`

   Aquí se guarda el catálogo que la web enseña:

   | id  | nombre             | tipo | bin_id | stock |
   | --- | ------------------ | ---- | ------ | ----- |
   | 101 | Paracetamol 1g     | L    | 1      | 9     |
   | 202 | Ibuprofeno 400 mg  | L    | 2      | 6     |
   | 303 | Amoxicilina 500 mg | R    | 3      | 0     |
   | 404 | Omeprazol 20 mg    | L    | 4      | 10    |

   * **`tipo`**: si es “libre” (`L`) o “receta” (`R`).
   * **`bin_id`**: es el cajetín del dispensador que se debe abrir cuando se pide ese medicamento.
   * **`stock`**: unidades disponibles.

2. ### `pacientes`

   Lista de personas que pueden pedir medicamentos. Cada documento tiene:

   * `nombre`, `apellidos`
   * versiones normalizadas (`nombre_norm`, `apellidos_norm`) para buscar
   * **`dni_hash`**: el DNI no se guarda tal cual, se guarda hasheado.

   Esto es lo que la web usa cuando te identificas en “Consultar” o cuando un medicamento pide receta.

3. ### `recetas`

   Relaciona **paciente** + **medicamento** + **si está activa**.

5. ### `dispenses` (registro de dispensaciones)

   * `ts` (timestamp)
   * `medicamento_id`
   * `dni_hash` (a veces vacío si es anónimo)

   Aquí la web va dejando un **histórico** de lo que se ha entregado.

---

### Resumen de cómo lo usa la web

1. El usuario entra en una de las 3 opciones (leer, consultar, diagnosticar).
2. Cuando pide un medicamento concreto:

   * la web mira en **`medicamentos`** si existe, si tiene `stock > 0` y cuál es su `bin_id`;
   * si el medicamento es de **tipo `R`**, mira en **`recetas`** si ese paciente tiene una con `activa: true`;
   * si todo OK, registra en **`dispenses`** un documento con `ts`, `medicamento_id` y el `dni_hash` (si lo había);
   * por último publica a ROS para que el dispensador abra el bin correspondiente.


## 10. Modo desarrollo (simular voz)

Si exportas:

```bash
export TIRGO_DEV=1
```

la web expone un endpoint que simula la llegada de la hotword:

```bash
curl -X POST http://localhost:9001/simulate_hola
```

Con eso la UI se desbloquea igual que si hubieras dicho “hola tirgo”.

---

## 11. Comandos útiles

```bash
# build
cd ~/carpeta_compartida/ros_ws && catkin_make && source devel/setup.bash

# lanzar STT primero (si lo usas)
roslaunch stt_vosk stt_vosk.launch

# lanzar la web
roslaunch tirgo_ui web.launch

# lanzar a mano
export MONGO_URI="mongodb://127.0.0.1:27017/tirgo"
export TIRGO_STT_TOPIC="stt/text"
export TIRGO_DEV=1
rosrun tirgo_ui tirgo_web_server
```


