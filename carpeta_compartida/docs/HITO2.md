# Hito 2: Diseño Conceptual del Sistema Robótico

**Proyecto:** TirGo Pharma  
**Fecha:** 21 de Noviembre de 2025  
**Equipo:**

  * Katrin Muñoz Errasti
  * Lucas Campillo de Andrés
  * Ángel Romero de la Maza

-----

## Índice de Contenidos

* [1. Resumen del Problema Biomédico](#1-resumen-del-problema-biomédico)
* [2. Arquitectura del Sistema](#2-arquitectura-del-sistema)
    * [2.a) Diagrama General y Lógica de Control](#2a-diagrama-general-y-lógica-de-control)
    * [2.b) Especificación de Componentes de Hardware](#2b-especificación-de-componentes-de-hardware)
    * [2.c) Interfaz de Usuario (UI/UX) y Flujo de Interacción](#2c-interfaz-de-usuario-uiux-y-flujo-de-interacción)
* [3. Diseño de Software y Comunicación](#3-diseño-de-software-y-comunicación)
    * [3.a) Arquitectura de Nodos en ROS 1](#3a-arquitectura-de-nodos-en-ros-1)
    * [3.b) Estructura del Repositorio y Gestión de Paquetes](#3b-estructura-del-repositorio-y-gestión-de-paquetes)
    * [3.c) Descripción de Contenedores Docker y Dependencias](#3c-descripción-de-contenedores-docker-y-dependencias)
* [4. Análisis de Viabilidad Técnica](#4-análisis-de-viabilidad-técnica)
    * [4.a) Identificación de Limitaciones y Riesgos](#4a-identificación-de-limitaciones-y-riesgos)
    * [4.b) Estrategia de Mitigación y Pruebas Iniciales](#4b-estrategia-de-mitigación-y-pruebas-iniciales)
* [5. Cronograma de Desarrollo](#5-cronograma-de-desarrollo)
    * [5.a) Plan Temporal](#5a-plan-temporal)
    * [5.b) Reparto de Responsabilidades Actualizado](#5b-reparto-de-responsabilidades-actualizado)

-----

## 1\. Resumen del Problema Biomédico

En el contexto hospitalario, una de las tareas que más tiempo y recursos del personal sanitario consume es la dispensación de medicamentos. Este proceso, además de implicar un elevado coste operativo, presenta riesgos asociados a errores humanos (confusión de dosis o fármacos) y retrasos en la entrega, afectando tanto a la seguridad del paciente como a la eficiencia del sistema de salud.

El proyecto **TirGo Pharma** plantea una solución basada en la automatización del proceso de dispensación y entrega de medicamentos mediante la integración del robot **TIAGo** con un dispensador automático controlado por una **Raspberry Pi**. El dispensador almacena distintos tipos de comprimidos y los libera bajo demanda en un punto de recogida. El robot TIAGo recoge el medicamento con su gripper y lo transporta de forma autónoma hasta el paciente, garantizando una entrega segura y trazable.

La interacción con el paciente se realiza a través de una interfaz conversacional, que permite confirmar la identidad del usuario, explicar la medicación prescrita y resolver dudas básicas sobre su administración. De esta manera, el sistema combina robótica móvil, manipulación, y comunicación natural para ofrecer un servicio integral de dispensación.

### Impacto esperado:

  * **Personal Sanitario:** Reducción significativa del tiempo dedicado a transporte y entrega, disminuyendo la carga operativa y el riesgo de error humano.
  * **Sistema Hospitalario:** Incremento de la eficiencia global y la trazabilidad del medicamento, optimizando el flujo de trabajo y la gestión de recursos.
  * **Paciente:** Mayor adherencia al tratamiento, mejor comprensión de las indicaciones médicas y una experiencia más cercana y personalizada.

El entorno previsto de implementación es un hospital o centro de salud, limitándose al manejo de medicamentos previamente autorizados y cumpliendo con los requisitos de privacidad y seguridad de datos personales y clínicos.

-----

## 2\. Arquitectura del Sistema

### 2.a) Diagrama General y Lógica de Control

El sistema se despliega sobre la red local del robot TIAGo (ROS Master), integrando el ordenador externo (Interfaz/BD) y la Raspberry Pi del dispensador en un mismo espacio.

```mermaid
---
config:
  look: classic
  theme: neutral
  layout: dagre
---
flowchart LR
 subgraph USU["Paciente / Personal"]
        PAC[("Paciente")]
        PER[("Personal sanitario")]
  end
 subgraph UI["Paquete ROS: tirgo_ui (Flask + ROS)"]
        HTML[["Páginas HTML<br>templates + static"]]
        UINODE(("Nodo ROS<br>tirgo_web_server"))
  end
 subgraph VOZ["Entrada por voz"]
        STT(("Nodo STT<br>/stt_vosk"))
  end
 subgraph DATA["Backend / Datos"]
        ME[["MongoExpress"]]
        DB[("MongoDB<br>recetas, meds/stock, pacientes,<br>registro dispensaciones")]
  end
 subgraph DISP["Paquete ROS: dispensador (Raspberry Pi 3B)"]
        DISP_NODE(("Nodo dispensador"))
        ACT[["Servos / LEDs"]]
  end
 subgraph ROBOT["Paquete ROS: robot / TIAGo"]
        R_NODE(("Nodo misión / navegación"))
  end
    HTML --> UINODE
    STT -- /stt/text --> UINODE
    ME --> DB
    UINODE -. MONGO_URI .-> DB
    DISP_NODE --> ACT
    PAC -- navegador web --> HTML
    PER -- carga física meds --x DISP
    PER -- gestiona datos --> ME
    UINODE -- /tirgo/dispense/request<br>(std_msgs/Int32) --> DISP_NODE
    UINODE -- /tirgo/mission/start<br>(std_msgs/String) --> R_NODE
    DISP_NODE -- /tirgo/dispense/ready<br>(std_msgs/Bool) --> UINODE
    R_NODE -- /tirgo/tiago/arrived<br>(std_msgs/Bool) --> UINODE
    R_NODE -- /tirgo/tiago/picked<br>(std_msgs/Bool) --> UINODE
    R_NODE -- /tirgo/mission/finish<br>(std_msgs/String) --> UINODE
    UINODE -- /tirgo/ui/state --> R_NODE
    UINODE -- /tirgo/ui/error --> R_NODE
```

#### Topología de Red y Nodos

Todos los dispositivos operan bajo la misma subred local mediante conexión Wi-Fi, compartiendo el `ROS_MASTER_URI` alojado en el TIAGo.

  * **ROS Master (TIAGo):** Ejecuta el núcleo del sistema (`roscore`), el stack de navegación y los servidores de acción del brazo.
  * **Nodo Cliente de Hardware (Raspberry Pi):** Se conecta como un nodo ROS esclavo. Su función es exponer el hardware del dispensador (servos) al ecosistema ROS mediante tópicos, sin ejecutar lógica de navegación.
  * **Nodo Cliente de Interfaz (PC):** Ejecuta la aplicación web, se comunica con el dispensador y el robot y orquestra la interacción. Además, ejecuta el procesamiento de voz, enviando comandos de alto nivel al Master.

#### Estrategia de Navegación y Manipulación

  * **Aproximación:** Se confía en la precisión del stack de navegación estándar (`move_base` + `amcl`). La localización en el mapa estático se considera suficiente para posicionar al robot frente al dispensador dentro del margen de tolerancia del brazo.
  * **Manipulación Ciega (Open-Loop):** Al no utilizar reconocimiento visual activo para el agarre, la estrategia es determinista. El robot ejecuta una trayectoria pre-grabada (`play_motion`) asumiendo que, si la navegación fue exitosa, el bote de medicina se encuentra en las coordenadas $(x,y,z)$ fijas de la rampa de salida.

#### Lógica de Control: Estado Actual vs. Evolución

Actualmente, la orquestación reside en la capa de aplicación web (paradigma basado en Tópicos), con una migración planificada hacia Action Servers.

**Fase 1 (Arquitectura Actual - Basada en Topics):**
La web `tirgo_ui` actúa como orquestador central. Recibe la petición del usuario, consulta MongoDB (mediante `MONGO_URI`) para verificar stock y recetas, y registra la operación.
Si la validación es correcta, publica mensajes de alto nivel a ROS a través del módulo puente `rosio.py`:

  * `/tirgo/mission/start`: Inicia la secuencia.
  * `/tirgo/dispense/request`: Envía el `bin_id` (cajetín) necesario.

Los nodos de hardware (TIAGo, Raspberry Pi) ejecutan las acciones y publican flags de estado (`/tirgo/tiago/arrived`, `/tirgo/dispense/ready`, `/tirgo/tiago/picked`). La web se limita a escuchar estos flags para actualizar la barra de progreso de la interfaz.

**Fase 2 (Arquitectura Objetivo - Basada en Actions):**
Se implementará una separación estricta mediante ROS Actions. La web dejará de gestionar la lógica de bajo nivel y actuará como un Action Client.

  * Enviará una única meta ("Goal") al servidor `/tirgo/dispense_mission` (Ej: "Dispensar Medicamento X a Paciente Z").
  * Un nuevo nodo dedicado, `tirgo_mission_server`, implementará un `SimpleActionServer`. Este nodo gestionará la máquina de estados completa: coordinar la navegación, ordenar la apertura del dispensador, monitorizar los sensores y devolver feedback y resultado a la web. Esto encapsula la complejidad robótica dentro de ROS, dejando la UI solo para visualización.

#### Módulos Funcionales Principales

El sistema se estructura en módulos independientes para garantizar la escalabilidad y facilitar el mantenimiento. Se distingue claramente entre la capa de interacción (UI) y la capa de control robótico.

1.  **Módulo de Interfaz y Diálogo (UI Client):**

      * Nodo híbrido (`tirgo_ui`) que actúa como la cara visible del sistema.
      * Gestiona la interacción con el usuario (web interactiva y voz).
      * Integra el reconocimiento de voz (`stt_vosk`) para la activación mediante hotword.
      * Función: Recopilar la intención del usuario y validarla. No contiene lógica de navegación ni control de servos.

2.  **Módulo de Orquestación de Misiones (Mission Server):**

      * Nodo dedicado (`tirgo_mission_server`) que encapsula la inteligencia del robot.
      * Actúa como un Action Server que recibe metas de alto nivel de la UI.
      * Función: Implementar la máquina de estados central: coordina la navegación hacia el dispensador, solicita la apertura del cajetín, espera la confirmación hardware y ordena el movimiento del brazo.

3.  **Módulo de Gestión de Datos (Backend):**

      * Lógica de negocio conectada a MongoDB.
      * Función: Verificar en tiempo real si el paciente tiene receta activa y si hay stock suficiente antes de autorizar cualquier movimiento físico.

4.  **Módulo Dispensador (Hardware):**

      * Nodo ROS en la Raspberry Pi (`dispenser_node`).
      * Función: Suscribirse a peticiones de dispensación, accionar los servomotores y publicar confirmación (ACK) una vez finalizada la caída del medicamento.

5.  **Módulo Robótico (Ejecución):**

      * **Navegación:** Stack `move_base` estándar para desplazarse de forma autónoma por el mapa del hospital.
      * **Manipulación Ciega:** Estrategia de agarre basada en `play_motion`. Al llegar al punto de recogida, el robot asume que el bote está en una posición conocida (garantizada mecánicamente) y ejecuta una trayectoria pregrabada de Pick & Place sin necesidad de corrección visual activa ("Coger y ir").

### 2.b) Especificación de Componentes de Hardware

La selección de hardware prioriza la integración directa y la simplicidad de mantenimiento para este prototipo funcional.

| Componente | Especificación Técnica | Justificación y Diseño Electrónico |
| :--- | :--- | :--- |
| **Robot Base** | TIAGo++ (PAL Robotics) | Plataforma móvil con brazo manipulador. Actúa como servidor de cómputo principal (ROS Master). |
| **Controlador Dispensador** | Raspberry Pi 3b | **Interfaz GPIO Directa:** Se utiliza la librería RPi.GPIO para generar señales PWM por software directamente desde los pines de la placa, eliminando la necesidad de drivers externos y reduciendo la complejidad del cableado. |
| **Actuadores** | Servomotores SG90 (x2 Unidades) | **Alimentación Compartida:** Los servos se alimentan directamente desde el raíl de 5V de la Raspberry Pi. Dado el bajo consumo de los SG90 en carga intermitente, esta configuración simplifica la fuente de alimentación a una única entrada (cargador USB-C de la RPi), optimizando el espacio. |
| **Estructura Dispensador** | Impresión 3D | El dispensador opera en "lazo abierto". No dispone de sensores de caída (barrera IR). Se asume la fiabilidad mecánica del diseño de la tolva y el servo para garantizar la entrega del medicamento tras la orden de movimiento. |
| **Envases** | Botes cilíndricos | Geometría estandarizada para facilitar el agarre ciego con el gripper paralelo. |

### 2.c) Interfaz de Usuario (UI/UX) y Flujo de Interacción

La interfaz es una aplicación web (Flask) diseñada bajo criterios de diseño clínico: fondo blanco limpio para maximizar el contraste, tipografía de alta legibilidad y una paleta de colores basada en el azul corporativo ("TirGo Pharma"). La disposición de los elementos está optimizada para personas mayores, priorizando botones grandes y flujos lineales.

#### 1\. Arquitectura de Interacción (Voz + Manual)

El sistema integra dos modalidades de entrada que coexisten para mejorar la accesibilidad y la higiene hospitalaria:

  * **Modo Manos Libres (Voz):** La pantalla de bloqueo muestra un micrófono animado escuchando continuamente. Al pronunciar la hotword "Hola Tirgo", el sistema desbloquea el acceso al menú principal sin necesidad de contacto físico.
  * **Modo Manual (Cards):** El menú principal se organiza en tres grandes tarjetas ("Consultar", "Leer", "Diagnóstico"), cada una con una descripción clara y gran tamaño, facilitando el uso a pacientes mayores o con movilidad reducida.

#### 2\. Flujos de Usuario Detallados

**A. Opción "Leer" (Catálogo Visual y Disponibilidad)**

  * **Diseño en Rejilla:** Presenta el inventario como una cuadrícula de tarjetas. Cada tarjeta incluye:
      * Fotografía real del envase.
      * Nombre y dosis (ej: "Amoxicilina 500 mg").
      * Datos técnicos: Stock actual y Cajetín de origen (Bin ID).
  * **Etiquetado de Seguridad:** Badges visuales de color: Verde (L) para venta libre y Rojo (R) para receta obligatoria.
  * **Lógica de Stock:** El sistema filtra proactivamente la vista; si `stock == 0` en MongoDB, la tarjeta desaparece de la interfaz para evitar frustración en el usuario.
  * **Restricción por Receta:** Al pulsar "Pedir", si el fármaco es etiqueta (R), se fuerza una redirección al flujo de identificación. Si es (L), se permite la dispensación directa.

**B. Opción "Consultar" (Gestión de Pacientes)**

  * **Formulario de Identificación:** Solicita explícitamente Nombre, Apellidos y DNI.
  * **Privacidad (Hashing):** Antes de enviar la consulta, el DNI es procesado mediante un algoritmo de hash. La búsqueda en base de datos se realiza contra este hash, protegiendo la identidad real.
  * **Manejo de Errores (Usuario Nuevo):** Si el hash no se encuentra, el sistema muestra una pantalla de estado "Paciente no encontrado" y ofrece un botón de acción primario "Crear paciente", permitiendo el registro sin abandonar el flujo.
  * **Vista de Recetas:** Una vez autenticado, el usuario accede a una lista filtrada de sus recetas activas. Cada elemento muestra el medicamento prescrito, el stock disponible y un botón directo "Pedir este".

**C. Opción "Diagnóstico" (Sistema de Triaje Automatizado)**

  * **Wizard Secuencial:** Interfaz paso a paso con barra de progreso superior.
  * **Formularios Dinámicos:** Preguntas con selectores simples.
  * **Feedback de Resultados:**
      * **Derivación (Doctor):** Tarjeta de aviso clara indicando que los síntomas requieren valoración presencial.
      * **Consejo (Advice):** Pautas de autocuidado sin fármacos.
      * **Tratamiento (Med):** Recomendación de un fármaco específico disponible en el catálogo, con un botón para iniciar la dispensación inmediata.

#### 3\. Ejecución Técnica y Feedback Visual

Una vez confirmada una acción, la interfaz bloquea la entrada de datos y pasa a modo "Monitorización":

  * **Validación Atómica:** El backend comprueba por última vez Stock y Receta en la BD.
  * **Registro:** Se genera una entrada inmutable en la colección `dispenses`.
  * **Comunicación ROS:** Se publican los tópicos de misión (`/tirgo/mission/start`).
  * **Estado en Tiempo Real:** La web muestra notificaciones ("Robot en camino", "Dispensando...") que reaccionan a los mensajes de retorno del hardware (`/tirgo/tiago/picked`, `/tirgo/dispense/ready`).

-----

## 3\. Diseño de Software y Comunicación

### 3.a) Arquitectura de Nodos en ROS 1

El sistema se fundamenta en una arquitectura híbrida que desacopla la lógica de negocio (Web/Datos) de la ejecución robótica (ROS). El nodo principal de interacción es `tirgo_ui`, el cual actúa como un puente bidireccional.

#### 1\. Estrategia de Integración Flask-ROS (`tirgo_ui`)

El nodo `tirgo_ui` no sigue el patrón estándar de ejecución cíclica, sino que integra un servidor web asíncrono.

  * **Gestión de Concurrencia:** Para evitar condiciones de carrera sin bloquear el servidor web, la comunicación entre el hilo de ROS (callbacks) y el hilo principal de Flask se realiza mediante variables de estado compartidas. Dado que los datos intercambiados son banderas booleanas atómicas (ej. `arrived`, `ready`) o cadenas cortas, este enfoque minimiza la latencia frente al uso de colas complejas.
  * **Separación de Responsabilidades:**
      * `app.py` / `leer.py` (Lógica): Gestionan el acceso a MongoDB, validan reglas de negocio (recetas, stock) y deciden cuándo actuar. Nunca tocan ROS directamente.
      * `rosio.py` (Pasarela): Actúa como Gateway. No tiene acceso a la base de datos. Su única función es traducir las decisiones de la web a mensajes ROS y mantener actualizado el estado de los sensores para la UI.

#### 2\. Especificación de Interfaces ROS

El diseño conceptual del sistema para el Hito 2 define tres tipos de interfaces. Aunque la implementación actual se basa en Tópicos para la validación funcional, la arquitectura contempla la inclusión de Servicios y Acciones para robustecer el flujo.

**(A) Tópicos (Implementados - Comunicación Asíncrona)**
La comunicación es ligera; no se transmiten estructuras de datos complejas (JSON de pacientes) por la red ROS, solo comandos de ejecución y telemetría.

| Dirección | Tópico | Tipo Mensaje | Payload (Ejemplo) | Descripción Técnica |
| :--- | :--- | :--- | :--- | :--- |
| Sub | `stt/text` | `std_msgs/String` | `"hola tirgo"` | Flujo de audio procesado externamente. Si coincide con la hotword, activa la UI. |
| Pub | `/tirgo/mission/start` | `std_msgs/String` | `"go_pickup"` | **Trigger de Misión:** Comando simbólico. Indica al robot que inicie la secuencia de navegación hacia el dispensador. |
| Pub | `/tirgo/dispense/request` | `std_msgs/Int32` | `2` | Envía únicamente el `bin_id` (entero) que la Raspberry Pi debe accionar. |
| Pub | `/tirgo/ui/state` | `std_msgs/String` | `"WAITING_USER"` | Telemetría para herramientas de depuración externas. |
| Sub | `/tirgo/dispense/ready` | `std_msgs/Bool` | `True` | Confirmación física de que el servo ha completado su recorrido. |
| Sub | `/tirgo/tiago/arrived` | `std_msgs/Bool` | `True` | Confirmación de navegación completado. |

**(B) Acciones (Diseño - Tareas de Larga Duración)**
Se establece la transición de la lógica de navegación actual (basada en tópicos) a un Action Server:

  * `/tirgo/orchestrate_mission` (`tirgo_msgs/Mission.action`): Permitirá a la web enviar un objetivo y recibir feedback continuo y un estado final (SUCCEEDED / ABORTED), gestionando internamente los reintentos de navegación.

#### 3\. Configuración del Entorno de Despliegue (roslaunch)

La arquitectura modular se refleja en la estrategia de arranque. Actualmente, el subsistema de interfaz se despliega de forma aislada mediante `web.launch`, que inyecta la configuración necesaria mediante variables de entorno, desacoplando el código de la infraestructura:

  * `MONGO_URI`: Cadena de conexión segura a la base de datos (ej: `mongodb://tirgoAdmin:pass@localhost...`).
  * `TIRGO_HOTWORD`: Palabra clave personalizable para la activación por voz.
  * `TIRGO_STT_TOPIC`: Permite remapear el tópico de entrada de audio sin recompilar.
  * `TIRGO_DEV`: Flag crítica que permite ejecutar la interfaz en modo "Mock", simulando respuestas del robot para desarrollo sin hardware.

Se prevé la creación de un lanzador maestro (`tirgo_bringup.launch`) que orqueste la carga simultánea de la navegación del TIAGo, el controlador de la Raspberry Pi y este servidor web.

### 3.b) Estructura del Repositorio y Gestión de Paquetes

El proyecto se gestiona mediante un Monorepositorio (Monorepo) que agrupa todos los paquetes ROS del sistema. Esta estrategia facilita la integración continua y el versionado atómico del código fuente.

### **1.0 Árbol de Directorios del Proyecto**

La estructura general del proyecto TirgoPharma sigue una separación clara entre:

* **Código ROS** (dentro del workspace `ros_ws/`, ubicado en la carpeta compartida del host para integrarse con Docker y la Raspberry Pi).
* **Infraestructura** (servicios de base de datos, inicialización y orquestación).
* **Recursos pesados** (modelos de voz, mapas, logs u otros ficheros que no se versionan).
* **Capa de contenedores** (Dockerfile y `docker-compose.yml` a nivel raíz).

Este enfoque garantiza mantenibilidad, despliegues reproducibles y una clara separación de responsabilidades entre desarrollo, infraestructura y runtime.

---

#### **1.1 Árbol del Repositorio Principal `TirGo/`**

El repositorio agrupa todo lo necesario para ejecutar TirgoPharma:
infraestructura (MongoDB), contenedores, y una carpeta compartida utilizada por los servicios (incluyendo el workspace ROS).

**Descripción breve**

* **carpeta_compartida/**
  Espacio accesible desde los contenedores (volumen). Contiene `ros_ws/` y los modelos de voz.
* **infra/**
  Infraestructura asociada al sistema: stack de MongoDB con inicialización automática.
* **Dockerfile / docker-compose.yml**
  Base para construir la imagen principal y definir los servicios del sistema.

**Árbol**

```text
TirGo/
├── carpeta_compartida/                 # Volumen compartido con los contenedores
│   ├── ros_ws/                         # Workspace ROS (código principal)
│   └── stt_models/                     # Modelos de voz Vosk y otros recursos pesados
│
├── infra/                              # Infraestructura del sistema
│   └── tirgo_db_stack/
│       ├── docker-compose.yml          # MongoDB + Mongo Express
│       └── mongo-init/
│           └── 001-init.js             # Inicialización de la base de datos
│
├── Dockerfile                          # Imagen principal (ROS + TirgoPharma)
├── docker-compose.yml                  # Orquestación de contenedores
└── .gitignore                          # Exclusiones (modelos, binarios, etc.)
```

---

#### **1.2 Árbol del Workspace ROS (`ros_ws/`)**

**Descripción breve**

El workspace sigue el estándar **Catkin**, organizando cada funcionalidad en un paquete independiente:

* **tirgo_msgs** → define las interfaces (acciones y servicios).
* **tirgo_ui** → la interfaz web + lógica de negocio + puente con ROS.
* **stt_vosk** → motor de reconocimiento de voz offline.
* **tiago_pharma_dispenser** → nodo de control de servos en la Raspberry.
* **tiago_pharma_bringup** → orquestación general del sistema.
* **move** → mapas, RViz y herramientas de navegación.

Esta separación modular evita dependencias circulares, facilita el testing y permite desplegar cada parte en el entorno adecuado (TIAGo, Raspberry Pi, servidor web).

**Árbol**

```text
ros_ws/
├── src/
│   ├── tirgo_msgs/                     # [N] Definición de Interfaces (evita dependencias circulares)
│   │   ├── action/
│   │   │   └── Mission.action          # Definición de la meta de dispensación
│   │   └── srv/
│   │       └── SystemCheck.srv         # Servicio de verificación de salud
│   │
│   ├── tirgo_ui/                       # Nodo híbrido Web + ROS
│   │   ├── launch/
│   │   │   └── web.launch              # Carga variables de entorno y lanza el servidor
│   │   ├── scripts/
│   │   │   └── tirgo_web_server        # Ejecutable (Entrypoint)
│   │   ├── src/tirgo_ui/               # Python Module
│   │   │   ├── app.py                  # Factory de Flask
│   │   │   ├── rosio.py                # Hilo de comunicación ROS (Pubs/Subs)
│   │   │   ├── storage_mongo.py        # Capa de acceso a Datos
│   │   │   ├── routes/                 # Blueprints (Modularización de vistas)
│   │   │   │   ├── main.py
│   │   │   │   ├── leer.py             # Lógica de dispensación
│   │   │   │   └── ...
│   │   │   └── ...
│   │   ├── templates/                  # Frontend: HTML (Jinja2)
│   │   └── static/                     # Frontend: CSS, JS y Assets
│   │
│   ├── stt_vosk/                       # Motor de Voz
│   │   ├── launch/
│   │   └── model/                      # (Ignorado en Git) montado desde `carpeta_compartida/stt_models`
│   │
│   ├── tiago_pharma_dispenser/         # Control Hardware Raspberry Pi
│   │   └── scripts/
│   │       └── servo_node.py           # Generación PWM
│   │
│   ├── tiago_pharma_bringup/           # Orquestación
│   │   └── launch/
│   │       └── tirgo_system.launch     # Arranque completo del sistema
│   │
│   └── move/                           # Paquete de navegación / visualización de mapas
│       ├── configs/
│       │   └── rviz_configs.rviz       # Configuración de RViz
│       ├── launch/
│       │   ├── README.md               # Notas de uso
│       │   └── rviz.launch             # Lanzador de RViz con el mapa del aula
│       ├── maps/                       # Mapas de navegación del aula
│       │   ├── Mapa_aula.pgm
│       │   ├── Mapa_aula.yaml
│       │   ├── Mapa_aula_mod_1.0.pgm
│       │   ├── Mapa_aula_mod_1.0.xcf   # Versión editable del mapa
│       │   └── Mapa_aula_mod_1.0.yaml
│       ├── scripts/
│       │   ├── README.md               # Descripción de scripts auxiliares
│       │   └── run_all.sh              # Script de arranque rápido (mapa + RViz, etc.)
│       ├── src/
│       │   └── move/                   # Código fuente del paquete
│       ├── CMakeLists.txt              # Configuración de build Catkin
│       ├── README.md                   # Descripción general del paquete move
│       ├── notas.txt                   # Notas de desarrollo / pruebas
│       └── package.xml                 # Metadatos del paquete ROS
│
└── CMakeLists.txt                      # Top-level de catkin (si aplica)
```


#### 2\. Descripción de Paquetes Principales (Completo)

El repositorio está organizado en un Monorepo con paquetes ROS que encapsulan las distintas capas funcionales del sistema (Interfaz, Lógica de Control, Hardware e Interfaces Comunes).

1.  **Paquetes de Interfaz y Lógica**

      * **`tirgo_ui`**:
          * *Función:* Es el paquete más complejo del sistema, actuando como el nodo híbrido que aloja la interfaz de usuario (Flask) y el puente ROS.
          * *Arquitectura:* Implementa una arquitectura modular que separa el Frontend (`templates`/`static`) de la Lógica de Negocio (`scripts/`, `routes/`, `storage_mongo.py`) y la Capa de Enlace ROS (`rosio.py`).
          * *Punto de Entrada:* El script `tirgo_web_server` actúa como punto de entrada ROS, levantando el servidor Flask y el nodo ROS simultáneamente.
      * **`stt_vosk`**:
          * *Función:* Encapsula el motor de reconocimiento de voz offline (vosk) necesario para la activación por hotword y la interacción conversacional.
          * *Gestión de Assets:* El modelo de lenguaje está excluido del repositorio mediante `.gitignore` para mantener el historial ligero. Su descarga y colocación se automatiza dentro del `Dockerfile` durante la construcción de la imagen.

2.  **Paquetes de Control y Ejecución**

      * **`move`**:
          * *Función:* Módulo que implementa el flujo completo de navegación básica y visualización del robot TIAGo. Es esencial para la fase de navegación del proyecto.
          * *Lógica de Navegación:* Contiene el nodo de lógica de navegación, que es un cliente Python encargado de enviar secuencias de coordenadas al sistema `move_base`. Este nodo se encarga de monitorear la posición actual (`/robot_pose`) para determinar si el robot ha iniciado el movimiento y ha llegado a su destino.
          * *Visualización:* Incluye el lanzador `rviz.launch`, que inicializa la herramienta de visualización RViz cargando un perfil de configuración (`rviz_configs.rviz`) predefinido para ver el mapa, el robot y los datos de los sensores.
          * *Orquestación Rápida:* Utiliza el script orquestador `run_all.sh` para automatizar la carga secuencial de RViz, map\_server y el nodo de lógica de navegación, gestionando los tiempos de espera entre procesos.
      * **`tiago_pharma_dispenser`**:
          * *Función:* Contiene el driver de bajo nivel y la lógica de control del hardware del dispensador (Raspberry Pi).
          * *Control:* Aloja el script Python (`servo_node.py`) encargado de recibir las órdenes ROS y generar las señales PWM a través de los pines GPIO para accionar los servomotores de las cubetas de medicamentos.

3.  **Paquetes de Infraestructura (Todavía no implementado)**

      * **`tirgo_msgs`**:
          * *Función:* Paquete de metapaquetes que contiene únicamente la definición de interfaces (`.msg`, `.srv`, y `.action`) para todo el workspace.
          * *Justificación Técnica:* Crear este paquete independiente es una *Best Practice* que rompe las dependencias circulares. Permite que paquetes dispares como `tirgo_ui` (web) y `move` (navegación) importen definiciones como la futura `Mission.action` sin obligar a compilar todo el stack de la web en el robot.
      * **`tiago_pharma_bringup`**:
          * *Función:* Contiene los archivos de lanzamiento (`launch`) globales y maestros (`tirgo_system.launch`) que se encargarán de arrancar el sistema completo (TIAGo + RPi + Web + STT) con un único comando.

#### 3\. Estrategia de Ramas (Branching Strategy)

Se ha adoptado una metodología basada en **Gitflow simplificado** para garantizar la estabilidad del código en un entorno colaborativo:

  * **`main` (Protected):** Rama de producción. Contiene únicamente código validado y funcional. Cada merge a esta rama representa una versión desplegable ("Release Candidate").
  * **`develop` (Protected):** Rama de integración continua. Aquí convergen los cambios de todos los desarrolladores. Es el punto de sincronización para probar la compatibilidad entre módulos (ej. UI con Dispensador) antes de pasar a producción.
  * **`feature/*` (Efímeras):** Ramas de trabajo individual. Se crean desde `develop` y se integran mediante Pull Requests (PR) tras revisión de código.

### 3.c) Descripción de Contenedores Docker y Dependencias

Para garantizar la reproducibilidad del entorno de desarrollo, el sistema se despliega mediante contenedores Docker orquestados. La infraestructura se divide en dos servicios lógicos diferenciados:

#### 1\. Arquitectura de Contenedores

**A. Stack de Datos**
Se despliegan servicios estándar para la gestión de información, desacoplados de la lógica del robot para facilitar actualizaciones independientes.

  * **`mongo`:** Base de datos. Se configura un Volumen Docker persistente (`/data/db`) para garantizar que los registros de pacientes y el histórico de dispensaciones sobrevivan al reinicio o destrucción de los contenedores.
  * **`mongo-express`:** Interfaz gráfica ligera basada en web para la administración visual de la base de datos, crucial para depuración rápida de recetas y stock durante el desarrollo.

**B. Stack Robótico y de Aplicación (Core)**
Es un contenedor `ros1_rob_tirgo` con `network_mode: host`. Permite ejecutar el entorno Noetic completo en ordenadores de desarrollo, compartiendo red transparente con el robot y la BD local.

#### 2\. Gestión de Dependencias del Entorno

Todas las dependencias se definen explícitamente en el `Dockerfile` para asegurar que cualquier miembro del equipo pueda replicar el entorno exacto con un solo comando (`docker build`).

  * **Dependencias de Sistema (APT):** Sobre la imagen base `ros:noetic-ros-base`, se instalan:
      * Paquetes de compilación básicos (`build-essential`, `git`).
      * Drivers de audio y sistema (`libasound2`, `portaudio19-dev`) necesarios para la captura de micrófono y el motor de voz.
  * **Dependencias de Python (PIP):** Se gestionan mediante un archivo `requirements.txt` que incluye:
      * Core Web: `Flask`, `Flask-Cors` (para la interfaz de usuario).
      * Datos: `pymongo` (driver oficial de MongoDB).
      * IA/Voz: `vosk` (motor de reconocimiento de voz offline) y `sounddevice`.
      * Hardware: `RPi.GPIO` (solo se instala condicionalmente o se simula en entornos no-Raspberry mediante mocking).

-----

## 4\. Análisis de Viabilidad Técnica

### 4.a) Identificación de Limitaciones y Riesgos

El siguiente análisis de riesgos evalúa los puntos críticos del diseño conceptual, enfocándose en las implicaciones de las decisiones de hardware (control en lazo abierto, alimentación directa) y software (seguridad de datos).

| Riesgo o Limitación | Impacto | Estrategia de Mitigación y Justificación |
| :--- | :--- | :--- |
| **Seguridad y Datos:** Acceso no autorizado a datos sensibles (DNI) o a la base de datos MongoDB. | Alto | **Hashing de DNI** para proteger la identidad del paciente en la BD. El acceso a la gestión de la base de datos (Mongo Express) está protegido por credenciales de administrador y el contenedor se mantiene apagado por defecto, encendiéndose solo para tareas específicas de mantenimiento. |
| **Control Mecánico:** Fallos más críticos derivan de la caída descentrada del bote en la tolva, que provoca empujes fuera de eje y posibles bloqueos según su orientación y carga. Además, la deriva posicional de los pushers altera su fuerza y recorrido efectivo, comprometiendo la fiabilidad del ciclo de dispensación. | Alto | La centración del bote puede garantizarse mediante guías mecánicas e unha entrada que asegure su alineamiento en la tolva. Para evitar bloqueos, pueden integrarse sensores de presencia/atasco y superficies internas optimizadas que mejoren el deslizamiento. La deriva de los pushers se corrige mediante un anclaje mecánico más robusto y una calibración periódica que restaure su posición de referencia. |
| **Manipulación (Lazo Abierto):** Falso Positivo: El bote no cae o el robot agarra aire (el dispensador no tiene sensores de confirmación de caída). | Alto | **Sistema de Validación del Gripper:** El robot utiliza sus sensores de fuerza (monitorización de corriente). El sistema solo avanza si se detecta la fuerza de agarre correcta, actuando como un sensor de contacto final. |
| **Gestión de Potencia:** Inestabilidad eléctrica de la Raspberry Pi por picos de corriente del servo. | Bajo | **Control Secuencial (Diseño):** Los servomotores nunca se moverán de forma simultánea, ya que los botes se dispensan uno en cada petición, mitigando el riesgo de caídas de tensión por picos. |
| **Navegación:** Desviación Posicional ubicacicion y error de motores. | Medio | **Precisión de Navegación:** Método de calibración al ejecutar el robot que lo ubica en el mapa y asegura una buena calibracion respecto a la sala. Además el robot trabaja con muchos decimales porloque con un filtro paso bajo se puede asegurar evitar ruido.|

#### Otros Riesgos y Limitaciones Clave

Adicionalmente, se identifican las siguientes limitaciones y riesgos que requieren consideración en la planificación y alcance futuro:

  * **Riesgo de seguridad en la capa de tránsito:** La comunicación en ROS 1 se realiza sin cifrado ni autenticación. En la memoria final se incluirán recomendaciones y consejos para mitigar este riesgo aún utilizando la arquitectura ROS 1.
  * **Dependencia del personal sanitario para la carga:** El error humano en la carga de los medicamentos en el bin incorrecto es un riesgo inherente al punto de entrada del sistema.
  * **Ausencia de trazabilidad en fallo de misión:** En la arquitectura actual basada en Tópicos, si la misión se interrumpe (ej. por fallo de red), no existe una capa persistente para registrar o reanudar el trabajo pendiente.

### 4.b) Estrategia de Mitigación y Pruebas Iniciales

La estrategia de mitigación y pruebas iniciales se centra en la validación empírica de los mecanismos de mitigación de lazo abierto y en la trazabilidad de las operaciones críticas de dispensación y seguridad.

#### 1\. Pruebas de Ejecución y Mecánica

El foco es validar la repetibilidad de los subsistemas, ya que el agarre ciego depende de la confiabilidad mecánica.

| Riesgo a Mitigar | Estrategia de Prueba | Criterio de Éxito |
| :--- | :--- | :--- |
| **Atasco y Fricción (Control Mecánico)** | **Prueba de Estrés Secuencial:** 10 ciclos completos y consecutivos de dispensación para cada bin (sin fallos por interferencia entre servos). | El sistema debe superar 8 de 10 ciclos con dispensación y recogida exitosa ($80\%$ de repetibilidad). |
| **Desviación Posicional (Navegación)** | **Calibración respecto al mapa:** Al comenzar el proceso el robot girará un par de veces sobre sí mismo buscando reanalizar la sala con sus sensores ubicandose en el mapa. | El error posicional se reduce en gran manera al estar calibrado. |

#### 2\. Validación de Potencia y Control

  * **Prueba de Estabilidad Eléctrica:** El riesgo de inestabilidad eléctrica de la Raspberry Pi ha sido mitigado mediante el **Control Secuencial de Servos**. Este diseño se ha validado empíricamente realizando 20 dispensaciones consecutivas, con la RPi manteniendo una operación estable sin reinicios.
  * **Control del Movimiento:** Para mitigar el riesgo de temblor por PWM de software, el servo está físicamente anclado dentro del dispensador impreso en 3D, lo que impide cualquier movimiento no deseado fuera de su eje de rotación.

#### 3\. Estrategia de Seguridad y Trazabilidad

  * **Software (Mocking):** Se mantendrá el uso de la variable de entorno `TIRGO_DEV=1` para simular respuestas del hardware y validar la lógica de la UI y la conexión a Mongo sin necesidad de robot físico, acelerando la depuración.
  * **Seguridad de la Capa de Tránsito (ROS 1):** Para mitigar el riesgo de seguridad en la capa de tránsito ROS 1, la memoria final incluirá un plan de recomendaciones de seguridad de infraestructura, basándose en las medidas aprendidas en el curso de Ciberseguridad. Estas medidas incluyen:
      * Uso de VPNs para cifrar y tunelizar el tráfico ROS.
      * Implementación de reglas de Firewall en los dispositivos de host (TIAGo y RPi) para restringir el acceso a puertos ROS.
      * Segregación de la red mediante Wi-Fi privada o VLANs para aislar el tráfico de control del robot.

-----

## 5\. Cronograma de Desarrollo

```mermaid
---
config:
  theme: neutral
---
gantt
    title Cronograma TirgoPharma · Fase Final
    dateFormat  YYYY-MM-DD
    axisFormat  %d-%m
    excludes    weekends
    todayMarker stroke-width:2,stroke:blue,opacity:0.6
    section Fase 1 · Hardening y Baja Carga (Sem 1)
    A1 Navegación desde punto inicial a final     :       f1a, 2025-11-17, 7d
    B1 Terminar paquete dispensador               :       f1b, 2025-11-17, 7d
    C1 Inicio Action Server Core                  :       f1c, 2025-11-17, 7d
    section Fase 2 · Manipulación y Acción (Sem 2)
    A2 Manipulación medicamentos (play_motion)    :       f2a, after f1a, 7d
    B2 Integración final RPi dispensador          :       f2b, after f1c, 7d
    C2 Mission Server (tirgo_mission_server)      :       f2c, after f1b, 7d
    section Fase 3 · Integración y Validación (Sem 3)
    A3 Action Server funcionando                  :       f3a, 2025-12-01, 7d
    B3 Pruebas funcionamiento                     :       f3b, 2025-12-01, 10d
    C3 Action Server sustituye tópicos            :       f3c, 2025-12-01, 7d
    section Fase 4 · Entrega Final y Cierre (Sem 4)
    Code freeze [TODOS]                        :       f4a, 2025-12-08, 7d
    Documentación final (memoria + plan)       :       f4b, 2025-12-08, 7d
    Demo final (vídeo + launch despliegue)     :       f4c, 2025-12-08, 7d
```

### 5.a) Plan Temporal

El presente cronograma detalla la planificación de las actividades restantes del proyecto TirGo Pharma, abarcando un periodo de cuatro semanas hasta la entrega final. Se estructura en fases secuenciales que priorizan la estabilidad del hardware y la lógica de control, culminando en una fase intensiva de pruebas e integración.

*Figura 1: Cronograma de Desarrollo del Proyecto TirGo Pharma*

**Fase 1: Hardening y Baja Carga (Semana 1)**
Esta fase se centra en consolidar los componentes de bajo nivel y establecer las bases de la nueva arquitectura de control.

  * **A1: Navegación desde punto inicial a final:** Puesta a punto final del stack de navegación autónoma del TIAGo (`move_base`, `amcl`). El objetivo es garantizar que el robot puede desplazarse de forma fiable desde su posición de inicio hasta el punto de recogida del dispensador.
  * **B1: Terminar paquete dispensador:** Finalización del nodo `tiago_pharma_dispenser` en la Raspberry Pi. Esto incluye el ajuste fino del control de los servomotores (PWM) para asegurar movimientos precisos y repetibles al dispensar los medicamentos.
  * **C1: Inicio Action Server Core:** Definición de las interfaces de acción en el paquete `tirgo_msgs` (especialmente `Mission.action`) y el esqueleto del `tirgo_mission_server` (Action Server) que orquestará las tareas complejas.

**Fase 2: Manipulación y Acción (Semana 2)**
Una vez estabilizados los componentes básicos, esta fase se enfoca en la interacción del robot con el entorno físico y la lógica central de la misión.

  * **A2: Manipulación medicamentos (`play_motion`):** Programación y ajuste de la trayectoria pregrabada de manipulación (`play_motion`) para que el gripper del TIAGo pueda agarrar el bote de medicamento de la cuna de recepción de forma fiable y repetitiva.
  * **B2: Integración final RPi dispensador:** Integración completa del nodo `tiago_pharma_dispenser` con el ROS Master del TIAGo, asegurando una comunicación robusta y bidireccional entre el robot y el hardware del dispensador.
  * **C2: Mission Server funcionando:** Implementación de la lógica de la máquina de estados dentro del `tirgo_mission_server` (Action Server). Este módulo será el "cerebro" que coordine las secuencias de navegación, dispensación y manipulación.

**Fase 3: Integración y Validación (Semana 3)**
Esta es la fase crítica de consolidación, donde todos los componentes se unen y se someten a pruebas rigurosas.

  * **A3: Action Server funcionando:** Verificación de que el Action Server gestiona correctamente la comunicación (Goal/Feedback/Result) con la UI y los demás nodos, reemplazando la lógica anterior basada en tópicos.
  * **B3: Pruebas funcionamiento:** Ejecución de pruebas de funcionamiento de extremo a extremo (E2E) y pruebas de estrés intensivas. Se realizarán 10 ciclos completos de dispensación y recogida, con un criterio de éxito de al menos 8 ciclos completados con éxito.
  * **C3: Action Server sustituye tópicos:** Confirmación final de que la lógica basada en Action Servers ha reemplazado completamente la anterior implementación mediante tópicos para las misiones principales.

**Fase 4: Entrega Final y Cierre (Semana 4)**
La última fase se dedica a la finalización de la documentación, la preparación de los materiales de entrega y el "congelamiento" del código.

  * **Code freeze [TODOS]:** Congelación del código. A partir de este punto, solo se permitirán correcciones de bugs críticos, enfocándose en la estabilidad y no en la adición de nuevas funcionalidades.
  * **Documentación final (memoria + plan):** Redacción y revisión final de la memoria del proyecto, incluyendo el plan de seguridad específico para ROS 1 y cualquier manual de usuario o guía técnica.
  * **Demo final (video + launch despliegue):** Preparación del material de la demo (grabación de vídeo del sistema funcionando) y creación del archivo roslaunch maestro para el despliegue sencillo y reproducible del sistema completo.

### 5.b) Reparto de Responsabilidades Actualizado

  * **Responsable de navegación y recogida - Ángel Romero:**
      * Lidera la navegación autónoma y la programación de la manipulación ciega (`play_motion`).
  * **Responsable de dispensador físico y electrónica - Lucas Campillo:**
      * Lidera la finalización del control PWM en la Raspberry Pi y la calibración mecánica del dispensador.
  * **Responsable de software y HRI - Katrin Muñoz:**
      * Lidera la sustitución de la lógica de Tópicos por Actions, la implementación de los Action Servers y la integración del Backend/UI con el Action Server.
