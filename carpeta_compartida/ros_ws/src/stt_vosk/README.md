<div align="center">

# stt_vosk

Nodo de reconocimiento de voz (Speech-To-Text) para ROS 1 (Noetic),
basado en Vosk y PyAudio.

Convierte audio capturado por micrófono en texto publicable por ROS,
permitiendo interacción por voz con el sistema TirGoPharma.

Diseñado para integrarse principalmente con la interfaz web `tirgo_ui`.

</div>

---

## Visión general

El paquete `stt_vosk` implementa un nodo ROS que escucha audio en tiempo real,
ejecuta reconocimiento de voz en streaming y publica el texto reconocido en topics ROS.

Su objetivo es servir como módulo de entrada por voz, manteniéndose totalmente
desacoplado de la lógica de negocio.

Características principales:

- Reconocimiento de voz offline (sin servicios externos).
- Uso de modelos Vosk intercambiables por idioma.
- Publicación de:
  - texto final
  - texto parcial (opcional)
- Soporte de wake word.
- Configuración flexible vía parámetros ROS.
- Integración directa con `tirgo_ui`.

---

## 1. Estructura del paquete

```text
stt_vosk/
├── package.xml
├── CMakeLists.txt
├── README.md
├── launch/
│   └── stt_vosk.launch           # Launch recomendado del nodo STT
├── scripts/
│   ├── stt_vosk_node.py          # Nodo principal (Vosk + PyAudio + ROS)
│   └── list_audio_devices.py     # Utilidad para listar micrófonos
└── tests/
    └── test_stt_vosk_node.py     # Tests unitarios (wake word, parciales, errores)
````

---

## 2. Qué hace el nodo

El nodo `stt_vosk_node.py` ejecuta el siguiente flujo:

1. Abre un dispositivo de audio (micrófono) usando PyAudio.
2. Carga un modelo Vosk desde disco.
3. Procesa el audio en streaming.
4. Publica el texto final reconocido en `stt/text`.
5. (Opcional) Publica texto parcial en `stt/partial`.
6. (Opcional) Detecta una wake word y emite un evento especial.

---

## 3. Rol dentro de TirGoPharma

Dentro del sistema global, `stt_vosk` actúa como módulo de entrada por voz:

* Publica texto reconocido en ROS.
* No interpreta comandos.
* No ejecuta acciones.
* No depende de la UI.

La interpretación del texto queda en manos de otros módulos, principalmente `tirgo_ui`.

Este diseño permite:

* Activar o desactivar voz sin romper el sistema.
* Reemplazar STT sin tocar la lógica.
* Probar el sistema sin micrófono.

---

## 4. Integración con `tirgo_ui` (wake word)

### Contrato de mensajes (importante)

Si se configura el parámetro `~wake_word`:

* Cuando el texto final reconocido contiene esa frase:

  * El nodo publica `"__WAKE__"` en `stt/text`
  * A continuación, publica también el texto completo reconocido

Ejemplo de publicación:

```text
stt/text: "__WAKE__"
stt/text: "hola tirgo empieza la misión"
```

### Reglas para consumidores (`tirgo_ui`)

* `__WAKE__` no es texto natural.
* Debe tratarse como un evento (trigger).
* El texto posterior contiene el contenido real del usuario.

Nota: esta convención está validada por tests y forma parte del contrato del nodo.

---

## 5. Dependencias

### ROS

* ROS 1 Noetic
* `rospy`
* `std_msgs`

### Python

* `vosk`
* `pyaudio`

### Sistema (Ubuntu)

* `portaudio19-dev` (necesario para PyAudio)

### Modelo

* Modelo Vosk descargado localmente (ejemplo: `vosk-model-small-es-0.42`)

---

## 6. Instalación



```bash
sudo apt install portaudio19-dev
pip install vosk pyaudio
```

---

## 7. Modelo Vosk

### Ruta recomendada (por defecto en el nodo)

```text
~/models/vosk-es-small
```

Ejemplo de descarga:

```bash
mkdir -p ~/models
cd ~/models
wget https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip
unzip vosk-model-small-es-0.42.zip
mv vosk-model-small-es-0.42 vosk-es-small
```

---

## 8. Uso rápido

### 8.1 Listar dispositivos de audio

Para identificar el índice del micrófono disponible:

```bash
rosrun stt_vosk list_audio_devices.py
```

Ejemplo de salida:

```text
[0] Built-in Audio | rate max: 48000 Hz | channels: 2
[9] USB Audio     | rate max: 16000 Hz | channels: 1
```

Guarda el índice (`device_index`) del micrófono correcto.

### 8.2 Sanity check recomendado

Antes de integrar con la UI, verifica que se publica texto:

```bash
rostopic echo /stt/text
```

Luego ejecuta el nodo (ver siguiente sección) y habla. Deberías ver mensajes.

Si usas wake word, prueba explícitamente que aparece `__WAKE__`.

---

## 9. Launch recomendado

Este launch evita hardcodear rutas e índices que suelen fallar entre máquinas.

Ejecutar:

```bash
roslaunch stt_vosk stt_vosk.launch device_index:=9
```

---

## 10. Parámetros del nodo

| Parámetro          | Tipo   | Descripción                                |
| ------------------ | ------ | ------------------------------------------ |
| `~model_path`      | string | Ruta al modelo Vosk                        |
| `~sample_rate`     | int    | Frecuencia de muestreo (16000 recomendado) |
| `~device_index`    | int    | Índice del micrófono                       |
| `~chunk`           | int    | Tamaño del bloque de audio (default: 8000) |
| `~publish_partial` | bool   | Publica texto parcial en `stt/partial`     |
| `~wake_word`       | string | Frase clave para generar `__WAKE__`        |

---

## 11. Topics ROS

### Publica

* `stt/text` (`std_msgs/String`)
  Texto final reconocido o evento `__WAKE__`

* `stt/partial` (`std_msgs/String`)
  Texto parcial mientras el usuario habla (opcional)

### Suscribe

* Ninguno

---

## 12. Tests

El paquete incluye tests unitarios:

```bash
pytest -q
```

Cubren:

* Detección de wake word.
* Publicación única de `__WAKE__`.
* Activación o desactivación de parciales.
* Error limpio si el modelo no existe.

---

## 13. Resumen

* `stt_vosk` proporciona STT offline y desacoplado para TirGoPharma.
* Publica texto en ROS de forma simple y testeada.
* La lógica y decisiones pertenecen a otros módulos (principalmente `tirgo_ui`).
* Permite que el sistema escuche sin imponer qué debe hacer con lo que oye.
