## Carpeta Move 
````markdown

# Módulo `move` (Python) – Nodos de Navegación

Este directorio contiene el código fuente en Python que implementa la lógica de navegación, secuenciación de objetivos y pruebas del robot TIAGo.

---

## 1. Archivos del Paquete

```text
src/move/
├── checkpointfollower.py   # Nodo principal de navegación
├── publish_initial_pose.py # Herramienta de localización
├── comunicacion_test.py    # Test de conectividad
└── test_puntos.py          # Validación de coordenadas
````

-----

## 2\. Descripción de los Nodos

### 2.1. `checkpointfollower.py`

Es el **nodo principal** ("Director de Misión"). Su función es guiar al robot a través de una ruta preestablecida.

  * **Funcionamiento:**
    1.  Lee una lista de coordenadas (checkpoints) definida en el código.
    2.  Envía cada punto como un objetivo (`goal`) al topic `/move_base/goal`.
    3.  Monitoriza el estado del robot y espera a que llegue al destino antes de enviar el siguiente punto.
  * **Mensajes clave:** `move_base_msgs/MoveBaseActionGoal`.

### 2.2. `publish_initial_pose.py`

Un script auxiliar para ayudar al sistema de localización (`amcl`) a situarse.

  * **Funcionamiento:** Publica una estimación de la posición inicial del robot en el topic `/initialpose`. Esto es útil para "resetear" la ubicación del robot en el mapa al inicio de la ejecución sin usar la interfaz gráfica de RViz.

### 2.3. `test_puntos.py`

Script de validación de datos.

  * **Uso:** Se utiliza para verificar que las listas de coordenadas (x, y, orientación) tienen el formato correcto antes de integrarlas en el nodo principal de navegación. Ayuda a evitar errores de sintaxis o formato en las rutas.

### 2.4. `comunicacion_test.py`

Script básico de diagnóstico.

  * **Uso:** Comprueba que la comunicación entre nodos ROS está funcionando correctamente enviando y recibiendo mensajes de prueba simples.

-----

## 3\. Dependencias

Para que estos scripts funcionen, el entorno debe tener acceso a las siguientes librerías de ROS y Python:

  * `rospy`
  * `geometry_msgs`
  * `move_base_msgs`
  * `tf.transformations` (para el manejo de cuaterniones y orientación)
  * `numpy`

<!-- end list -->

```

