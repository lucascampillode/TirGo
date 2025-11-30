# checkpointfollower.py

Nodo de **lógica de navegación** en Python.

Este script actúa como un "director de misión": contiene una lista predefinida de coordenadas (checkpoints) y se encarga de enviarlas una a una al sistema de navegación (`move_base`), verificando manualmente que el robot se mueva y llegue a su destino antes de enviar la siguiente.

---

## 1. Ubicación en el Paquete

Se encuentra en la carpeta `scripts/` y debe tener permisos de ejecución.

```text
move/
└── scripts/
    ├── run_all.sh               ← Script que lanza este nodo
    └── checkpointfollower.py    ← ESTE ARCHIVO
```
## 2.Cómo funciona

A diferencia de un cliente de acción estándar, este nodo implementa su propia lógica de verificación basada en la posición real del robot:

  * Publica un objetivo (goal) en el topic de navegación.

  * Espera movimiento: Monitoriza la posición del robot (/robot_pose) hasta detectar que ha empezado a desplazarse.

  * Espera llegada: Sigue monitorizando hasta que el robot deja de moverse (velocidad cercana a 0 durante un tiempo), asumiendo que ha llegado al objetivo.

  * Siguiente punto: Repite el proceso con la siguiente coordenada de la lista.

## 3. Topics ROS
El nodo interactúa con la navegación a través de estos canales:
```
Tipo              Topic                    Mensaje                                              Descripción
Publica,    /move_base/goal    move_base_msgs/MoveBaseActionGoal           Envía la coordenada objetivo al planificador.
Suscribe      /robot_pose      geometry_msgs/PoseWithCovarianceStamped     Lee la posición actual para saber si el robot se mueve o está quieto.
```
## 4.Cómo editar los Puntos (Checkpoints)

Las coordenadas están definidas al final del archivo, en el bloque if __name__ == "__main__":.

Para cambiar la ruta, edita la lista checkpoints. El formato es: [Posición X, Posición Y, Orientación Z, Orientación W]
```Python
checkpoints = [
    # [   X,      Y,       oz,       ow   ]
    [1.80,  -0.72,   -0.17,    0.98],  # Puerta
    [0.75,   0.69,    0.87,    0.48],  # Puerta 2
    [3.67,  -1.89,   -0.34,    0.93],  # Al Medio
    [6.12,  -2.15,   -0.10,    0.99],  # Al Fondo
]
```
## 6.Dependencias Python

Este script utiliza las siguientes librerías estándar y de ROS:

  rospy

  numpy (cálculo matricial y distancias)

  tf.transformations (manejo de cuaterniones)
