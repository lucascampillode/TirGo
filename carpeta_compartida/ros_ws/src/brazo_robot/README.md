# Control y movimiento del brazo (TIAGo · ROS1 Noetic)

Este paquete implementa un **nodo residente de control del brazo** del robot **TIAGo**, integrado dentro de un **sistema completo de dispensación autónoma de medicación**.

El nodo actúa como una **capa de ejecución de movimientos** que permanece en funcionamiento constante y **responde a eventos de alto nivel** enviados por otros subsistemas del robot (navegación, dispensador, lógica de misión).

Su función es **ejecutar de forma segura y secuencial** las maniobras de:

* **recogida del medicamento desde el dispensador**
* **entrega del medicamento al paciente**

utilizando el **torso, brazo de 7 GDL, cabeza y gripper**.

---

## Rol dentro del proyecto global

Este nodo **no decide cuándo moverse** ni planifica navegación.
Forma parte de un sistema mayor donde:

* La **base móvil** se encarga de navegar
* El **dispensador** gestiona la liberación del medicamento
* Este nodo **solo ejecuta las secuencias de brazo** cuando recibe los eventos adecuados

### Filosofía de diseño

* Nodo **siempre activo**
* Lógica **reactiva por eventos**
* Movimientos **compactos y seguros**
* Separación clara entre:

  * *decisión / navegación*
  * *ejecución física del brazo*

---

## Funcionamiento general

### Secuencia de recogida (COGER)

1. La **base del robot** alcanza la posición frente al dispensador.
2. El **dispensador confirma** que el medicamento está listo.
3. Se publica el evento:

   * `/tirgo/dispense/ready = true`
4. El nodo:

   * despliega el brazo de forma progresiva
   * se aproxima al dispensador
   * **abre el gripper**
   * **coge el bote**
   * **cierra el gripper**
   * vuelve a **posición HOME con el bote sujeto**
5. Publica:

   * `/tirgo/tiago/picked = true`

Esto permite que el robot **se desplace de forma segura y compacta** hacia el punto de entrega **con el medicamento ya asegurado**, evitando colisiones o movimientos innecesarios.

---

### Secuencia de entrega (DEJAR)

1. La **base del robot** alcanza la posición de entrega al paciente.
2. El sistema de navegación publica:

   * `/tirgo/tiago/at_patient = true`
3. El nodo:

   * ejecuta la misma secuencia de movimientos
   * **invierte la lógica del gripper**:

     * se posiciona
     * **abre el gripper para soltar el bote**
     * se retira de forma controlada
     * vuelve a HOME
4. Publica:

   * `/tirgo/tiago/delivered = true`

La inversión del orden de apertura/cierre del gripper garantiza una **entrega correcta y controlada** del medicamento al paciente.

---

## Qué hace exactamente el nodo

* Controla:

  * **Torso** (`torso_lift_joint`)
  * **Brazo** (7 GDL)
  * **Cabeza**
  * **Gripper**
* Ejecuta una **secuencia de poses predefinidas**
* Bloquea cada paso hasta que el brazo termina el movimiento
* No avanza si un paso no ha finalizado correctamente
* Publica eventos de estado para el resto del sistema

---

## Interfaces ROS

### Suscriptores (eventos de alto nivel)

| Topic                     | Tipo            | Significado                                                 |
| ------------------------- | --------------- | ----------------------------------------------------------- |
| `/tirgo/dispense/ready`   | `std_msgs/Bool` | El robot está en el dispensador y el medicamento está listo |
| `/tirgo/tiago/at_patient` | `std_msgs/Bool` | La base del robot ha alcanzado la posición de entrega       |

> El nodo **solo actúa cuando recibe `true`**.
> Los `false` se ignoran explícitamente.

---

### Publicadores de control (movimiento)

#### Brazo (Actionlib)

* `/arm_controller/follow_joint_trajectory`
* Tipo: `control_msgs/FollowJointTrajectoryAction`
* Se utiliza como **elemento de sincronización** de la secuencia

#### Torso

* `/torso_controller/command`
* Tipo: `trajectory_msgs/JointTrajectory`

#### Cabeza

* `/head_controller/command`
* Tipo: `trajectory_msgs/JointTrajectory`

#### Gripper

* `/gripper_controller/command`
* Tipo: `trajectory_msgs/JointTrajectory`
* Control explícito de apertura y cierre durante la secuencia

---

### Publicadores de estado (feedback al sistema)

| Topic                    | Tipo            | Cuándo se publica                  |
| ------------------------ | --------------- | ---------------------------------- |
| `/tirgo/tiago/picked`    | `std_msgs/Bool` | Finaliza correctamente la recogida |
| `/tirgo/tiago/delivered` | `std_msgs/Bool` | Finaliza correctamente la entrega  |

Estos eventos permiten que otros nodos **continúen la misión** sin acoplarse al control del brazo.

---

## Requisitos

* Ubuntu 20.04
* ROS **Noetic**
* Robot TIAGo (real o simulado)
* Controladores activos:

  * `arm_controller`
  * `torso_controller`
  * `head_controller`
  * `gripper_controller`

---

## Ejecución

> Asumiendo que el robot, la navegación y los controladores ya están lanzados.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun brazo_robot tiago_secuencia_brazo.py
```

Al arrancar, el nodo:

* Se registra como `tiago_secuencia_brazo`
* Espera al action server del brazo
* Queda en **modo espera de eventos**

---

## Disparo manual de secuencias (testing)

### Simular recogida

```bash
rostopic pub /tirgo/dispense/ready std_msgs/Bool "data: true" -1
```

### Simular entrega

```bash
rostopic pub /tirgo/tiago/at_patient std_msgs/Bool "data: true" -1
```

---

## Monitorización y debugging

### Estados de controladores

```bash
rostopic echo /arm_controller/state
```

### Logs del sistema

```bash
rostopic echo /rosout
```

### Ver comandos publicados

```bash
rostopic echo /torso_controller/command
rostopic echo /head_controller/command
rostopic echo /gripper_controller/command
```

---

## Notas de integración

* Este nodo **no debe lanzarse múltiples veces**
* Está pensado para mantanerse activo durante toda la ejecución del sistema
* No acumula eventos: usa **flags lógicos**


