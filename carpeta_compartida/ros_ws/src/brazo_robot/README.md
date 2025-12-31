# Control y movimiento del brazo (TIAGo · ROS1 Noetic)

Control secuencial de **torso + brazo (7 GDL) + cabeza + gripper** usando controladores `FollowJointTrajectory` (actionlib) en TIAGo.

Publica/ejecuta trayectorias en los controladores estándar de PAL y permite lanzar una **secuencia de movimientos** desde un nodo Python.

---

## Qué hace

- Envía trayectorias a:
  - **Torso**: `torso_lift_joint`
  - **Brazo** (7 joints): `arm_1_joint ... arm_7_joint`
  - **Cabeza**: `head_1_joint`, `head_2_joint`
- Controla el **gripper**
- Ejecuta en **bloqueo secuencial**: no pasa al siguiente paso hasta que el anterior termine.

---

## Interfaces ROS

### Actions (recomendado)
- `/torso_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectoryAction`)
- `/arm_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectoryAction`)
- `/head_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectoryAction`)

### Topics (debug / alternativas según setup)
- `/torso_controller/command` (`trajectory_msgs/JointTrajectory`)
- `/arm_controller/command` (`trajectory_msgs/JointTrajectory`)
- `/head_controller/command` (`trajectory_msgs/JointTrajectory`)

---

## Requisitos

- Ubuntu 20.04 + ROS **Noetic**
- Robot TIAGo (o simulación) con controladores cargados
- `roscore` en ejecución
- Python 3 (con `rospy` disponible)

---

## Estructura típica

```text
<tu_ws>/src/<tu_paquete>/
├─ scripts/
│  └─ tiago_secuencia_brazo.py
└─ README.md


````markdown
## Ejecución

### 1) Iniciar ROS
En una terminal:

```bash
roscore
````

---

### 2) Lanzar el robot y los controladores

En otra terminal, arranca el robot real o la simulación asegurándote de que los **controladores del torso, brazo y cabeza** están activos.

Comprueba que los servidores de trayectoria existen:

```bash
rostopic list | grep follow_joint_trajectory
```

Deben aparecer, al menos:

```text
/arm_controller/follow_joint_trajectory/goal
/torso_controller/follow_joint_trajectory/goal
/head_controller/follow_joint_trajectory/goal
```

---

### 3) Ejecutar el nodo de control del brazo

En otra terminal:

```bash
source /opt/ros/noetic/setup.bash
source <tu_ws>/devel/setup.bash
rosrun <tu_paquete> tiago_secuencia_brazo.py
```

El nodo esperará automáticamente a que los servidores de trayectoria estén disponibles y ejecutará la secuencia de movimientos definida.

---

### 4) Verificar ejecución

Durante la ejecución puedes comprobar la actividad de los controladores:

```bash
rostopic echo /arm_controller/state
```

o revisar los logs:

```bash
rostopic echo /rosout
```

