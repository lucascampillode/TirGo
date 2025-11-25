# Navegación y Uso de run_all.sh

Sistema mínimo para levantar navegación básica del robot TIAGo con: teleoperación, carga de mapa, RViz configurado y un cliente que publica posiciones objetivo.

---

## Documentación Relacionada

- [Documentación de launch](./launch/README.md)
- [Documentación de run_all.sh](./scripts/README.md)
- [Documentación del módulo move](./src/move/README.md)

---

## Requisitos

Asegúrate de cumplir con lo siguiente antes de ejecutar el sistema:

- **Sistema Operativo:** Ubuntu 20.04  
- **ROS:** Noetic  
- **Contenedorización (Opcional, Recomendado):** Docker  
- **Lenguaje de Programación:** Python 3.8+

---

## Paquetes ROS Necesarios

Los siguientes paquetes deben estar instalados:

- `ros-noetic-teleop-twist-keyboard`  
- `ros-noetic-map-server`  
- `ros-noetic-move-base-msgs`

Este módulo implementa el flujo completo de navegación del robot TIAGo dentro del proyecto TirgoPharma.

---

## Ejecuciones

### **run_all.sh**
```bash
cd carpeta_compartida/ros_ws/src/move/scripts/
./run_all.sh
```
### rviz.launch (Carga rviz)
```bash
    source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash
    roslaunch move rviz.launch 
 ```
### map_server (carga mapa)
```bash
    source /opt/ros/noetic/setup.bash
    rosrun map_server map_server /home/TirGo/carpeta_compartida/ros_ws/src/move/maps/Mapa_aula_mod_1.0.yaml 
 ```

 ## Verificar

Al lanzar run_all.sh, se visualizará el robot en el mapa de RViz y comenzará a moverse hasta la posición objetivo predefinida.
