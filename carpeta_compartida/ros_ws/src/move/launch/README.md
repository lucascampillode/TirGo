# rviz.launch

Archivo de lanzamiento (**launch file**) que inicializa la herramienta de visualización gráfica **RViz** cargando un perfil de configuración predefinido.

Este fichero es esencial para visualizar el estado del robot, los datos de los sensores y el mapa durante la navegación sin tener que configurar las ventanas manualmente cada vez.

---

## 1. Ubicación en el Paquete

Este archivo se encuentra en la carpeta `launch/` y depende de un archivo de configuración situado en `configs/`.

```text
move/
├── configs/
│   └── rviz_configs.rviz    ← Perfil guardado (paneles, topics, vista)
└── launch/
    └── rviz.launch          ← ESTE ARCHIVO
```
## 2. Qué hace este launch

 * Busca la configuración: Localiza el archivo rviz_configs.rviz dentro del paquete move.

 * Inicia el Nodo: Arranca el ejecutable de RViz (pkg="rviz" type="rviz").

 * Aplica el Perfil: Pasa el argumento -d (display config) para que RViz se abra mostrando ya los topics configurados (Laser, Mapa, RobotModel, TF, etc.).
## 3. Uso
```bash
    source /home/TirGo/carpeta_compartida/ros_ws/devel/setup.bash
    roslaunch move rviz.launch 
 ```
