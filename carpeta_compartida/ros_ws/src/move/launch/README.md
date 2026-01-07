<div align="center">

# rviz.launch

Archivo de **lanzamiento ROS (launch file)** encargado de iniciar
la herramienta de visualización **RViz** con una configuración
predefinida para el proyecto **TirGoPharma**.

Permite visualizar el estado del robot, el mapa y los datos relevantes
de navegación **sin necesidad de configurar RViz manualmente** en cada ejecución.

</div>

---

## Visión general

El archivo `rviz.launch` automatiza la apertura de RViz cargando
un perfil guardado que define:

- Paneles visibles
- Topics suscritos
- Vista 2D/3D
- Mapa, láser, modelo del robot y transformaciones TF

Esto asegura una **visualización consistente y reproducible**
durante desarrollo, pruebas y demostraciones.

---

## 1. Ubicación dentro del paquete

El launch depende de un archivo de configuración almacenado
en la carpeta `configs/`.

```text
move/
├── configs/
│   └── rviz_configs.rviz        # Perfil RViz (paneles, topics, vista)
└── launch/
    └── rviz.launch              # Launch de RViz (este archivo)
````

---

## 2. Qué hace este launch

Al ejecutarse, `rviz.launch` realiza las siguientes acciones:

1. **Localiza la configuración RViz**
   Busca el archivo `rviz_configs.rviz` dentro del paquete `move`.

2. **Inicia RViz**
   Arranca el nodo gráfico de RViz:

   ```xml
   pkg="rviz" type="rviz"
   ```

3. **Aplica el perfil guardado**
   Utiliza el argumento `-d` (*display config*) para que RViz se abra
   mostrando directamente los elementos necesarios, como:

   * Mapa (`/map`)
   * Modelo del robot (`RobotModel`)
   * Láser/Sensores
   * Árbol de transformaciones (`TF`)

Gracias a esto, el operador puede centrarse en el sistema
sin invertir tiempo en configurar la interfaz.

---

## 3. Uso

Antes de lanzar RViz, asegúrate de que el entorno de ROS
está correctamente cargado.

```bash
source ~/carpeta_compartida/ros_ws/devel/setup.bash
roslaunch move rviz.launch
```

Al ejecutarlo:

* Se abrirá RViz automáticamente
* El mapa y el robot aparecerán cargados
* Los topics relevantes quedarán visibles desde el inicio

---

## 4. Integración con el sistema

Este launch suele ejecutarse como parte del flujo completo de navegación,
normalmente a través del script:

* `scripts/run_all.sh`

De esta forma, RViz se inicia junto al mapa y la lógica de movimiento,
permitiendo una **supervisión visual en tiempo real**.


---

## 5. Resumen

* `rviz.launch` estandariza la visualización del proyecto.
* Evita configuraciones manuales repetitivas.
* Facilita depuración, desarrollo y demos.
* Es una pieza clave para entender qué está ocurriendo en el robot.

Este launch hace que **ver el sistema sea tan sencillo como lanzarlo.** 
