<div align="center">

# Configuraciones (`configs/`)

Archivos de **configuración visual y de herramientas gráficas**
utilizados por el paquete `move` dentro del proyecto **TirGoPharma**.

</div>

---

## Visión general

La carpeta `configs/` almacena configuraciones predefinidas
que permiten lanzar herramientas gráficas (principalmente **RViz**)
con un estado coherente y listo para usar.

Esto evita tener que configurar manualmente la interfaz
en cada ejecución.

---

## `rviz_configs.rviz`

Archivo de perfil para **RViz**.

Guarda el estado completo de la interfaz gráfica, incluyendo:

- **Displays activos**
  - Mapa (`/map`)
  - Modelo del robot (`RobotModel`)
  - Sensores (LaserScan)
  - Transformaciones (`TF`)
  - Trayectoria (`Path`)
- **Vistas**
  - Posición y orientación de la cámara
- **Paneles**
  - Herramientas laterales y controles visibles

Este archivo es cargado automáticamente por:

- `launch/rviz.launch`

### Modificación del perfil

Si realizas cambios en RViz (añadir displays, mover la cámara, etc.):

1. Abre RViz con `rviz.launch`
2. Ajusta la vista según tus necesidades
3. Guarda con **Ctrl + S**

Los cambios quedarán persistentes en `rviz_configs.rviz`.

---

## Resumen

- `configs/` centraliza la configuración visual del proyecto
- `rviz_configs.rviz` garantiza una visualización consistente
- Es clave para depuración, desarrollo y demostraciones
