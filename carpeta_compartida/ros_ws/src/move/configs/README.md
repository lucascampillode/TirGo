## Configuración de mapas

### 1\. `move/configs/README.md`

Crea este archivo dentro de la carpeta `configs` y pega esto:

```markdown
# Configuraciones (`configs/`)

Esta carpeta almacena los archivos de configuración predefinidos para las herramientas gráficas.

---

## `rviz_configs.rviz`

Es el perfil de configuración para **RViz**. Guarda el estado de la interfaz gráfica, incluyendo:

* **Displays activos:** Mapa, RobotModel, LaserScan, TF, Path, etc.
* **Vistas:** Punto de vista de la cámara.
* **Paneles:** Herramientas laterales activas.

Este archivo es cargado automáticamente por `rviz.launch`. Si modificas la vista en RViz y pulsas "Save" (Ctrl+S), los cambios se guardarán aquí.
```

-----

### 2\. `move/maps/README.md`

Crea este archivo dentro de la carpeta `maps` y pega esto:

```markdown
# Mapas (`maps/`)

Esta carpeta contiene los archivos del mapa estático del entorno (aula/laboratorio) utilizados para la localización y navegación.

---

## Archivos del Mapa

El mapa se compone de pares de archivos generados normalmente mediante `gmapping` o `cartographer`:

* **Archivos de imagen (`.pgm`):** Contienen la información visual de ocupación (blanco = libre, negro = ocupado, gris = desconocido).
* **Archivos de metadatos (`.yaml`):** Archivos de texto que definen la resolución, el origen de coordenadas y referencias a la imagen `.pgm` correspondiente.

### Mapas disponibles

* **`Mapa_aula`**: Mapa base original.
* **`Mapa_aula_mod_1.0`**: Versión modificada/limpia, usada generalmente por el `map_server` en los scripts de lanzamiento.
```

