
````markdown
# Scripts de Navegación (`scripts/`)

Esta carpeta contiene los scripts en Bash encargados de orquestar el lanzamiento de los distintos nodos del sistema (visualización, mapas y lógica de navegación).

---

## 1. Archivos disponibles

```text
scripts/
├── run_all.sh   # Script principal: Lanza el sistema completo
└── run_test.sh  # Script auxiliar: Para pruebas aisladas o de desarrollo
````

> **Nota:** Antes de ejecutarlos, asegúrate de que tienen permisos de ejecución:
>
> ```bash
> chmod +x run_all.sh run_test.sh
> ```

-----

## 2\. `run_all.sh` (Sistema Principal)

Este es el script que debe ejecutarse para iniciar la demostración completa. Realiza las siguientes acciones en secuencia:

1.  **Lanza RViz** cargando la configuración visual del proyecto.
2.  **Inicia el `map_server`** para publicar el mapa estático del aula.
3.  **Ejecuta `checkpointfollower.py`**, el nodo que gestiona el envío de objetivos de navegación al robot.

### Uso

Con el entorno de ROS cargado y el robot (real o simulado) activo:

```bash
roscd move/scripts
./run_all.sh
```

-----

## 3\. `run_test.sh` (Pruebas)

Este script se utiliza para depuración o pruebas rápidas. Dependiendo de su configuración interna (se puede editar según necesidad), suele lanzar solo herramientas de visualización o nodos de test sin ejecutar la lógica completa de navegación automática.

### Uso

```bash
roscd move/scripts
./run_test.sh
```

```

