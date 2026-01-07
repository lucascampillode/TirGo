<div align="center">

# Scripts de Navegación (`scripts/`)

Conjunto de **scripts Bash** para orquestar el lanzamiento de los distintos
componentes de navegación del robot TIAGo en el proyecto **TirGoPharma**.

Estos scripts simplifican la ejecución del sistema completo
(RViz, mapa y lógica de movimiento) tanto para **demo** como para **pruebas**.

</div>

---

## Visión general

La carpeta `scripts/` contiene utilidades de ejecución pensadas para:

- Evitar lanzar nodos manualmente uno a uno
- Garantizar el orden correcto de arranque
- Facilitar pruebas rápidas durante el desarrollo
- Reducir errores humanos en la demo

---

## 1. Archivos disponibles

```text
scripts/
├── run_all.sh    # Script principal: demo completa (RViz + mapa + movimiento)
└── run_test.sh   # Script auxiliar: pruebas/debug (puede requerir ajustes)
````

> ⚠️ **Nota**
> Antes de ejecutar los scripts por primera vez, asegúrate de que tienen
> permisos de ejecución:
>
> ```bash
> chmod +x run_all.sh run_test.sh
> ```

---

## 2. `run_all.sh` — Sistema principal (demo)

Este es el **script de referencia para la demo**.
Debe utilizarse cuando se quiere ejecutar el flujo completo de navegación.

### Funcionalidad

Ejecuta las siguientes acciones **en orden**:

1. **Lanza RViz**
   Cargando la configuración visual específica del proyecto (`rviz.launch`).

2. **Inicia `map_server`**
   Publica el mapa estático del entorno para localización/navegación.

3. **Publica la pose inicial**
   Lanza `publish_initial_pose.py` para ayudar a iniciar la localización sin intervención manual.

4. **Ejecuta el orquestador de movimiento**
   Lanza `comunication_move.py`, que:

   * escucha el inicio de misión (p. ej. `/tirgo/mission/start`)
   * ejecuta desplazamientos a puntos clave del flujo
   * publica hitos como `/tirgo/tiago/arrived` y `/tirgo/tiago/at_patient`

Este script deja el sistema listo para ser controlado
por el coordinador de misión (`tirgo_mission_server`).

### Uso

Con el entorno ROS cargado y el robot (real o simulado) activo:

```bash
roscd move/scripts
./run_all.sh
```

---

## 3. `run_test.sh` — Pruebas y depuración

Este script está pensado para **desarrollo y debugging**.

Permite acelerar iteraciones durante el desarrollo, por ejemplo:

* Probar visualización/localización sin levantar todo el sistema
* Hacer comprobaciones rápidas del stack de navegación
* Validar ajustes del entorno

> Nota importante: `run_test.sh` es un script auxiliar.
> Según el entorno (rutas, mapa, configuración del robot), puede requerir ajustes.

### Uso

```bash
roscd move/scripts
./run_test.sh
```

---

## 4. Cuándo usar cada script

| Escenario                      | Script recomendado |
| ------------------------------ | ------------------ |
| Demo completa del sistema      | `run_all.sh`       |
| Pruebas de navegación aisladas | `run_test.sh`      |
| Ajuste de mapa / localización  | `run_test.sh`      |
| Ejecución integrada con misión | `run_all.sh`       |

---

## 5. Resumen

* `scripts/` centraliza la **ejecución controlada** del sistema de navegación
* `run_all.sh` es el punto de entrada para la demo oficial
* `run_test.sh` acelera desarrollo/pruebas (con posibles ajustes por entorno)

Esta carpeta permite que la navegación de TIAGo
**se ejecute de forma consistente con un solo comando** 
