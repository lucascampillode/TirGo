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
├── run_all.sh    # Script principal: lanza el sistema completo
└── run_test.sh   # Script auxiliar: ejecución simplificada para pruebas
````

> ⚠️ **Nota**
> Antes de ejecutar los scripts por primera vez, asegúrate de que tienen
> permisos de ejecución:
>
> ```bash
> chmod +x run_all.sh run_test.sh
> ```

---

## 2. `run_all.sh` — Sistema principal

Este es el **script de referencia para la demo**.
Debe utilizarse cuando se quiere ejecutar el flujo completo de navegación.

### Funcionalidad

Ejecuta las siguientes acciones **en orden**:

1. **Lanza RViz**
   Cargando la configuración visual específica del proyecto.

2. **Inicia `map_server`**
   Publica el mapa estático del aula para localización y navegación.

3. **Ejecuta el nodo de navegación**
   Lanza el nodo responsable de:

   * Enviar objetivos de movimiento al robot
   * Gestionar el recorrido por checkpoints
   * Publicar flags ROS cuando se alcanza cada punto

Este script deja el sistema listo para ser controlado
por el coordinador de misión (`tirgo_mission_server`).

### Uso

Con el entorno de ROS cargado y el robot (real o simulado) activo:

```bash
roscd move/scripts
./run_all.sh
```

---

## 3. `run_test.sh` — Pruebas y depuración

Este script está pensado para **desarrollo y debugging**.

Permite:

* Probar posiciones concretas
* Recalibrar el robot
* Ejecutar navegación sin levantar todo el sistema completo
* Ahorrar tiempo durante ajustes finos

Es especialmente útil cuando se están validando
coordenadas, mapas o comportamiento del robot.

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
* `run_test.sh` acelera el desarrollo y las pruebas
* Ambos scripts reducen errores y mejoran la reproducibilidad

Esta carpeta permite que la navegación de TIAGo
**se ejecute de forma consistente con un solo comando** 🚀
