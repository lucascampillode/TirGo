<div align="center">

# Mapas (`maps/`)

Archivos de **mapas estáticos** utilizados para la localización
y navegación del robot TIAGo en el proyecto **TirGoPharma**.

</div>

---

## Visión general

La carpeta `maps/` contiene los mapas del entorno físico
(aula o laboratorio) sobre los que el robot se localiza
y planifica sus trayectorias.

Estos mapas son consumidos por:

- `map_server`
- `amcl`
- El stack de navegación (`move_base`)

---

## Estructura de los mapas

Cada mapa está compuesto por **dos tipos de archivos**:

### 1. Imagen del mapa (`.pgm`)

- Representa la ocupación del entorno
- Convención habitual:
  - **Blanco** → espacio libre
  - **Negro** → obstáculo
  - **Gris** → desconocido

### 2. Metadatos del mapa (`.yaml`)

Archivo de texto que define:

- Resolución del mapa (metros/píxel)
- Origen del sistema de coordenadas
- Ruta al archivo `.pgm` correspondiente
- Parámetros de umbral de ocupación

---

## Mapas disponibles

- **`Mapa_aula`**  
  Mapa base original del entorno.

- **`Mapa_aula_mod_1.0`**  
  Versión modificada y depurada.  
  Es la utilizada habitualmente por `map_server`
  en los scripts de lanzamiento.

---

## Vista previa del mapa

A continuación se muestra una **vista previa del mapa de ocupación**
utilizado durante la navegación del robot.

<img width="506" height="609" alt="image" src="https://github.com/user-attachments/assets/a1ce8c56-fac2-4c5f-b525-2934c3362af6" />

Esta visualización ayuda a comprender la relación entre:
- el entorno físico real,
- el mapa utilizado por el sistema,
- y las trayectorias que sigue el robot durante la demo.

---

## Uso en el sistema

Los mapas de esta carpeta se cargan normalmente mediante:

- Scripts de `scripts/`
- Launch files del paquete `move`

Esto permite cambiar de mapa
sin modificar la lógica de navegación.

---

## Resumen

- `maps/` contiene los mapas estáticos del entorno
- Los mapas se componen de `.pgm` + `.yaml`
- Son fundamentales para localización y navegación fiable
- Permiten reproducir el comportamiento del robot en cada ejecución
