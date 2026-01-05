<div align="center">

# Hardware Dispensador TirGo

**Diseño mecánico, validación e ingeniería de fabricación aditiva.**

Módulo compacto para la dosificación automatizada de fármacos, diseñado para integración robótica y fabricación mediante impresión 3D FDM.

[Diseño CAD] • [Impresión 3D] • [Mecatrónica]

</div>

---

## Índice

- [1. Visión general del sistema](#1-visión-general-del-sistema)
- [2. Diseño CAD y validación](#2-diseño-cad-y-validación)
- [3. Mecatrónica y Lista de Materiales (BOM)](#3-mecatrónica-y-lista-de-materiales-bom)
- [4. Preparación para fabricación (DFM)](#4-preparación-para-fabricación-dfm)
- [5. Proceso de impresión 3D](#5-proceso-de-impresión-3d)
- [6. Resultado final e integración](#6-resultado-final-e-integración)

---

## 1. Visión general del sistema

El dispensador está concebido como un **módulo compacto, robusto y reproducible**, pensado para integrarse en entornos automatizados (como farmacias robotizadas o el robot TIAGo). El diseño prioriza:

* **Modularidad:** Separación estructural para facilitar mantenimiento.
* **Fabricación FDM:** Optimizado para impresoras 3D estándar sin sacrificar tolerancias.
* **Universalidad:** Integración limpia de frascos de farmacia estándar.
* **Estética Industrial:** Cerramiento protector y líneas limpias.

A nivel funcional, el sistema combina una estructura portante, una tolva interna de direccionamiento, un sistema de guiado de frascos y un volumen superior para almacenamiento y actuación.

---

## 2. Diseño CAD y validación

El diseño se ha realizado íntegramente en CAD, iterando desde volúmenes simples hasta la geometría final funcional. Se han priorizado espesores constantes para FDM y radios para evitar concentraciones de tensiones.

### Vistas Técnicas del Conjunto

<div align="center">
  <table>
    <tr>
      <td align="center">
        <b>Vista Isométrica (Ensamblaje)</b><br>
        <a href="Galeria/RenderPerspectivaIsometrica.png">
          <img src="Galeria/RenderPerspectivaIsometrica.png" width="400" alt="Vista Isométrica"/>
        </a><br>
        <a href="Galeria/RenderPerspectivaIsometrica.png">📄 Ver imagen original</a>
      </td>
      <td align="center">
        <b>Vista Frontal (Alzado)</b><br>
        <a href="Galeria/RenderAlzadoFrontal.png">
          <img src="Galeria/RenderAlzadoFrontal.png" width="400" alt="Vista Frontal"/>
        </a><br>
        <a href="Galeria/RenderAlzadoFrontal.png">📄 Ver imagen original</a>
      </td>
    </tr>
    <tr>
      <td align="center">
        <b>Perfil Lateral</b><br>
        <a href="Galeria/RenderPerfilLateral.png">
          <img src="Galeria/RenderPerfilLateral.png" width="400" alt="Vista Lateral"/>
        </a><br>
        <a href="Galeria/RenderPerfilLateral.png">📄 Ver imagen original</a>
      </td>
      <td align="center">
        <b>Mecanismo Superior</b><br>
        <a href="Galeria/RenderDetalleMecanismoSuperior.png">
          <img src="Galeria/RenderDetalleMecanismoSuperior.png" width="400" alt="Detalle mecanismo"/>
        </a><br>
        <a href="Galeria/RenderDetalleMecanismoSuperior.png">📄 Ver imagen original</a>
      </td>
    </tr>
  </table>
</div>

Estas vistas permitieron verificar:
* **Coaxialidad** del cuello de salida.
* **Espacio libre** para la manipulación de frascos.
* **Continuidad** de superficies internas (evitar atascos en la tolva).
* **Validación de electrónica:** Espacio reservado para el actuador y cableado.

---

## 3. Mecatrónica y Lista de Materiales (BOM)

Para el sistema de liberación y dosificación de los medicamentos, se ha optado por una solución compacta y de bajo consumo.

### Componentes Principales

| Componente | Cantidad | Especificaciones | Función |
| :--- | :---: | :--- | :--- |
| **Servo Motor** | 4 | **SG90** (Micro Servo 9g) | Accionamiento de las levas de dispensación. |
| **Cuerpo Principal** | 1 | PETG / PLA+ (Impreso) | Estructura, tolva y alojamiento de servos. |
| **Base Estructural** | 1 | PETG / PLA+ (Impreso) | Chasis de soporte y anclaje al robot. |
| **Frascos** | 4 | Estándar Farmacia | Contenedores de medicamento. |

> **Nota técnica sobre los SG90:** El diseño integra alojamientos específicos (interference fit) para los micro-servos, permitiendo un acople directo a los gatillos de dispensación sin necesidad de adhesivos complejos.

---

## 4. Preparación para fabricación (DFM)

Antes de imprimir, el modelo se adaptó específicamente a fabricación aditiva (Design for Additive Manufacturing):

* **División estratégica:** Separación de la base y el cuerpo superior.
* **Voladizos:** Eliminación de ángulos críticos (<45º) para reducir soportes.
* **Orientación:** Optimizada para maximizar la resistencia mecánica en el eje Z.

---

## 5. Proceso de impresión 3D

La fabricación se realizó mediante **impresión 3D FDM**, utilizando parámetros ajustados para resistencia estructural.

### Pieza A: Cuerpo del Dispensador (Tolva)

<div align="center">
  <a href="Galeria/LaminadoEmbudoInterno.png">
    <img src="Galeria/LaminadoEmbudoInterno.png" width="600" alt="Slicing Cuerpo"/>
  </a>
  <br>
  <a href="Galeria/LaminadoEmbudoInterno.png">📄 Ver captura de laminado original</a>
</div>

* **Tiempo estimado:** ~7h 45m
* **Estrategia:** Infill estructural y costuras alineadas en la cara posterior para estética visual.
* **Soportes:** Mínimos, solo en los puentes de los alojamientos de servos.

### Pieza B: Base Estructural

<div align="center">
  <a href="Galeria/LaminadoBaseEstructural.png">
    <img src="Galeria/LaminadoBaseEstructural.png" width="600" alt="Slicing Base"/>
  </a>
  <br>
  <a href="Galeria/LaminadoBaseEstructural.png">📄 Ver captura de laminado original</a>
</div>

* **Tiempo estimado:** ~3h 24m
* **Estrategia:** Alta densidad de relleno para asegurar un centro de gravedad bajo y mayor estabilidad en la base.

---

## 6. Resultado final e integración

El sistema final valida las tolerancias teóricas del CAD. La separación en piezas permite un ensamblaje con holguras controladas.

### Prueba de ensamblaje (4 Botes)

<div align="center">
  <a href="Galeria/RenderEnsamblajeCompleto.png">
    <img src="Galeria/RenderEnsamblajeCompleto.png" width="500" alt="Integración final"/>
  </a>
  <br>
  <a href="Galeria/RenderEnsamblajeCompleto.png">📄 Ver render de alta resolución</a>
</div>

El diseño final cumple con los requisitos de:
1.  **Estabilidad:** Guiado vertical de los frascos garantizado.
2.  **Accesibilidad:** Ventanas de inspección para verificar stock visualmente.
3.  **Mantenibilidad:** Los servos SG90 son accesibles para su sustitución en caso de fallo sin desmontar toda la estructura.

---

### Próximos pasos

* Integración del cableado hacia el controlador (ESP32 / Arduino / ROS Driver).
* Ensayos de fatiga mecánica en las levas impresas.
* Optimización de flujo para reducir tiempos de impresión en serie.
