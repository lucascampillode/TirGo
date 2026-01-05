# Dispensador automático – Diseño y construcción

Este documento describe el **proceso completo de diseño y fabricación** del dispensador automático, con especial foco en el trabajo de ingeniería mecánica, iteraciones de diseño y fabricación mediante impresión 3D. El objetivo es mostrar no solo el resultado final, sino el razonamiento y las decisiones técnicas que hay detrás del producto.

---

## 1. Visión general del sistema

El dispensador está concebido como un **módulo compacto, robusto y reproducible**, pensado para integrarse en entornos automatizados (p. ej. farmacia robotizada o sistema de dispensación autónomo). El diseño prioriza:

* Modularidad de piezas
* Fabricación mediante impresión 3D FDM
* Integración limpia de frascos estándar
* Accesibilidad para mantenimiento
* Estética industrial y cerramiento protector

A nivel funcional, el sistema combina:

* Estructura portante
* Tolva interna
* Sistema de guiado de frascos
* Volumen superior para almacenamiento

---

## 2. Diseño CAD y validación geométrica

El diseño se ha realizado íntegramente en CAD, iterando progresivamente desde volúmenes simples hasta la geometría final funcional.

Aspectos clave del diseño:

* Espesores optimizados para FDM
* Radios y chaflanes para evitar concentraciones de tensiones
* Ventanas de inspección visual
* Alojamiento preciso de frascos y cuello de salida

### Vista general del conjunto

![Vista CAD general](Dispensador_2025-Dec-31_09-39-01AM-000_CustomizedView6789720553.png)

### Vistas funcionales y de integración

![Vista frontal](Dispensador_2025-Dec-13_09-21-46PM-000_CustomizedView14323265278.png)
![Vista lateral](Dispensador_2025-Dec-13_09-20-00PM-000_CustomizedView14512961602.png)
![Vista trasera](Dispensador_2025-Dec-13_09-18-20PM-000_CustomizedView32298590179.png)
![Vista inferior](Dispensador_2025-Dec-13_09-14-16PM-000_CustomizedView20955392085.png)

Estas vistas permitieron verificar:

* Coaxialidad del cuello de salida
* Espacio libre para frascos
* Continuidad de superficies internas
* Accesos para ensamblaje

---

## 3. Preparación para fabricación (DFM)

Antes de imprimir, el modelo se adaptó específicamente a fabricación aditiva:

* División en piezas imprimibles
* Eliminación de voladizos críticos
* Orientación optimizada para resistencia mecánica
* Minimización de soportes

Se decidió **separar la base y el cuerpo superior**, lo que facilita:

* Reducción del tiempo de impresión
* Menor riesgo de fallo
* Sustitución independiente de piezas

---

## 4. Proceso de impresión 3D

La fabricación se realizó mediante **impresión 3D FDM**, utilizando un material plástico técnico (PETG / PLA+ según iteración).

### Pieza principal – cuerpo del dispensador

![Slicing cuerpo completo](Captura de pantalla 2025-12-31 104324.png)

Parámetros representativos:

* Altura de capa: optimizada para equilibrio entre detalle y tiempo
* Infill estructural
* Perímetros reforzados
* Costuras controladas para estética

Tiempo aproximado de impresión:

* ~7 h 45 min

### Base inferior

![Slicing base](Captura de pantalla 2025-12-31 104239.png)

Tiempo aproximado de impresión:

* ~3 h 24 min

La separación en piezas permitió imprimir de forma más fiable y repetir solo la parte necesaria en caso de fallo.

---

## 5. Postprocesado y ensamblaje

Tras la impresión se realizaron tareas de:

* Retirada de soportes
* Limpieza de superficies críticas
* Verificación dimensional
* Ensamblaje en seco

El ajuste entre piezas fue diseñado con **holguras controladas**, evitando necesidad de mecanizado posterior.

---

## 6. Integración de frascos y pruebas físicas

El sistema está diseñado para trabajar con frascos estándar, manteniendo:

* Guiado vertical estable
* Alineación precisa con la salida
* Protección frente a interferencias externas

### Prueba de integración con frascos

![Integración de frascos](Dispensador_2025-Dec-13_07-27-59PM-000_CustomizedView9611341866.png)

Esta fase permitió validar:

* Tolerancias reales vs CAD
* Estabilidad estructural
* Accesibilidad y visibilidad

---

## 7. Resultado final

El resultado es un **dispensador funcional, fabricable y escalable**, con un diseño que refleja:

* Iteración de ingeniería
* Pensamiento orientado a fabricación
* Integración mecánica limpia
* Estética coherente con un producto final

Este README pretende dejar constancia del **trabajo de diseño y fabricación** detrás del sistema, más allá del simple objeto impreso.

---

## 8. Próximos pasos

* Integración de electrónica y actuadores
* Ensayos de ciclo de vida
* Optimización de tiempos de impresión
* Preparación para documentación de usuario final
