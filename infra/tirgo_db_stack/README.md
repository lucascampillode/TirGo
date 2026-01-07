<div align="center">

# `tirgo_db_stack`

Infraestructura **MongoDB + mongo-express** para el sistema **TirGoPharma**.

Stack Docker de **base de datos persistente**, aislado a nivel de red  
y orientado al **entorno de desarrollo e integración** del sistema completo.

</div>

---

## 1. Visión general

Este directorio define la infraestructura de base de datos utilizada por **TirGoPharma**.

El objetivo del stack es proporcionar:

- Un servicio **MongoDB 7** reproducible
- **Persistencia de datos** mediante volumen Docker
- **Aislamiento de red**, exponiendo los servicios únicamente en `localhost`
- Una consola web (**mongo-express**) para inspección y edición manual de datos

La base de datos levantada por este stack actúa como **backend de datos** para la aplicación
`tirgo_ui` y el resto de componentes del sistema TirGoPharma.

---

## 2. Estructura del directorio

```text
infra/tirgo_db_stack/
  ├─ .env                       # Variables de entorno del stack (contraseñas)
  ├─ docker-compose.yml         # Servicios Docker (Mongo + mongo-express)
  └─ mongo-init/
      └─ 001-init.js            # Inicialización de la BD "tirgo"
````

---

## 3. Configuración del stack (`.env`)

Ejemplo del fichero `.env`:

```env
MONGO_ROOT_PASSWORD=superseguro
TIRGO_DB_PASSWORD=tirgo_pass_cambia
```

### Variables definidas

* **`MONGO_ROOT_PASSWORD`**
  Contraseña del usuario **administrador de MongoDB** (`admin`), con permisos de administración
  sobre el servidor completo.
  Este usuario es utilizado por `mongo-express`.

* **`TIRGO_DB_PASSWORD`**
  Contraseña del usuario de aplicación **`tirgo_user`**, creado durante la inicialización de la base
  de datos y utilizado por la aplicación para acceder a la BD `tirgo`.

> Los valores de este fichero están pensados **exclusivamente para desarrollo**.
> En un entorno productivo deben usarse contraseñas robustas y mecanismos seguros de gestión
> de secretos.

---

## 4. Definición del stack Docker

El archivo `docker-compose.yml` define:

* **Servicio `mongo`** → servidor MongoDB 7
* **Servicio `mongo-express`** → consola web de administración
* **Volumen `mongo_data`** → almacenamiento persistente

Los puertos se publican **exclusivamente en la interfaz local (`127.0.0.1`)**, evitando la exposición
del servicio fuera del host.

### 4.1 Servicio `mongo`

Características principales:

* Imagen oficial: `mongo:7`
* Volumen persistente montado en `/data/db`
* Ejecución automática de scripts `.js` ubicados en
  `/docker-entrypoint-initdb.d` **solo cuando el volumen está vacío**
* Puerto `27017` publicado únicamente en `127.0.0.1`

---

### 4.2 Servicio `mongo-express`

Características:

* Interfaz web para inspección y edición de datos
* Conexión a Mongo usando el usuario administrador (`admin`)
* Protección mediante autenticación básica
* Consola accesible solo en:

```text
http://localhost:8081
```

---

## 5. Proceso de inicialización de la base de datos

La inicialización se realiza mediante el script:

```text
mongo-init/001-init.js
```

Este script se ejecuta automáticamente **solo en el primer arranque** del contenedor
cuando el volumen `mongo_data` no existe.

### 5.1 Base de datos de aplicación

Se trabaja explícitamente sobre la base de datos:

```js
tirgo
```

---

### 5.2 Usuario de aplicación

Se crea el usuario:

* **Nombre:** `tirgo_user`
* **Rol:** `readWrite`
* **Base de datos:** `tirgo`
* **Contraseña:** valor de `TIRGO_DB_PASSWORD`

Este es el usuario previsto para el acceso de la aplicación a la base de datos.

---

### 5.3 Índices definidos

Para garantizar integridad y eficiencia:

* `pacientes.dni_hash` → índice **único**
* `medicamentos.nombre` → índice **único**
* `recetas`:

  * `(paciente_id, activa)`
  * `(medicamento_id, activa)`

---

### 5.4 Datos iniciales

Se insertan (mediante `upsert`) dos medicamentos de referencia:

* **Amoxicilina 500mg** (requiere receta)
* **Ibuprofeno 400mg** (no requiere receta)

Estos datos permiten que la aplicación tenga un estado mínimo funcional
desde el primer arranque.

---

## 6. Integración con TirGoPharma

La base de datos `tirgo` es utilizada como backend por el sistema TirGoPharma.

Colecciones principales:

* `pacientes`
* `medicamentos`
* `recetas`
* `dispensaciones`
* `logs`

### Cadena de conexión típica

Ejemplo de `MONGO_URI` utilizado por la aplicación:

```bash
mongodb://tirgo_user:<TIRGO_DB_PASSWORD>@127.0.0.1:27017/tirgo?authSource=tirgo
```

---

## 7. Puesta en marcha del stack

### 7.1 Requisitos

* Docker
* Docker Compose v2

### 7.2 Arranque

```bash
cd infra/tirgo_db_stack
docker compose up -d
```

Verificación del estado:

```bash
docker compose ps
docker logs tirgo_mongo
```

Una vez arrancado, MongoDB queda disponible en:

```text
mongodb://localhost:27017
```

---

## 8. Reset y recreación completa de la base de datos

Para restaurar la base de datos a su estado inicial:

```bash
cd infra/tirgo_db_stack

docker compose down -v
docker compose up -d
```

Este proceso:

* Elimina todos los datos persistidos
* Reejecuta `mongo-init/001-init.js`
* Recrea:

  * Usuario `tirgo_user`
  * Índices
  * Medicamentos de ejemplo

---

## 9. Uso de mongo-express

### Acceso

* URL:

```text
http://localhost:8081
```

* Autenticación básica (credenciales definidas en `docker-compose.yml`)

### Acceso directo a la colección de pacientes

```text
http://localhost:8081/db/tirgo/pacientes
```

Permite:

* Consultar pacientes
* Editar registros
* Insertar documentos de prueba

---

## 10. Consideraciones de seguridad

Este stack está diseñado para **desarrollo**, pero incorpora medidas básicas:

1. **Exposición limitada**

   * MongoDB y mongo-express solo están accesibles desde `localhost`

2. **Separación de usuarios**

   * Usuario `admin` → administración
   * Usuario `tirgo_user` → acceso de aplicación

3. **Persistencia controlada**

   * Los datos sobreviven reinicios, pero pueden resetearse explícitamente

---

## 11. Resumen

* `tirgo_db_stack` proporciona la infraestructura de base de datos para TirGoPharma
* MongoDB persistente, inicializada con usuarios, índices y datos mínimos
* Consola web disponible para inspección durante desarrollo
* Diseñado para ser **predecible, reproducible y fácil de resetear**

Este stack garantiza que el sistema TirGoPharma disponga siempre de una
**base de datos coherente y controlada** durante el desarrollo e integración.
