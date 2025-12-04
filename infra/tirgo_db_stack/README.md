# Infraestructura de base de datos de TirGoPharma  
## Stack Docker `infra/tirgo_db_stack`

Este directorio define la infraestructura de base de datos utilizada por el sistema TirGoPharma.  
El objetivo de este stack es proporcionar un servicio **MongoDB** reproducible, aislado a nivel de red y con datos persistentes, junto con una interfaz web de administración (**mongo-express**) para inspeccionar y editar los registros clínicos utilizados por la aplicación.

La infraestructura está pensada para funcionar en el entorno de desarrollo del proyecto, en combinación con el contenedor principal `ros1_rob_tirgo` (que ejecuta ROS, la interfaz web `tirgo_ui` y el resto de nodos de TirGoPharma).

---

## 1. Estructura del directorio

```text
infra/tirgo_db_stack/
  ├─ .env                       # Variables de entorno del stack (contraseñas, bind, etc.)
  ├─ docker-compose.yml         # Definición de servicios Docker (Mongo + mongo-express)
  └─ mongo-init/
      └─ 001-init.js            # Script de inicialización de la BD "tirgo"
````

### 1.1 Fichero `.env`

El fichero `.env` contiene las variables de configuración sensibles del stack:

```env
MONGO_ROOT_PASSWORD=superseguro
TIRGO_DB_PASSWORD=tirgo_pass_cambia
# OJO: atamos los puertos al loopback para no exponerlos a la red
MONGO_BIND_HOST=127.0.0.1
```

* `MONGO_ROOT_PASSWORD`
  Contraseña del usuario **administrador** de MongoDB (`admin`), con permisos de administración sobre el servidor.

* `TIRGO_DB_PASSWORD`
  Contraseña del usuario de aplicación `tirgo_user`, que se crea en el script `mongo-init/001-init.js` y que es el utilizado por la interfaz web `tirgo_ui` para acceder a la base de datos de trabajo.

* `MONGO_BIND_HOST`
  Comentario recordatorio de que los puertos se exponen únicamente en la interfaz de loopback (`127.0.0.1`), de forma que la BD solo es accesible localmente.

> **Nota de seguridad:** los valores presentes en este fichero están pensados para **entornos de desarrollo**. En un despliegue real deben sustituirse por contraseñas robustas y almacenarse en ficheros `.env` no versionados.

---

## 2. Definición del stack Docker

El archivo `docker-compose.yml` define dos servicios y un volumen persistente:

* Servicio `mongo` → servidor MongoDB 7.
* Servicio `mongo-express` → consola web de administración.
* Volumen `mongo_data` → almacenamiento persistente de los datos.

### 2.1 Servicio `mongo` (servidor de base de datos)

* Se utiliza la imagen oficial `mongo:7`.
* El usuario administrador (`admin`) y la base de datos por defecto (`tirgo`) se configuran mediante variables de entorno.
* El volumen `mongo_data` se monta en `/data/db` para garantizar la persistencia de los datos entre reinicios.
* La carpeta `mongo-init/` se monta en modo solo lectura en `/docker-entrypoint-initdb.d`, de manera que cualquier script `.js` presente se ejecuta automáticamente **solo la primera vez** que el contenedor arranca con un volumen de datos vacío.
* El puerto 27017 se expone exclusivamente en `127.0.0.1`, de forma que la BD solo es accesible desde la propia máquina host (y, por extensión, desde el contenedor `ros1_rob_tirgo`, que se ejecuta en `network_mode: host`).

### 2.2 Servicio `mongo-express` (consola web)

* Proporciona una interfaz web para explorar y editar la BD.
* Se conecta al servidor `mongo` utilizando el usuario administrador.
* Se configura un mecanismo de autenticación básica (usuario `TirgoAdmin`, contraseña `tirgo`) para proteger el acceso a la consola.
* La consola se expone únicamente en `http://localhost:8081`, de nuevo limitada a la máquina local.

### 2.3 Volumen `mongo_data`

Al final del fichero se define el volumen:

```yaml
volumes:
  mongo_data:
```

Este volumen almacena físicamente todos los datos de la BD.
En caso de eliminar el volumen (`docker compose down -v`), Mongo arrancará “en blanco” y volverá a ejecutar los scripts de inicialización.

---

## 3. Proceso de inicialización de la base de datos

La lógica de inicialización reside en `mongo-init/001-init.js`:

La secuencia es la siguiente:

1. **Selección de la base de datos de aplicación**
   Se trabaja explícitamente sobre la base de datos `tirgo`:

   ```js
   const appDb = db.getSiblingDB('tirgo');
   ```

2. **Creación del usuario de aplicación**
   Se crea el usuario `tirgo_user` con permisos de lectura y escritura sobre la BD `tirgo`:

   Este usuario es el que debe utilizar la aplicación `tirgo_ui` en su cadena de conexión (`MONGO_URI`).

3. **Definición de índices**
   Se crean índices para garantizar unicidad y eficiencia en las consultas:

   * `pacientes.dni_hash`: índice único → evita duplicados de paciente.
   * `medicamentos.nombre`: índice único → evita duplicados de medicamento.
   * `recetas`: índices compuestos sobre `(paciente_id, activa)` y `(medicamento_id, activa)`, optimizando las consultas de “recetas activas” por paciente o por medicamento.

4. **Datos de medicamentos de ejemplo**
   Se insertan (si no existían) dos medicamentos básicos mediante operaciones `upsert`:

   * **Amoxicilina 500mg** (requiere receta).
   * **Ibuprofeno 400mg** (no requiere receta).

   Esto permite que la aplicación tenga datos mínimos con los que trabajar desde el primer arranque.

> La creación de pacientes reales, recetas, dispensaciones y logs se delega en la aplicación `tirgo_ui` y en los scripts de “seed” específicos; el script de infra se limita a establecer la estructura base y algunos datos de referencia.

---

## 4. Integración con el sistema TirGoPharma

La BD levantada por este stack está diseñada para ser el backend de datos del sistema TirGoPharma. En particular:

* La aplicación web `tirgo_ui` se ejecuta en el contenedor `ros1_rob_tirgo` y se conecta a la BD `tirgo` utilizando el usuario `tirgo_user`.
* Las colecciones principales utilizadas por TirGoPharma incluyen:

  * `pacientes`: información identificativa (hasheada), nombre, apellidos, etc.
  * `medicamentos`: catálogo de fármacos y propiedades (si requieren receta, etc.).
  * `recetas`: prescripciones activas/inactivas asociadas a pacientes y medicamentos.
  * `dispensaciones`: registro de los actos de dispensación realizados por el sistema robótico.
  * `logs`: trazas de eventos relevantes (estados de misión, errores, etc.).

La infraestructura de base de datos que se describe aquí proporciona:

* **Persistencia** de todos estos datos entre sesiones de trabajo.
* **Aislamiento de red**, al exponer los servicios únicamente en `localhost`.
* **Facilidad de inspección** mediante la consola mongo-express.

---

## 5. Puesta en marcha del stack

### 5.1 Requisitos previos

* Docker y Docker Compose v2 instalados en el sistema.
* Clonado del repositorio TirGoPharma en el sistema local.

### 5.2 Arranque del stack

1. Posicionarse en el directorio del stack:

   ```bash
   cd infra/tirgo_db_stack
   ```

2. Revisar el fichero `.env` y, si es necesario, ajustar las contraseñas para el entorno local.

3. Levantar los servicios en segundo plano:

   ```bash
   docker compose up -d
   ```

4. Verificar que los contenedores se están ejecutando correctamente:

   ```bash
   docker ps | grep tirgo_mongo
   docker ps | grep tirgo_mongo_express
   ```

5. Consultar los logs de Mongo si se desea comprobar la ejecución del script de inicialización:

   ```bash
   docker logs tirgo_mongo
   ```

Tras estos pasos, la base de datos `tirgo` estará disponible en `mongodb://localhost:27017` y lista para ser utilizada por la aplicación.

---

## 6. Acceso y edición de datos con mongo-express

Para facilitar la inspección y edición manual de los datos durante las pruebas, se ha incluido el servicio `mongo-express`, accesible desde el navegador.

### 6.1 Acceso a la consola

* URL de la consola general:

  ```text
  http://localhost:8081
  ```

* Credenciales de acceso (autenticación básica de la consola)

### 6.2 Edición de la colección de pacientes

En el contexto del proyecto, una operación frecuente es la inspección y modificación de los registros de pacientes. Mongo-express permite acceder directamente a la colección `pacientes` de la base de datos `tirgo` mediante la siguiente URL:

```text
http://localhost:8081/db/tirgo/pacientes
```

Desde esa vista se pueden:

* Consultar todos los pacientes registrados.
* Editar registros individuales (por ejemplo, nombre, apellidos o `dni_hash`).
* Crear nuevos documentos de paciente para las pruebas de integración.

---

## 7. Reset y recreación de la base de datos

En algunos escenarios (por ejemplo, antes de una demostración o tras una batería de pruebas), puede interesar restaurar la BD a su estado inicial.

Procedimiento:

```bash
cd infra/tirgo_db_stack

# 1) Parar servicios y eliminar contenedores + volumen
docker compose down -v

# 2) Volver a levantar el stack desde cero
docker compose up -d
```

Al eliminar el volumen `mongo_data`:

* Se borran todos los datos anteriores de `tirgo`.
* En el siguiente arranque, Mongo ejecuta de nuevo `mongo-init/001-init.js`, recreando:

  * El usuario `tirgo_user`.
  * Los índices de `pacientes`, `medicamentos` y `recetas`.
  * Los medicamentos de ejemplo (Amoxicilina e Ibuprofeno).

La aplicación `tirgo_ui` y los scripts de seed pueden entonces volver a poblar la base de datos con pacientes, recetas y dispensaciones de prueba.

---

## 8. Consideraciones de seguridad

Aunque este stack está orientado a un entorno de desarrollo, se han incorporado algunas medidas básicas:

1. **Exposición limitada de puertos**
   Tanto Mongo (27017) como mongo-express (8081) están publicados únicamente en la interfaz `127.0.0.1`. Ningún servicio de la base de datos queda accesible desde la red local o externa por defecto.

2. **Separación de roles de usuario**

   * Usuario `admin` (root de Mongo): reservado a tareas de administración y utilizado por mongo-express.
   * Usuario `tirgo_user`: usuario de aplicación, con rol `readWrite` únicamente sobre la base de datos `tirgo`.

3. **Autenticación en la consola web**
   El acceso a mongo-express está protegido mediante usuario y contraseña propios, que deben modificarse en entornos distintos al de desarrollo.

4. **Gestión de credenciales**
   En una versión productiva, se recomienda:

   * No versionar el fichero `.env` con las contraseñas reales.
   * Emplear gestores de secretos o variables de entorno específicas del entorno de despliegue.
