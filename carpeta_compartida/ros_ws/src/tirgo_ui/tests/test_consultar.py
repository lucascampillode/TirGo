import os
import sys
from pathlib import Path

import pytest
from flask import Flask, get_flashed_messages
from unittest.mock import MagicMock, patch  # por si luego quieres ampliar tests

# ==========================
# CONFIGURAR PYTHONPATH
# ==========================

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

# Importamos el módulo real de la ruta
from tirgo_ui.routes import consultar as consultar_mod
from tirgo_ui.routes.consultar import (
    _normalize_dni,
    _is_valid_dni,
    _normalize_name_for_compare,
    _is_valid_name,
    bp as consultar_bp,  # blueprint real
)

# ==========================
# DOBLES (FAKES / STUBS)
# ==========================


class DummyStore:
    """
    Fake de storage_mongo para no tocar la BD real.
    """

    def __init__(self):
        # diccionario DNI normalizado -> doc paciente
        self.pacientes_by_dni = {}
        # id paciente -> lista de meds
        self.meds_by_pac_id = {}

    def find_paciente_by_dni(self, dni):
        return self.pacientes_by_dni.get(dni)

    def create_paciente_if_allowed(self, nombre, apellidos, dni):
        if dni in self.pacientes_by_dni:
            raise ValueError("Paciente ya existe")
        pac = {
            "_id": f"pac_{dni}",
            "nombre": nombre,
            "apellidos": apellidos,
            "dni": dni,
        }
        self.pacientes_by_dni[dni] = pac
        return pac

    def permitted_meds_for_patient(self, pac_id):
        # Fallback que usa _get_meds_for_patient cuando no hay DB directa
        return self.meds_by_pac_id.get(pac_id, [])


class DummyRosio:
    """
    Fake de rosio para registrar las llamadas a pub_state().
    """

    def __init__(self):
        self.states = []

    def pub_state(self, state):
        self.states.append(state)


# ==========================
# FIXTURES
# ==========================


@pytest.fixture
def app():
    """
    Creamos una app mínima de Flask y registramos el blueprint real.
    No usamos las plantillas reales: las vamos a fakear con monkeypatch.
    """
    app = Flask(__name__)
    app.secret_key = "testing-secret-key"
    app.register_blueprint(consultar_bp)
    return app


@pytest.fixture
def client(app):
    return app.test_client()


@pytest.fixture
def fake_env(monkeypatch):
    """
    Inyecta DummyStore, DummyRosio y un render_template fake
    dentro del módulo consultar.
    Devuelve (store, rosio) para que los tests puedan inspeccionarlos.
    """
    store = DummyStore()
    rosio = DummyRosio()

    def fake_render_template(template_name, **context):
        """
        Simula los templates devolviendo un texto plano que incluye:
        - nombre de la plantilla
        - mensajes flash
        - algunos campos del contexto útiles para los asserts
        """
        pieces = [f"TEMPLATE:{template_name}"]

        # Añadimos mensajes flash
        flashes = get_flashed_messages()
        pieces.extend(flashes)

        # Algunos campos típicos de contexto que nos interesan
        # (ajusta nombres si en consultar.py usas otros)
        for key in ("dni", "dni_normalizado", "dni_input"):
            if key in context and context[key]:
                pieces.append(str(context[key]))

        paciente = context.get("paciente")
        if paciente:
            pieces.append(str(paciente.get("nombre", "")))
            pieces.append(str(paciente.get("apellidos", "")))

        meds = context.get("meds") or context.get("medicamentos")
        if meds:
            for med in meds:
                nombre_med = med.get("nombre", None) if isinstance(med, dict) else str(med)
                if nombre_med:
                    pieces.append(str(nombre_med))

        # Mensajes específicos que puedas pasar en el contexto
        for key in ("mensaje", "mensaje_ok", "mensaje_error"):
            if key in context and context[key]:
                pieces.append(str(context[key]))

        # Devolvemos un string que contenga todo eso
        return "\n".join(pieces)

    # Inyectamos los dobles en el módulo real
    monkeypatch.setattr(consultar_mod, "store", store)
    monkeypatch.setattr(consultar_mod, "rosio", rosio)
    monkeypatch.setattr(consultar_mod, "render_template", fake_render_template)

    return store, rosio


# ==========================
# TESTS CONSULTAR
# ==========================


def test_consultar_dni_vacio_redirige_con_error(client, fake_env):
    """
    DNI vacío -> flash de error y redirect a /consultar/.
    """
    resp = client.post(
        "/consultar/",
        data={"nombre": "", "apellidos": "", "dni": ""},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert b"Debes introducir un DNI para realizar la consulta." in resp.data


def test_consultar_dni_formato_invalido(client, fake_env):
    """
    DNI con formato malo (o letra incorrecta) -> error.
    """
    resp = client.post(
        "/consultar/",
        data={"nombre": "Katrin", "apellidos": "Munoz", "dni": "1234ABC"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"DNI no v\xc3\xa1lido. Debe tener 8 n\xc3\xbameros y una letra correcta."
        in resp.data
    )


def test_consultar_nombre_invalido(client, fake_env):
    """
    Nombre con números/símbolos -> error antes de consultar.
    """
    resp = client.post(
        "/consultar/",
        data={"nombre": "K4tr1n", "apellidos": "Munoz", "dni": "12345678Z"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"El nombre solo puede contener letras, espacios, ap\xc3\xb3strofos y guiones"
        in resp.data
    )


def test_consultar_apellidos_invalidos(client, fake_env):
    """
    Apellidos con números/símbolos -> error.
    """
    resp = client.post(
        "/consultar/",
        data={"nombre": "Katrin", "apellidos": "Mun0z!!", "dni": "12345678Z"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"Los apellidos solo pueden contener letras, espacios, ap\xc3\xb3strofos y guiones"
        in resp.data
    )


def test_consultar_paciente_inexistente_muestra_pantalla_crear(client, fake_env):
    """
    Paciente inexistente -> plantilla 'no existente'.
    (Comprobamos que aparece el DNI normalizado en la respuesta).
    """
    resp = client.post(
        "/consultar/",
        data={"nombre": "Katrin", "apellidos": "Munoz", "dni": "12345678z"},
    )

    assert resp.status_code == 200
    # el DNI se normaliza a mayúsculas y sin espacios/guiones
    assert b"12345678Z" in resp.data


def test_consultar_mismatch_nombre_apellidos_da_error(client, fake_env):
    """
    Mismo DNI pero nombre/apellidos que no coinciden → error + PATIENT_DATA_MISMATCH.
    """
    store, rosio = fake_env

    # Paciente guardado en la "BD"
    dni = "12345678Z"
    store.pacientes_by_dni[dni] = {
        "_id": "pac_1",
        "nombre": "Katrin",
        "apellidos": "Munoz",
        "dni": dni,
    }

    resp = client.post(
        "/consultar/",
        data={"nombre": "Otra", "apellidos": "Persona", "dni": dni},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"Los datos introducidos (nombre/apellidos) no coinciden con el DNI registrado."
        in resp.data
    )
    assert "PATIENT_DATA_MISMATCH" in rosio.states


def test_consultar_nombre_normalizado_aceptado(client, fake_env):
    """
    Mismo DNI, pero nombre con mayúsculas/minúsculas/espacios distintos → aceptado.
    """
    store, rosio = fake_env

    dni = "49582367W"   # DNI real que aceptas en tu sistema
    pac_id = "pac_1"
    store.pacientes_by_dni[dni] = {
        "_id": pac_id,
        "nombre": "Katrin",
        "apellidos": "Muñoz",   # tal y como está en la BD
        "dni": dni,
    }
    # Simulamos que tiene una receta activa
    store.meds_by_pac_id[pac_id] = [
        {"id": 101, "nombre": "Paracetamol 1g"}
    ]

    resp = client.post(
        "/consultar/",
        data={
            # nombre con espacios y casing raros
            "nombre": "   kaTrin   ",
            # apellidos con mayúsculas y espacios de más
            "apellidos": "   MUÑOZ    ",
            # DNI en minúscula o con la misma forma, se normaliza dentro
            "dni": "49582367w",
        },
        follow_redirects=True,
    )

    assert resp.status_code == 200
    # Debería mostrarse el nombre real del paciente y algún medicamento
    assert b"Katrin" in resp.data
    assert b"Paracetamol" in resp.data
    assert "PATIENT_FOUND" in rosio.states or "NO_RECIPES" in rosio.states


def test_consultar_dni_con_espacios_y_guiones_se_normaliza(client, fake_env):
    """
    El DNI con espacios/guiones se normaliza correctamente antes de buscar.
    """
    store, _ = fake_env

    dni_normal = "12345678Z"
    store.pacientes_by_dni[dni_normal] = {
        "_id": "pac_1",
        "nombre": "Katrin",
        "apellidos": "Munoz",
        "dni": dni_normal,
    }

    resp = client.post(
        "/consultar/",
        data={
            "nombre": "",
            "apellidos": "",
            "dni": "1234 5678-z",
        },
    )

    assert resp.status_code == 200
    # si ha encontrado al paciente, debería aparecer su nombre
    assert b"Katrin" in resp.data


# ==========================
# TESTS CREAR PACIENTE
# ==========================


def test_crear_paciente_nombre_invalido(client, fake_env):
    """
    Crear paciente con nombre inválido -> error.
    """
    resp = client.post(
        "/consultar/crear_paciente",
        data={"nombre": "1234", "apellidos": "Munoz", "dni": "12345678Z"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"El nombre solo puede contener letras, espacios, ap\xc3\xb3strofos y guiones"
        in resp.data
    )


def test_crear_paciente_dni_invalido(client, fake_env):
    """
    Crear paciente sin DNI válido -> error.
    """
    resp = client.post(
        "/consultar/crear_paciente",
        data={"nombre": "Katrin", "apellidos": "Munoz", "dni": "BAD"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert (
        b"DNI no v\xc3\xa1lido. Debe tener 8 n\xc3\xbameros y una letra correcta."
        in resp.data
    )


def test_crear_paciente_duplicado_por_dni(client, fake_env):
    """
    Si ya existe un paciente con ese DNI, no se permite crear otro.
    """
    store, rosio = fake_env

    dni = "12345678Z"
    store.pacientes_by_dni[dni] = {
        "_id": "pac_1",
        "nombre": "Katrin",
        "apellidos": "Munoz",
        "dni": dni,
    }

    resp = client.post(
        "/consultar/crear_paciente",
        data={"nombre": "Otra", "apellidos": "Persona", "dni": dni},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    assert b"Ya existe un paciente registrado con ese DNI." in resp.data
    assert "PATIENT_DUPLICATE_DNI" in rosio.states


def test_crear_paciente_ok_y_luego_ver_resultado(client, fake_env):
    """
    Flujo feliz de creación de paciente:
    - DNI válido y libre
    - Nombre y apellidos válidos
    -> Paciente creado y se muestran (o no) sus recetas.
    """
    store, rosio = fake_env

    dni = "12345678Z"

    # No hay paciente inicialmente
    assert dni not in store.pacientes_by_dni

    # Además simulamos que luego tendrá una receta activa
    # (usando el id que generará DummyStore)
    expected_pac_id = f"pac_{dni}"
    store.meds_by_pac_id[expected_pac_id] = [{"id": 101, "nombre": "Paracetamol 1g"}]

    resp = client.post(
        "/consultar/crear_paciente",
        data={"nombre": "Katrin", "apellidos": "Munoz", "dni": dni},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    # Se ha creado en la "BD"
    assert dni in store.pacientes_by_dni
    assert b"Paciente creado correctamente" in resp.data
    assert b"Katrin" in resp.data
    assert b"Paracetamol" in resp.data
    assert "PATIENT_CREATED" in rosio.states
