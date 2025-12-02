# tests/test_leer.py

import sys
from pathlib import Path

import pytest
from flask import Flask, get_flashed_messages

# ==========================
# CONFIGURAR PYTHONPATH
# ==========================

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import leer as leer_mod
from tirgo_ui.routes.leer import bp as leer_bp


# ==========================
# DOBLES / STUBS
# ==========================

class DummySession:
    def __init__(self):
        self.active = True
        self.ended = False

    def is_active(self):
        return self.active

    def end_session(self):
        self.ended = True
        self.active = False


class DummyRosio:
    def __init__(self):
        self.states = []
        self.errors = []
        self.missions = []
        self.status_msgs = []
        self.mission_status_data = {"state": "IDLE"}

    def pub_state(self, state):
        self.states.append(state)

    def pub_error(self, code, msg):
        self.errors.append((code, msg))

    def start_mission_async(self, patient_id, bin_id):
        self.missions.append({"patient_id": patient_id, "bin_id": bin_id})

    def pub_status(self, obj):
        self.status_msgs.append(obj)

    def get_mission_status(self):
        return self.mission_status_data


# ==========================
# FIXTURES
# ==========================

@pytest.fixture
def app():
    """
    App mínima con el blueprint real de leer.
    Las plantillas se fingean con monkeypatch en fake_env.
    """
    app = Flask(__name__)
    app.secret_key = "testing-secret-key"
    app.register_blueprint(leer_bp)
    return app


@pytest.fixture
def client(app):
    return app.test_client()


@pytest.fixture
def fake_env(monkeypatch):
    """
    Inyecta:
      - DummySession como session
      - DummyRosio como rosio
      - funciones fake de storage_mongo
      - render_template fake que devuelve texto plano
    Devuelve (session, rosio) para inspeccionarlos desde los tests.
    """
    session = DummySession()
    rosio = DummyRosio()

    # --- Fake de "BD de medicamentos" en memoria ---
    meds_by_id = {
        1: {"_id": 1, "nombre": "Ibuprofeno 600mg", "tipo": "", "bin_id": 10},
        2: {"_id": 2, "nombre": "Amoxicilina 500mg", "tipo": "R", "bin_id": 20},
    }

    def fake_meds_disponibles():
        return list(meds_by_id.values())

    def fake_lookup_medicamento_by_id(med_id):
        return meds_by_id.get(int(med_id))

    def fake_get_stock(med_id):
        # Por defecto, stock de sobra
        return 5

    def fake_create_paciente_if_allowed(nombre, apellidos, dni):
        # Para estos tests básicos, no usamos el flujo de crear paciente
        return {"_id": "pac_123", "nombre": nombre, "apellidos": apellidos, "dni": dni}

    def fake_paciente_necesita_restringido(pid):
        # Por defecto, no necesita R especial
        return False

    def fake_tiene_receta_activa(pid, med_id):
        # Por defecto, no hay receta activa
        return False

    def fake_h_dni(dni):
        return f"HASH-{dni}"

    def fake_render_template(template_name, **context):
        """
        Simula los templates devolviendo un texto plano que incluye:
        - nombre de la plantilla
        - mensajes flash
        - campos clave del contexto
        """
        pieces = [f"TEMPLATE:{template_name}"]

        # Mensajes flash
        flashes = get_flashed_messages()
        pieces.extend(flashes)

        # Campos habituales de contexto
        for key in ("state", "body_msg", "msg", "mensaje", "mensaje_ok", "mensaje_error"):
            if key in context and context[key]:
                pieces.append(str(context[key]))

        # Medicamentos en contexto
        meds_ctx = context.get("meds") or context.get("medicamentos")
        if meds_ctx:
            for med in meds_ctx:
                nombre_med = med.get("nombre", "") if isinstance(med, dict) else str(med)
                if nombre_med:
                    pieces.append(nombre_med)

        med_obj = context.get("med")
        if med_obj and isinstance(med_obj, dict):
            nombre_med = med_obj.get("nombre")
            if nombre_med:
                pieces.append(nombre_med)

        return "\n".join(pieces).encode("utf-8")

    # Inyectamos en el módulo real
    monkeypatch.setattr(leer_mod, "session", session)
    monkeypatch.setattr(leer_mod, "rosio", rosio)

    monkeypatch.setattr(leer_mod, "meds_disponibles", fake_meds_disponibles)
    monkeypatch.setattr(leer_mod, "lookup_medicamento_by_id", fake_lookup_medicamento_by_id)
    monkeypatch.setattr(leer_mod, "get_stock", fake_get_stock)
    monkeypatch.setattr(leer_mod, "create_paciente_if_allowed", fake_create_paciente_if_allowed)
    monkeypatch.setattr(leer_mod, "paciente_necesita_restringido", fake_paciente_necesita_restringido)
    monkeypatch.setattr(leer_mod, "tiene_receta_activa", fake_tiene_receta_activa)
    monkeypatch.setattr(leer_mod, "h_dni", fake_h_dni)

    monkeypatch.setattr(leer_mod, "render_template", fake_render_template)

    return session, rosio


# ==========================
# TESTS leer_get
# ==========================

def test_leer_get_lista_meds_y_pub_state(client, fake_env):
    """
    GET /leer:
    - debe devolver 200
    - usar plantilla leer.html
    - incluir los nombres de los medicamentos
    - publicar estado MODE_SELECTED
    """
    session, rosio = fake_env

    resp = client.get("/leer")
    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:leer.html" in data
    assert b"Ibuprofeno 600mg" in data
    assert b"Amoxicilina 500mg" in data
    assert "MODE_SELECTED" in rosio.states


def test_leer_get_meds_disponibles_falla_muestra_mensaje_error(client, fake_env, monkeypatch):
    """
    Si meds_disponibles lanza excepción, la vista:
    - registra el error (logger)
    - mete flash "No puedo listar medicamentos ahora mismo."
    - responde 200 con plantilla leer.html y lista vacía.
    """
    session, rosio = fake_env

    def boom():
        raise RuntimeError("DB down")

    monkeypatch.setattr(leer_mod, "meds_disponibles", boom)

    resp = client.get("/leer")
    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:leer.html" in data
    assert b"No puedo listar medicamentos ahora mismo." in data


# ==========================
# TESTS leer_post
# ==========================

def test_leer_post_med_id_invalido(client, fake_env):
    """
    POST /leer con med_id no entero:
    - pub_error BAD_REQ
    - flash 'Selección inválida.'
    - redirect a /leer (seguido por GET /leer con follow_redirects)
    """
    session, rosio = fake_env

    resp = client.post(
        "/leer",
        data={"med_id": "no-es-un-int"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    data = resp.data

    assert b"Selecci\xc3\xb3n inv\xc3\xa1lida." in data
    assert ("BAD_REQ", "Selección inválida") in rosio.errors


def test_leer_post_med_no_existe(client, fake_env, monkeypatch):
    """
    POST /leer con med_id que no existe:
    - pub_error NOT_FOUND
    - flash 'No he encontrado ese medicamento.'
    - redirect a /leer
    """
    session, rosio = fake_env

    def fake_lookup(med_id):
        return None

    monkeypatch.setattr(leer_mod, "lookup_medicamento_by_id", fake_lookup)

    resp = client.post(
        "/leer",
        data={"med_id": "999"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    data = resp.data

    assert b"No he encontrado ese medicamento." in data
    assert ("NOT_FOUND", "Medicamento no existe") in rosio.errors


def test_leer_post_libre_con_stock_lanza_mision(client, fake_env):
    """
    POST /leer con medicamento de tipo libre (no R) y stock > 0:
    - llama a start_mission_async("", bin_id)
    - pub_state('DISPENSING')
    - termina la sesión
    - renderiza status.html con 'Preparando tu producto'
    """
    session, rosio = fake_env

    # Usamos med_id=1 -> tipo "" (libre) según fake_env
    resp = client.post(
        "/leer",
        data={"med_id": "1"},
    )

    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:status.html" in data
    assert b"Preparando tu producto" in data

    # Misión lanzada
    assert len(rosio.missions) == 1
    mission = rosio.missions[0]
    assert mission["patient_id"] == ""
    assert mission["bin_id"] == 10  # bin_id del med_id=1 en fake_env

    # Estado de dispensing
    assert "DISPENSING" in rosio.states
    # Sesión terminada
    assert session.ended is True


def test_leer_post_restringido_pasa_a_identificacion(client, fake_env):
    """
    POST /leer con medicamento tipo R:
    - no lanza misión todavía
    - pub_state('IDENTIFYING')
    - renderiza leer_ident.html con el medicamento en contexto
    """
    session, rosio = fake_env

    # med_id=2 -> tipo 'R'
    resp = client.post(
        "/leer",
        data={"med_id": "2"},
    )

    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:leer_ident.html" in data
    assert b"Amoxicilina 500mg" in data
    assert "IDENTIFYING" in rosio.states
    # No debe haber misiones lanzadas aún
    assert rosio.missions == []


# ==========================
# TEST leer_mission_status
# ==========================

def test_leer_mission_status_devuelve_json(client, fake_env):
    """
    GET /leer/mission_status:
    - debe devolver el JSON que proporciona rosio.get_mission_status()
    """
    session, rosio = fake_env
    rosio.mission_status_data = {"state": "RUNNING", "progress": 42}

    resp = client.get("/leer/mission_status")
    assert resp.status_code == 200

    data = resp.get_json()
    assert data == {"state": "RUNNING", "progress": 42}
