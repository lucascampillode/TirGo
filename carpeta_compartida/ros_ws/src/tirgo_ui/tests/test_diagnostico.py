# tests/test_diagnostico.py

import sys
from pathlib import Path

import pytest
from flask import Flask

# ==========================
# CONFIGURAR PYTHONPATH
# ==========================

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import diagnostico as diag_mod
from tirgo_ui.routes.diagnostico import bp as diag_bp
from tirgo_ui.routes.main import bp as main_bp


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

    def pub_state(self, state):
        self.states.append(state)


# ==========================
# FIXTURES
# ==========================

@pytest.fixture
def app(monkeypatch):
    """
    App mínima de Flask con:
      - blueprint main (real) -> para que url_for('main.index') funcione
      - blueprint diagnostico (real)
    Además, monkeypatcheamos render_template dentro de diagnostico.py
    para no depender de Jinja ni de ficheros html reales.
    """
    app = Flask(__name__)
    app.secret_key = "testing-secret-key"

    # Registramos el main real para que existan las rutas 'main.index'
    app.register_blueprint(main_bp)
    # Registramos el blueprint de diagnóstico
    app.register_blueprint(diag_bp)

    return app


@pytest.fixture
def client(app):
    return app.test_client()


@pytest.fixture
def fake_env(monkeypatch):
    """
    Inyecta DummySession, DummyRosio y un render_template fake
    dentro del módulo diagnostico.
    Devuelve (session, rosio) para que los tests puedan inspeccionarlos.
    """
    session = DummySession()
    rosio = DummyRosio()

    def fake_render_template(template_name, **context):
        """
        Devuelve un texto plano que incluye:
        - nombre de la plantilla
        - campo msg si existe
        """
        pieces = [f"TEMPLATE:{template_name}"]
        msg = context.get("msg")
        if msg:
            pieces.append(str(msg))
        return "\n".join(pieces).encode("utf-8")

    monkeypatch.setattr(diag_mod, "session", session)
    monkeypatch.setattr(diag_mod, "rosio", rosio)
    monkeypatch.setattr(diag_mod, "render_template", fake_render_template)

    return session, rosio


# ==========================
# TESTS /diagnostico
# ==========================

def test_diagnostico_get_activa_sesion_pub_state_y_renderiza(client, fake_env):
    """
    GET /diagnostico con sesión activa:
    - responde 200
    - usa plantilla diagnostico.html
    - publica estado DIAG_START
    """
    session, rosio = fake_env

    resp = client.get("/diagnostico")
    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:diagnostico.html" in data
    assert "DIAG_START" in rosio.states


def test_diagnostico_get_sesion_inactiva_redirige_menu(client, fake_env):
    """
    GET /diagnostico con sesión inactiva:
    - redirige a main.index (302)
    """
    session, rosio = fake_env
    session.active = False

    resp = client.get("/diagnostico", follow_redirects=False)
    assert resp.status_code == 302
    # Location debe apuntar al índice ('/')
    assert resp.headers["Location"].endswith("/")


def test_diagnostico_post_finalizar_termina_sesion_y_redirige(client, fake_env):
    """
    POST /diagnostico con decision=finalizar:
    - hace end_session()
    - redirige a main.index
    """
    session, rosio = fake_env

    resp = client.post(
        "/diagnostico",
        data={"decision": "finalizar"},
        follow_redirects=False,
    )

    assert resp.status_code == 302
    assert session.ended is True
    assert resp.headers["Location"].endswith("/")


def test_diagnostico_post_otro_valor_vuelve_a_mostrar_vista(client, fake_env):
    """
    POST /diagnostico con otra decision (o vacía):
    - NO termina la sesión
    - renderiza diagnostico.html con msg='Acción registrada'
    """
    session, rosio = fake_env

    resp = client.post(
        "/diagnostico",
        data={"decision": "otra_cosa"},
        follow_redirects=True,
    )

    assert resp.status_code == 200
    data = resp.data

    assert b"TEMPLATE:diagnostico.html" in data
    assert b"Acci\xc3\xb3n registrada" in data
    assert session.ended is False
