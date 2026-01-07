# tests/test_diagnostico.py
import sys
from pathlib import Path

import pytest
from flask import Flask

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import diagnostico as diag_mod
from tirgo_ui import session as session_mod, rosio as rosio_mod


# ==========================
# FIXTURE: Flask client
# ==========================

@pytest.fixture
def client():
    app = Flask("test_diagnostico")
    app.config["TESTING"] = True
    app.secret_key = "testing-secret"

    app.register_blueprint(diag_mod.bp)
    return app.test_client()


# ==========================
# TESTS
# ==========================

def test_diagnostico_without_session_redirects_to_index(client, monkeypatch):
    """
    Si no hay sesión activa, /diagnostico debe redirigir a 'main.index'.
    En el test parcheamos url_for para que devuelva '/' y evitar el BuildError.
    """
    from tirgo_ui.routes import diagnostico as diagnostico_mod
    from tirgo_ui import session

    # No hay sesión activa
    monkeypatch.setattr(session, "is_active", lambda: False)

    # Evitar BuildError de url_for('main.index')
    monkeypatch.setattr(diagnostico_mod, "url_for", lambda endpoint, **kwargs: "/")

    resp = client.get("/diagnostico")

    # Es suficiente con comprobar que es una redirección
    assert resp.status_code in (302, 308)
    assert resp.headers["Location"].endswith("/")

def test_diagnostico_with_session_publishes_diag_start_once(client, monkeypatch):
    """
    Con sesión activa:
    - publica DIAG_START una vez
    - devuelve 200
    - no rompe por falta de plantilla
    """
    monkeypatch.setattr(session_mod, "is_active", lambda: True, raising=False)

    calls = []

    def fake_pub_state(state):
        calls.append(state)

    monkeypatch.setattr(rosio_mod, "pub_state", fake_pub_state, raising=False)

    # Evitar TemplateNotFound
    monkeypatch.setattr(
        diag_mod,
        "render_template",
        lambda *a, **k: "OK DIAG",
        raising=False,
    )

    resp = client.get("/diagnostico", follow_redirects=True)

    assert resp.status_code == 200
    assert calls == ["DIAG_START"]


def test_diagnostico_post_finalizar_ends_session(client, monkeypatch):
    """
    Si hay sesión activa y se hace POST con decision='finalizar',
    debe llamarse a session.end_session() y redirigir a main.index.
    """
    from tirgo_ui.routes import diagnostico as diagnostico_mod
    from tirgo_ui import session

    # Hay sesión activa
    monkeypatch.setattr(session, "is_active", lambda: True)

    ended = {"value": False}

    def fake_end_session():
        ended["value"] = True

    monkeypatch.setattr(session, "end_session", fake_end_session)

    # Evitar BuildError igual que antes
    monkeypatch.setattr(diagnostico_mod, "url_for", lambda endpoint, **kwargs: "/")

    resp = client.post("/diagnostico", data={"decision": "finalizar"})

    assert resp.status_code in (302, 308)
    assert ended["value"] is True
    assert resp.headers["Location"].endswith("/")
