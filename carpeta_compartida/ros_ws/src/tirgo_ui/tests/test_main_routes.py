# tests/test_main_routes.py
import sys
from pathlib import Path

import pytest
from flask import Flask

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import main as main_mod
from tirgo_ui.routes import leer as leer_mod
from tirgo_ui.routes import diagnostico as diag_mod
from tirgo_ui.routes import consultar as consultar_mod
from tirgo_ui import session, rosio


@pytest.fixture(autouse=True)
def fake_templates(monkeypatch):
    """
    Evita depender de los HTML reales en main/leer/diagnostico/consultar.
    """
    def _fake_render(template_name, **ctx):
        return f"tpl:{template_name} ctx:{ctx}"

    monkeypatch.setattr(main_mod, "render_template", _fake_render)
    monkeypatch.setattr(leer_mod, "render_template", _fake_render, raising=False)
    monkeypatch.setattr(diag_mod, "render_template", _fake_render, raising=False)
    monkeypatch.setattr(consultar_mod, "render_template", _fake_render, raising=False)


@pytest.fixture
def app():
    app = Flask("test_main_routes")
    app.secret_key = "testing"
    app.testing = True

    # Registramos blueprints principales
    app.register_blueprint(main_mod.bp)
    app.register_blueprint(leer_mod.bp)
    app.register_blueprint(diag_mod.bp)
    app.register_blueprint(consultar_mod.bp)

    return app


@pytest.fixture
def client(app):
    return app.test_client()


def test_index_without_session_200(client, monkeypatch):
    """
    Si no hay sesión activa, index debe mostrar la pantalla de espera de hotword.
    """
    monkeypatch.setattr(session, "is_active", lambda: False)
    monkeypatch.setattr(rosio, "conv_state", lambda: "idle")

    resp = client.get("/")
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    assert "tpl:await_hotword.html" in body


def test_index_with_session_200(client, monkeypatch):
    """
    Con sesión activa, index debe mostrar el menú principal.
    """
    monkeypatch.setattr(session, "is_active", lambda: True)
    monkeypatch.setattr(rosio, "conv_state", lambda: "idle")

    resp = client.get("/")
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    # main.index en tu proyecto renderiza 'menu.html' cuando hay sesión
    assert "tpl:menu.html" in body
