# tests/test_web_ros_integration.py
import sys
from pathlib import Path

import pytest
from flask import Flask

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import leer as leer_mod
from tirgo_ui import storage_mongo, session, rosio


@pytest.fixture(autouse=True)
def fake_templates(monkeypatch):
    """
    Igual que en test_leer: plantillas fake.
    """
    def _fake_render(template_name, **ctx):
        return f"tpl:{template_name} ctx:{ctx}"

    monkeypatch.setattr(leer_mod, "render_template", _fake_render)


@pytest.fixture
def app():
    app = Flask("test_web_ros_integration")
    app.secret_key = "testing"
    app.testing = True
    app.register_blueprint(leer_mod.bp)
    return app


@pytest.fixture
def client(app):
    return app.test_client()


def _force_session_active(monkeypatch):
    monkeypatch.setattr(session, "is_active", lambda: True)

def test_leer_calls_start_mission_async_with_valid_med(client, monkeypatch):
    """
    Flujo feliz:
    - med tipo 'L' (libre)
    - hay stock (dec_stock_if_available → True)
    - debe llamar a start_mission_async("", bin_id)
    """
    _force_session_active(monkeypatch)

    def fake_lookup(med_id):
        return {
            "_id": "mongoid-202",
            "id": med_id,
            "nombre": "Ibuprofeno 400 mg",
            "tipo": "L",   # ← LIBRE
            "bin_id": 2,
            "stock": 20,
        }

    def fake_dec_stock(med_id, cantidad):
        assert med_id == 202
        assert cantidad == 1
        return True

    # IMPORTANTE: parchear funciones del propio módulo leer_mod
    monkeypatch.setattr(leer_mod, "lookup_medicamento_by_id", fake_lookup)
    monkeypatch.setattr(leer_mod, "dec_stock_if_available", fake_dec_stock)

    calls = []

    def fake_start_mission_async(patient_id, bin_id):
        calls.append((patient_id, bin_id))

    monkeypatch.setattr(rosio, "start_mission_async", fake_start_mission_async)
    monkeypatch.setattr(rosio, "pub_state", lambda state: None)

    resp = client.post("/leer", data={"med_id": "202"}, follow_redirects=True)
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    assert "tpl:status.html" in body
    assert "DISPENSING" in body
    # Debe haberse llamado una vez con patient_id vacío y bin_id=2
    assert calls == [("", 2)]


def test_leer_handles_start_mission_async_exception(client, monkeypatch):
    """
    Si start_mission_async revienta, la vista no debe 500ear.
    Para simplificar, comprobamos que devuelve alguna respuesta 200.
    """
    _force_session_active(monkeypatch)

    def fake_lookup(med_id):
        return {
            "_id": "mongoid-404",
            "id": med_id,
            "nombre": "Omeprazol 20 mg",
            "tipo": "L",   # ← LIBRE
            "bin_id": 4,
            "stock": 20,
        }

    def fake_dec_stock(med_id, cantidad):
        return True

    def fake_start_mission_async(patient_id, bin_id):
        raise RuntimeError("TIAGo no responde")

    monkeypatch.setattr(storage_mongo, "lookup_medicamento_by_id", fake_lookup)
    monkeypatch.setattr(storage_mongo, "dec_stock_if_available", fake_dec_stock)
    monkeypatch.setattr(rosio, "start_mission_async", fake_start_mission_async)
    monkeypatch.setattr(rosio, "pub_state", lambda state: None)
    monkeypatch.setattr(rosio, "pub_error", lambda code, msg: None)

    resp = client.post("/leer", data={"med_id": "404"}, follow_redirects=True)

    # La vista atrapa la excepción y redirige de vuelta a /leer
    assert resp.status_code == 200 or resp.status_code == 302
