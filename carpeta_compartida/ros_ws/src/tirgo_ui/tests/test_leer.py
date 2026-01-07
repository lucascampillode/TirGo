# tests/test_leer.py
import sys
from pathlib import Path

import pytest
from flask import Flask

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import leer as leer_mod
from tirgo_ui import storage_mongo, session, rosio


# ==========================
# FIX: render_template fake
# ==========================
@pytest.fixture(autouse=True)
def fake_templates(monkeypatch):
    """
    Evita depender de los HTML reales: devolvemos un string
    tipo 'tpl:leer.html ctx:{...}'.
    """
    def _fake_render(template_name, **ctx):
        return f"tpl:{template_name} ctx:{ctx}"

    monkeypatch.setattr(leer_mod, "render_template", _fake_render)


# ==========================
# App / client
# ==========================
@pytest.fixture
def app():
    app = Flask("test_leer")
    app.secret_key = "testing"
    app.testing = True
    app.register_blueprint(leer_mod.bp)
    return app


@pytest.fixture
def client(app):
    return app.test_client()


# ==========================
# Helpers
# ==========================
def _force_session_active(monkeypatch):
    monkeypatch.setattr(session, "is_active", lambda: True)


# ==========================
# TESTS
# ==========================
def test_leer_get_ok_with_meds(client, monkeypatch):
    """
    GET /leer devuelve 200 y lista medicamentos cuando meds_disponibles()
    trae cosas. Los libres son tipo 'L'.
    """
    _force_session_active(monkeypatch)

    fake_meds = [
        {
            "_id": "mongoid-101",
            "id": 101,
            "nombre": "Paracetamol 1g",
            "tipo": "L",          # ← LIBRE
            "bin_id": 1,
            "stock": 18,
        }
    ]

    # OJO: en leer.py se importa 'meds_disponibles' directamente,
    # así que hay que parchear leer_mod.meds_disponibles
    monkeypatch.setattr(leer_mod, "meds_disponibles", lambda: fake_meds)

    resp = client.get("/leer")
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    assert "tpl:leer.html" in body
    # el contexto lleva la lista de meds y aparece el nombre
    assert "Paracetamol 1g" in body
    assert "'tipo': 'L'" in body  # nos aseguramos de que el tipo es L


def test_leer_stock_zero_shows_error(client, monkeypatch):
    """
    POST /leer con medicamento libre (tipo 'L') pero sin stock:
    dec_stock_if_available devuelve False → status.html con NO_STOCK.
    """
    _force_session_active(monkeypatch)

    # Medicamento LIBRE (L)
    def fake_lookup(med_id):
        return {
            "_id": "mongoid-101",
            "id": med_id,
            "nombre": "Paracetamol 1g",
            "tipo": "L",      # ← LIBRE
            "bin_id": 1,
            "stock": 0,
        }

    # Simulamos que no se puede descontar stock
    def fake_dec_stock(med_id, cantidad):
        assert med_id == 101
        assert cantidad == 1
        return False

    # Igual: en leer.py se importan estas funciones al módulo
    monkeypatch.setattr(leer_mod, "lookup_medicamento_by_id", fake_lookup)
    monkeypatch.setattr(leer_mod, "dec_stock_if_available", fake_dec_stock)

    # No queremos que reviente si intenta publicar errores/estado
    monkeypatch.setattr(rosio, "pub_error", lambda code, msg: None)
    monkeypatch.setattr(rosio, "pub_state", lambda state: None)

    form = {"med_id": "101"}
    resp = client.post("/leer", data=form, follow_redirects=True)
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    # Debe usar la plantilla de estado, no recargar leer.html
    assert "tpl:status.html" in body
    # El estado que le pasamos a la plantilla tiene que ser NO_STOCK
    assert "NO_STOCK" in body
    # El nombre del medicamento también debe aparecer
    assert "Paracetamol 1g" in body
