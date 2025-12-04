# tests/test_consultar.py
import sys
from pathlib import Path

import pytest
from flask import Flask

# ---------- Path al src ----------
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

from tirgo_ui.routes import consultar as consultar_mod
from tirgo_ui.routes.consultar import (
    _normalize_dni,
    _is_valid_dni,
    _normalize_name_for_compare,
    _is_valid_name,
)
from tirgo_ui import rosio as rosio_mod
from tirgo_ui.routes import consultar as consultar_mod


# ==========================
# FIXTURE: Flask client
# ==========================

@pytest.fixture
def client(monkeypatch):
    """
    Crea una app mínima solo con el blueprint de /consultar.
    Pone SECRET_KEY para que flash() no pete.
    """
    app = Flask("test_consultar")
    app.config["TESTING"] = True
    app.secret_key = "testing-secret"

    # No queremos hablar con ROS de verdad en estos tests
    monkeypatch.setattr(rosio_mod, "pub_state", lambda *a, **k: None, raising=False)

    app.register_blueprint(consultar_mod.bp)
    return app.test_client()


# ==========================
# TESTS UNITARIOS UTILIDADES
# ==========================

def test_normalize_dni_strips_and_upper():
    assert _normalize_dni(" 12345678z ") == "12345678Z"
    assert _normalize_dni("12 345 678-z") == "12345678Z"


def test_is_valid_dni_ok_and_ko():
    """
    _is_valid_dni debe aceptar un DNI correcto y rechazar uno incorrecto.
    Usamos 49582367W que es válido según la tabla oficial.
    """

    # DNI correcto
    assert consultar_mod._is_valid_dni("49582367W") is True

    # DNI incorrecto (letra mal)
    assert consultar_mod._is_valid_dni("49582367A") is False



def test_is_valid_name_ok_and_ko():
    assert _is_valid_name("Katrin Muñoz") is True
    assert _is_valid_name("José-Luis O'Connor") is True
    assert _is_valid_name("") is False
    assert _is_valid_name("   ") is False
    # meter números no debería colar
    assert _is_valid_name("Juan123") is False


def test_normalize_name_for_compare_collapses_spaces_and_casefold():
    s = _normalize_name_for_compare("  José   Luis  ")
    assert s == "josé luis"


# ==========================
# TESTS VISTA /consultar
# ==========================

def _patch_render_template_basic(monkeypatch):
    """
    Parche genérico para que consultar.render_template no busque
    plantillas reales. Devuelve un string plano.
    """
    monkeypatch.setattr(
        consultar_mod,
        "render_template",
        lambda *a, **k: "TPL:" + str(a[0]) if a else "TPL",
        raising=False,
    )


def test_consultar_get_renders_form(client, monkeypatch):
    """
    GET /consultar/ debe responder 200 y no petar por plantillas.
    """
    _patch_render_template_basic(monkeypatch)

    resp = client.get("/consultar/")
    assert resp.status_code == 200
    assert b"tpl:consultar.html" in resp.data.lower()


def test_consultar_post_missing_dni_redirects(client, monkeypatch):
    """
    Sin DNI -> redirect a /consultar/ con flash de error.
    """
    _patch_render_template_basic(monkeypatch)

    data = {"nombre": "Paciente", "apellidos": "Prueba", "dni": ""}
    resp = client.post("/consultar/", data=data, follow_redirects=False)

    # redirección (302/303/307 está OK, miramos que es redirect)
    assert resp.status_code in (302, 303, 307)
    assert "/consultar/" in resp.headers.get("Location", "")


def test_consultar_post_invalid_dni_redirects(client, monkeypatch):
    """
    DNI con formato o letra incorrecta -> redirect.
    """
    _patch_render_template_basic(monkeypatch)

    data = {"nombre": "Paciente", "apellidos": "Prueba", "dni": "AAAAAA"}
    resp = client.post("/consultar/", data=data, follow_redirects=False)

    assert resp.status_code in (302, 303, 307)
    assert "/consultar/" in resp.headers.get("Location", "")


def test_consultar_post_invalid_name_shows_error(client, monkeypatch):
    """
    Nombre inválido (si se proporciona) -> redirect.
    """
    _patch_render_template_basic(monkeypatch)

    data = {"nombre": "X$", "apellidos": "Prueba", "dni": "00000000T"}
    resp = client.post("/consultar/", data=data, follow_redirects=False)

    assert resp.status_code in (302, 303, 307)
    assert "/consultar/" in resp.headers.get("Location", "")

def test_consultar_existing_patient_without_meds_shows_message(client, monkeypatch):
    """
    Si el paciente existe pero no tiene recetas activas, se debe renderizar
    consultar_result.html con has_meds=False y un mensaje informativo.
    """
    from tirgo_ui.routes import consultar as consultar_mod
    from tirgo_ui import rosio

    # Paciente existente con datos COHERENTES con el formulario
    pac = {
        "_id": "pac-id-1",
        "nombre": "Katrin",
        "apellidos": "Muñoz",
        "dni_hash": "hash123",
    }

    # find_paciente_by_dni debe devolver ese paciente
    def fake_find_paciente_by_dni(dni):
        assert dni == "49582367W"
        return pac

    # Forzamos que _get_meds_for_patient devuelva lista vacía + mensaje
    def fake_get_meds_for_patient(p, msg_if_empty):
        assert p == pac
        # simulamos "paciente sin recetas activas"
        return [], msg_if_empty

    # Capturamos qué plantilla y qué contexto se usan
    def fake_render_template(name, **ctx):
        # devolvemos un string reconocible en el body de la respuesta
        return f"tpl:{name} ctx:{ctx}"

    # Parcheamos dentro del módulo consultar
    monkeypatch.setattr(consultar_mod, "store", consultar_mod.store)
    monkeypatch.setattr(consultar_mod.store, "find_paciente_by_dni", fake_find_paciente_by_dni)
    monkeypatch.setattr(consultar_mod, "_get_meds_for_patient", fake_get_meds_for_patient)
    monkeypatch.setattr(consultar_mod, "render_template", fake_render_template)

    # Evitar side effects de ROS
    monkeypatch.setattr(rosio, "pub_state", lambda *_args, **_kwargs: None)

    data = {
        "dni": "49582367W",   # DNI válido
        "nombre": "Katrin",
        "apellidos": "Muñoz",
    }

    # No hace falta seguir redirects: consultar_post ya renderiza directamente
    resp = client.post("/consultar/", data=data)
    body = resp.get_data(as_text=True)

    assert resp.status_code == 200
    # Ha usado la plantilla de resultado
    assert "tpl:consultar_result.html" in body
    # El contexto indica que no hay meds
    assert "has_meds': False" in body
    # Y aparece el mensaje informativo
    assert "Este paciente no tiene recetas activas." in body
