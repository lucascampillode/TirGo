# tests/test_storage_stock_dispenses.py
import sys
from pathlib import Path
from datetime import datetime

import pytest

# ==========
# Import del módulo a probar
# ==========
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_SRC = CURRENT_DIR.parent / "src"
sys.path.insert(0, str(PROJECT_SRC))

import tirgo_ui.storage_mongo as storage_mongo  # noqa: E402


# ==========
# Fakes de Mongo
# ==========

class FakeResult:
    def __init__(self, modified_count=0):
        self.modified_count = modified_count


class FakeCursor:
    def __init__(self, docs):
        self._docs = list(docs)

    def sort(self, key, direction):
        rev = direction == -1
        self._docs.sort(key=lambda d: d.get(key, ""), reverse=rev)
        return self

    def __iter__(self):
        return iter(self._docs)


class FakeCollection:
    def __init__(self, docs=None):
        self.docs = list(docs or [])
        self.inserted = []

    def find_one(self, filt, projection=None):
        for d in self.docs:
            ok = True
            for k, v in filt.items():
                if d.get(k) != v:
                    ok = False
                    break
            if ok:
                if projection:
                    out = {}
                    for k, v in projection.items():
                        if k == "_id":
                            continue
                        if v == 1 and k in d:
                            out[k] = d[k]
                    return out
                return dict(d)
        return None

    def update_one(self, filt, update):
        # soportamos solo $gte y $inc sobre "stock"
        for d in self.docs:
            ok = True
            for k, v in filt.items():
                if isinstance(v, dict) and "$gte" in v:
                    if d.get(k, 0) < v["$gte"]:
                        ok = False
                        break
                else:
                    if d.get(k) != v:
                        ok = False
                        break
            if ok:
                if "$inc" in update and "stock" in update["$inc"]:
                    d["stock"] = d.get("stock", 0) + update["$inc"]["stock"]
                return FakeResult(modified_count=1)
        return FakeResult(modified_count=0)

    def find(self, filt=None, projection=None):
        filt = filt or {}
        out = []
        for d in self.docs:
            include = True
            for k, cond in filt.items():
                if isinstance(cond, dict) and "$gt" in cond:
                    if not (d.get(k, 0) > cond["$gt"]):
                        include = False
                        break
                else:
                    if d.get(k) != cond:
                        include = False
                        break
            if include:
                if projection:
                    nd = {}
                    for field, flag in projection.items():
                        if field == "_id":
                            continue
                        if flag == 1 and field in d:
                            nd[field] = d[field]
                    out.append(nd)
                else:
                    out.append(dict(d))
        return FakeCursor(out)

    def insert_one(self, doc):
        # añadimos timestamp si no viene
        if "ts" in doc and isinstance(doc["ts"], datetime):
            pass
        self.docs.append(dict(doc))
        self.inserted.append(dict(doc))
        class R:
            inserted_id = len(self.docs)
        return R()

    def create_index(self, *a, **k):
        # no-op en tests
        return None


class FakeDB:
    def __init__(self, medicamentos=None, dispenses=None):
        self.medicamentos = FakeCollection(medicamentos)
        self.dispenses = FakeCollection(dispenses or [])
        # para compatibilidad si en el futuro se usan
        self.pacientes = FakeCollection([])
        self.recetas = FakeCollection([])


@pytest.fixture
def fake_db(monkeypatch):
    meds = [
        {"id": 1, "nombre": "Ibuprofeno", "stock": 10, "tipo": "L", "bin_id": 1},
        {"id": 2, "nombre": "Paracetamol", "stock": 0, "tipo": "L", "bin_id": 2},
        {"id": 3, "nombre": "Antibiótico", "stock": 5, "tipo": "R", "bin_id": 3},
    ]
    db = FakeDB(medicamentos=meds)
    monkeypatch.setattr(storage_mongo, "_db", db)
    return db


# ==========
# TESTS STOCK
# ==========

def test_get_stock_devuelve_valor_correcto(fake_db):
    assert storage_mongo.get_stock(1) == 10
    assert storage_mongo.get_stock(3) == 5
    # med inexistente → 0
    assert storage_mongo.get_stock(999) == 0


def test_dec_stock_if_available_solo_descuenta_si_hay_stock(fake_db):
    # hay 10 → puede descontar 2
    ok = storage_mongo.dec_stock_if_available(1, units=2)
    assert ok is True
    assert storage_mongo.get_stock(1) == 8

    # intenta descontar más stock del que hay → no modifica
    ok2 = storage_mongo.dec_stock_if_available(1, units=100)
    assert ok2 is False
    assert storage_mongo.get_stock(1) == 8  # sin cambios


def test_meds_disponibles_filtra_stock_mayor_que_cero(fake_db):
    meds = storage_mongo.meds_disponibles()
    ids = {m["id"] for m in meds}
    # solo 1 y 3 tienen stock > 0
    assert ids == {1, 3}
    # aseguramos que vienen ordenados por nombre
    nombres = [m["nombre"] for m in meds]
    assert nombres == sorted(nombres, key=str.lower)


# ==========
# TESTS LOG DISPENSE
# ==========

def test_log_dispense_inserta_documento_ok(fake_db):
    med = {"id": 1, "nombre": "Ibuprofeno"}
    dni_hash = "hash123"

    storage_mongo.log_dispense(med, dni_hash, ok=True, error=None, meta={"mission_id": "abc"})

    docs = fake_db.dispenses.docs
    assert len(docs) == 1
    d = docs[0]
    assert d["medicamento_id"] == 1
    assert d["dni_hash"] == dni_hash
    assert d["ok"] is True
    assert d["med_nombre"] == "Ibuprofeno"
    assert d["meta"]["mission_id"] == "abc"
    assert "ts" in d


def test_log_dispense_guarda_error_y_ok_false(fake_db):
    med = {"id": 3, "nombre": "Antibiótico"}
    storage_mongo.log_dispense(med, dni_hash=None, ok=False, error="MISSION_FAIL")

    d = fake_db.dispenses.docs[-1]
    assert d["medicamento_id"] == 3
    assert d["ok"] is False
    assert d["error"] == "MISSION_FAIL"
