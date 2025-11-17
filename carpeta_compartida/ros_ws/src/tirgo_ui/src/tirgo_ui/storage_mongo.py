import os
import re
import unicodedata
import hmac
import hashlib
from typing import Optional
from datetime import datetime, timezone

from pymongo.errors import DuplicateKeyError
from bson import ObjectId
from .mongo_client import get_db

_db = get_db()
PEPPER = os.environ.get("TIRGO_PEPPER", "").encode()

# ---------- helpers ----------

def _norm_txt(s: str) -> str:
    s = (s or "").strip()
    s = unicodedata.normalize("NFD", s)
    s = "".join(c for c in s if unicodedata.category(c) != "Mn")  # quita tildes
    s = re.sub(r"\s+", " ", s).lower()
    return s

def h_dni(dni: str) -> str:
    return hmac.new(PEPPER, (dni or "").strip().upper().encode(), hashlib.sha256).hexdigest()

# ---------- init & indexes ----------

def init_db_if_needed():
    # medicamentos
    _db.medicamentos.create_index("id", unique=True)
    _db.medicamentos.create_index("nombre_norm")
    # pacientes
    _db.pacientes.create_index("dni_hash", unique=True)
    _db.pacientes.create_index(
        [("nombre_norm", 1), ("apellidos_norm", 1)],
        unique=True,
        partialFilterExpression={
            # Existe y es > ""  => no vacío (válido en partial indexes)
            "nombre_norm":     {"$exists": True, "$gt": ""},
            "apellidos_norm":  {"$exists": True, "$gt": ""},
        }
    )
    # recetas / dispenses
    _db.recetas.create_index([("paciente_id", 1), ("medicamento_id", 1), ("activa", 1)])
    _db.dispenses.create_index("ts")
    _db.dispenses.create_index([("medicamento_id", 1), ("ts", -1)])
    return True

# ---------- medicamentos ----------

def _map_med(doc):
    if not doc:
        return None
    return {
        "id": int(doc.get("id")),
        "nombre": doc.get("nombre", ""),
        "nombre_norm": doc.get("nombre_norm", ""),
        "tipo": doc.get("tipo", "L"),
        "bin_id": int(doc.get("bin_id", 0)),
        "stock": int(doc.get("stock", 0)),
        "img_url": doc.get("img_url"),
    }

def lookup_medicamento_by_name(name: str):
    n = _norm_txt(name)
    doc = _db.medicamentos.find_one({"nombre_norm": n})
    return _map_med(doc) if doc else None

def lookup_medicamento_by_id(med_id: int):
    """Devuelve un medicamento por id exacto (int). Sin _id de Mongo."""
    from .mongo_client import get_db
    db = get_db()
    try:
        med_id = int(med_id)
    except Exception:
        return None
    doc = db.medicamentos.find_one({"id": med_id}, {"_id": 0})
    if doc and "tipo" in doc and isinstance(doc["tipo"], str):
        doc["tipo"] = doc["tipo"].strip().upper()
    return doc

def get_stock(med_id: int) -> int:
    d = _db.medicamentos.find_one({"id": int(med_id)}, {"stock": 1, "_id": 0})
    return int(d.get("stock", 0)) if d else 0

def dec_stock_if_available(med_id: int, units: int = 1) -> bool:
    res = _db.medicamentos.update_one(
        {"id": int(med_id), "stock": {"$gte": units}},
        {"$inc": {"stock": -units}},
    )
    return res.modified_count == 1

def inc_stock(med_id: int, units: int = 1):
    _db.medicamentos.update_one({"id": int(med_id)}, {"$inc": {"stock": units}})

def meds_disponibles():
    """Para selector en /leer: solo stock>0, campos necesarios."""
    cur = _db.medicamentos.find(
        {"stock": {"$gt": 0}},
        {"_id": 0, "id": 1, "nombre": 1, "stock": 1, "tipo": 1, "img_url": 1, "bin_id": 1},
    ).sort("nombre", 1)
    return list(cur)

# ---------- pacientes & recetas ----------

def find_paciente_by_dni(dni: str):
    return _db.pacientes.find_one({"dni_hash": h_dni(dni)})

def get_paciente(paciente_id):
    return _db.pacientes.find_one({"_id": ObjectId(paciente_id)})

def create_paciente_if_allowed(nombre: str, apellidos: str, dni: str):
    """
    Crea paciente SOLO si no existe otro con mismo nombre+apellidos.
    Unicidad garantizada por índice (nombre_norm, apellidos_norm).
    Si ya existe con distinto dni_hash -> error.
    """
    nombre_norm = _norm_txt(nombre)
    apellidos_norm = _norm_txt(apellidos)
    doc = {
        "nombre": (nombre or "").strip(),
        "apellidos": (apellidos or "").strip(),
        "nombre_norm": nombre_norm,
        "apellidos_norm": apellidos_norm,
        "dni_hash": h_dni(dni),
    }
    try:
        res = _db.pacientes.insert_one(doc)
        return _db.pacientes.find_one({"_id": res.inserted_id})
    except DuplicateKeyError:
        existing = _db.pacientes.find_one({"nombre_norm": nombre_norm, "apellidos_norm": apellidos_norm})
        if existing and existing["dni_hash"] != doc["dni_hash"]:
            raise ValueError("Ya existe un paciente con el mismo nombre y apellidos y otro DNI.")
        return existing

def paciente_necesita_restringido(paciente_id):
    """Tu lógica existente: placeholder para compatibilidad."""
    # Si tenías otra semántica, mantenla; aquí devolvemos False por defecto.
    return False

def tiene_receta_activa(paciente_id, med_id: int) -> bool:
    return _db.recetas.find_one({
        "paciente_id": ObjectId(paciente_id),
        "medicamento_id": int(med_id),
        "activa": True
    }) is not None

def permitted_meds_for_patient(paciente_id):
    meds = _db.medicamentos.find({}, {
        "_id": 0, "id": 1, "nombre": 1, "nombre_norm": 1,
        "tipo": 1, "bin_id": 1, "stock": 1, "img_url": 1
    })
    out = []
    for d in meds:
        m = _map_med(d)
        permitido = True
        motivo = "LIBRE"
        if m["tipo"] == "R":
            permitido = tiene_receta_activa(paciente_id, m["id"])
            motivo = "RECIPE_OK" if permitido else "RECIPE_REQUIRED"
        m["permitido"] = permitido
        m["motivo"] = motivo
        out.append(m)
    out.sort(key=lambda x: x["nombre"].lower())
    return out

# ---------- compatibilidad API antigua ----------

def find_or_create_paciente(nombre: str, apellidos: str, dni: str):
    """
    Mantiene la firma antigua devolviendo el ObjectId del paciente.
    Internamente usa create_paciente_if_allowed() (con la regla de unicidad
    nombre+apellidos). Devuelve None si no se pudo crear.
    """
    pac = create_paciente_if_allowed(nombre, apellidos, dni)
    return pac["_id"] if pac else None

# ---------- logs de dispensado ----------

def log_dispense(
    med: dict,
    dni_hash: Optional[str] = None,
    ok: bool = True,
    error: Optional[str] = None,
    meta: Optional[dict] = None
):
    """
    Guarda un log de dispensado en la colección 'dispenses'.
    - med: dict con al menos 'id' (int) y, opcionalmente, 'nombre'
    - dni_hash: hash del DNI (o None si libre)
    - ok: True si el proceso terminó bien; False si hubo error y se revirtió
    - error: texto del error si ok=False
    - meta: campos adicionales (p.ej. {'bin_id': 3})
    """
    doc = {
        "ts": datetime.now(timezone.utc),
        "medicamento_id": int(med.get("id")),
        "dni_hash": dni_hash,
        "ok": bool(ok),
    }
    if "nombre" in med:
        doc["med_nombre"] = med["nombre"]
    if meta:
        doc["meta"] = meta
    if error:
        doc["error"] = str(error)

    _db.dispenses.insert_one(doc)
