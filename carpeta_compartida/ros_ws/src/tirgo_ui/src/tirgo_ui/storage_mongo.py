from typing import Optional, Dict, Any, List
import re, hmac, hashlib
from datetime import datetime
from pymongo import ReturnDocument
from bson import ObjectId

from .mongo_client import db as _db
from .config import PEPPER

# -------- Helpers --------
def _norm_text(s: str) -> str:
    return re.sub(r"\s+", " ", (s or "").strip()).lower()

def _safe_int(x, default=None):
    try: return int(x)
    except Exception: return default

def h_dni(dni: str) -> str:
    key = (PEPPER or "pepper").encode("utf-8")
    msg = _norm_text(dni).encode("utf-8")
    return hmac.new(key, msg, hashlib.sha256).hexdigest()

# -------- Índices --------
def init_db_if_needed() -> None:
    db = _db()
    db.medicamentos.create_index("id", unique=True)
    db.medicamentos.create_index([("nombre_norm", 1)])
    db.pacientes.create_index("dni_hash", unique=True)
    db.recetas.create_index([("paciente_id", 1), ("medicamento_id", 1), ("activa", 1)])
    db.dispenses.create_index([("ts", -1)])
    db.dispenses.create_index([("medicamento_id", 1)])
    db.dispenses.create_index([("dni_hash", 1)])

# -------- Medicamentos --------
def _map_med(doc: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not doc:
        return None
    mid = _safe_int(doc.get("id"))
    if mid is None:
        return None
    return {
        "id": mid,
        "nombre": doc.get("nombre"),
        "nombre_norm": doc.get("nombre_norm") or (doc.get("nombre") and _norm_text(doc.get("nombre"))),
        "tipo": doc.get("tipo", "L"),
        "bin_id": _safe_int(doc.get("bin_id"), 0),
        "stock": _safe_int(doc.get("stock"), 0),
    }

def lookup_medicamento_by_name(name: str) -> Optional[Dict[str, Any]]:
    db = _db()
    n = _norm_text(name)
    doc = db.medicamentos.find_one({"nombre_norm": n})
    if not doc:
        doc = db.medicamentos.find_one({"nombre": {"$regex": f"^{re.escape(name)}$", "$options": "i"}})
    return _map_med(doc)

def lookup_medicamento_by_id(med_id: int) -> Optional[Dict[str, Any]]:
    db = _db()
    doc = db.medicamentos.find_one({"id": _safe_int(med_id)})
    return _map_med(doc)

def get_stock(med_id: int) -> int:
    db = _db()
    doc = db.medicamentos.find_one({"id": _safe_int(med_id)}, {"stock": 1})
    return _safe_int(doc.get("stock"), 0) if doc else 0

def dec_stock_if_available(med_id: int, units: int = 1) -> bool:
    db = _db()
    res = db.medicamentos.find_one_and_update(
        {"id": _safe_int(med_id), "stock": {"$gte": int(units)}},
        {"$inc": {"stock": -int(units)}},
        return_document=ReturnDocument.AFTER,
    )
    return bool(res)

def inc_stock(med_id: int, units: int = 1) -> None:
    db = _db()
    db.medicamentos.update_one({"id": _safe_int(med_id)}, {"$inc": {"stock": int(units)}})

# -------- Pacientes / Recetas --------
def find_or_create_paciente(nombre: str, apellidos: str, dni: str) -> Optional[str]:
    nombre    = _norm_text(nombre)
    apellidos = _norm_text(apellidos)
    dni_norm  = _norm_text(dni)
    if not (nombre and apellidos and dni_norm):
        return None

    db = _db()
    dni_hash = h_dni(dni_norm)
    doc = db.pacientes.find_one({"dni_hash": dni_hash})
    if doc:
        return str(doc["_id"])

    res = db.pacientes.insert_one({
        "nombre": nombre,
        "apellidos": apellidos,
        "dni_hash": dni_hash,
        "necesita_restringido": 0,
    })
    return str(res.inserted_id)

def paciente_necesita_restringido(paciente_id: str) -> int:
    db = _db()
    doc = db.pacientes.find_one({"_id": _as_obj_id(paciente_id)}, {"necesita_restringido": 1})
    if not doc:
        return 0
    try:
        return int(doc.get("necesita_restringido", 0))
    except Exception:
        return 0

def tiene_receta_activa(paciente_id: str, med_id: int) -> bool:
    db = _db()
    r = db.recetas.find_one({
        "paciente_id": _as_obj_id(paciente_id),
        "medicamento_id": int(med_id),
        "activa": True
    })
    return bool(r)

def permitted_meds_for_patient(paciente_id: str) -> List[Dict[str, Any]]:
    db = _db()
    necesita = bool(paciente_necesita_restringido(paciente_id))
    meds: List[Dict[str, Any]] = []
    cursor = db.medicamentos.find({}, {"_id": 0, "id": 1, "nombre": 1, "nombre_norm": 1, "tipo": 1, "bin_id": 1, "stock": 1})
    for raw in cursor:
        m = _map_med(raw)
        if not m:
            continue
        if m.get("tipo", "L") != "R":
            meds.append(m)
        else:
            if necesita or tiene_receta_activa(paciente_id, int(m["id"])):
                meds.append(m)
    return meds

# -------- Log auditoría --------
def log_dispense(med: Dict[str, Any], dni_hash: Optional[str]) -> None:
    db = _db()
    payload = {
        "ts": datetime.utcnow(),
        "medicamento_id": int(med["id"]),
        "medicamento_nombre": med.get("nombre"),
        "dni_hash": dni_hash,
    }
    db.dispenses.insert_one(payload)

# -------- Utils --------
def _as_obj_id(s: str) -> ObjectId:
    return s if isinstance(s, ObjectId) else ObjectId(str(s))
