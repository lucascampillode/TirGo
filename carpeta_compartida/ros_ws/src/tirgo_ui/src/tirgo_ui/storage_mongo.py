import os
import hmac
import hashlib
from datetime import datetime, timezone
from typing import Optional, List, Dict, Any

from bson import ObjectId
from pymongo.errors import DuplicateKeyError

from .mongo_client import get_db

_db = get_db()

# Pepper para el hash del DNI (HMAC)
PEPPER = os.environ.get("TIRGO_PEPPER", "").encode()


def h_dni(dni: str) -> str:
    """
    Devuelve un hash HMAC-SHA256 del DNI normalizado (upper + strip),
    usando un PEPPER configurable por variable de entorno.

    Esto permite no guardar el DNI en claro en la BD.
    """
    return hmac.new(
        PEPPER,
        (dni or "").strip().upper().encode(),
        hashlib.sha256,
    ).hexdigest()


def _norm_txt(txt: str) -> str:
    """
    Normaliza texto para comparaciones sencillas:
    - strip
    - lower
    """
    return (txt or "").strip().lower()


def init_db_if_needed() -> bool:
    """
    Crea índices necesarios si no existen.
    Se puede llamar al inicio de la app sin problemas (idempotente).
    """
    # --- Colección medicamentos ---
    _db.medicamentos.create_index("id", unique=True)
    _db.medicamentos.create_index("nombre_norm")

    # --- Colección pacientes ---
    _db.pacientes.create_index("dni_hash", unique=True)
    _db.pacientes.create_index(
        [("nombre_norm", 1), ("apellidos_norm", 1)],
        unique=True,
        partialFilterExpression={
            "nombre_norm": {"$exists": True, "$gt": ""},
            "apellidos_norm": {"$exists": True, "$gt": ""},
        },
    )

    # --- Colección recetas ---
    _db.recetas.create_index(
        [("paciente_id", 1), ("medicamento_id", 1), ("activa", 1)]
    )

    # --- Colección dispenses (logs de dispensado) ---
    _db.dispenses.create_index("ts")
    _db.dispenses.create_index([("medicamento_id", 1), ("ts", -1)])

    return True


def _map_med(doc: Dict[str, Any]) -> Dict[str, Any]:
    """
    Mapea un documento de la colección medicamentos a un dict estándar
    para usar en el resto de la app.
    """
    if not doc:
        return {}

    return {
        "id": int(doc.get("id")),
        "nombre": doc.get("nombre", ""),
        "nombre_norm": doc.get("nombre_norm", ""),
        "tipo": doc.get("tipo", "L"),  # 'L' libre, 'R' receta
        "bin_id": int(doc.get("bin_id", 0)),
        "stock": int(doc.get("stock", 0)),
        "img_url": doc.get("img_url"),
    }


# ==========================
# MEDICAMENTOS
# ==========================

def get_stock(med_id: int) -> int:
    """
    Devuelve el stock actual de un medicamento (por id lógico),
    o 0 si no existe.
    """
    d = _db.medicamentos.find_one(
        {"id": int(med_id)},
        {"stock": 1, "_id": 0},
    )
    return int(d.get("stock", 0)) if d else 0


def dec_stock_if_available(med_id: int, units: int = 1) -> bool:
    """
    Decrementa el stock de forma atómica solo si hay stock suficiente.
    Devuelve True si se ha modificado (stock descontado), False si no.
    """
    res = _db.medicamentos.update_one(
        {"id": int(med_id), "stock": {"$gte": units}},
        {"$inc": {"stock": -units}},
    )
    return res.modified_count == 1


def meds_disponibles() -> List[Dict[str, Any]]:
    """
    Devuelve una lista de medicamentos con stock > 0 para mostrarlos
    en la vista de /leer.
    """
    cur = _db.medicamentos.find(
        {"stock": {"gt": 0}} if False else {"stock": {"$gt": 0}},
        {
            "_id": 0,
            "id": 1,
            "nombre": 1,
            "stock": 1,
            "tipo": 1,
            "img_url": 1,
            "bin_id": 1,
        },
    ).sort("nombre", 1)
    return list(cur)


def lookup_medicamento_by_id(med_id: int) -> Optional[Dict[str, Any]]:
    """
    Busca un medicamento por su id lógico.
    """
    doc = _db.medicamentos.find_one({"id": int(med_id)})
    if not doc:
        return None
    return _map_med(doc)


def find_medicamento_by_bin(bin_id: int) -> Optional[Dict[str, Any]]:
    """
    Devuelve el medicamento asociado a un determinado bin_id, o None si no existe.
    """
    doc = _db.medicamentos.find_one({"bin_id": int(bin_id)})
    if not doc:
        return None
    return _map_med(doc)


# ==========================
# PACIENTES
# ==========================

def find_paciente_by_dni(dni: str) -> Optional[Dict[str, Any]]:
    """
    Busca un paciente por DNI usando el hash HMAC.
    """
    return _db.pacientes.find_one({"dni_hash": h_dni(dni)})


def create_paciente_if_allowed(nombre: str, apellidos: str, dni: str) -> Dict[str, Any]:
    """
    Crea un paciente nuevo si no entra en conflicto con otros pacientes.

    Reglas:
    - dni_hash es único.
    - (nombre_norm, apellidos_norm) también es único (con filtro parcial).
    - Si ya existe uno con el mismo nombre/apellidos pero OTRO DNI, se lanza ValueError.
    - Si el conflicto es con el mismo nombre/apellidos y mismo DNI, se devuelve el existente.
    """
    nombre_norm = _norm_txt(nombre)
    apellidos_norm = _norm_txt(apellidos)

    doc = {
        "nombre": (nombre or "").strip(),
        "apellidos": (apellidos or "").strip(),
        "dni_hash": h_dni(dni),
        "nombre_norm": nombre_norm,
        "apellidos_norm": apellidos_norm,
        "created_at": datetime.now(timezone.utc),
    }

    try:
        res = _db.pacientes.insert_one(doc)
        return _db.pacientes.find_one({"_id": res.inserted_id})
    except DuplicateKeyError:
        existing = _db.pacientes.find_one({
            "nombre_norm": nombre_norm,
            "apellidos_norm": apellidos_norm,
        })

        if existing and existing.get("dni_hash") != doc["dni_hash"]:
            # Mismo nombre y apellidos pero otro DNI → conflicto
            raise ValueError(
                "Ya existe un paciente con el mismo nombre y apellidos pero otro DNI."
            )

        # Si es el mismo paciente (mismo nombre/apellidos y mismo hash de DNI), devolvemos el existente
        if existing:
            return existing

        existing_by_dni = find_paciente_by_dni(dni)
        if existing_by_dni:
            return existing_by_dni

        # Caso raro, devolvemos doc "tal cual"
        return doc


# ==========================
# RECETAS
# ==========================

def tiene_receta_activa(paciente_id: ObjectId, med_id: int) -> bool:
    """
    Devuelve True si el paciente tiene una receta activa para ese medicamento.
    """
    q = {
        "paciente_id": ObjectId(paciente_id),
        "medicamento_id": int(med_id),
        "activa": True,
    }
    return _db.recetas.find_one(q) is not None


def permitted_meds_for_patient(paciente_id: ObjectId) -> List[Dict[str, Any]]:
    """
    Devuelve el catálogo de medicamentos con campos extra:
    - permitido: bool
    - motivo: "LIBRE", "RECIPE_OK", "RECIPE_REQUIRED"
    """
    meds_cur = _db.medicamentos.find(
        {},
        {
            "_id": 0,
            "id": 1,
            "nombre": 1,
            "nombre_norm": 1,
            "tipo": 1,
            "bin_id": 1,
            "stock": 1,
            "img_url": 1,
        },
    )

    out: List[Dict[str, Any]] = []
    for d in meds_cur:
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


def paciente_necesita_restringido(paciente, med) -> bool:
    """
    Función legacy para compatibilidad con leer.py.

    Devuelve True si el medicamento es de tipo 'R' (restringido) y
    el paciente NO tiene una receta activa para ese medicamento.
    """
    from bson import ObjectId as _OID

    # --- obtener ObjectId del paciente ---
    pac_id = None
    if isinstance(paciente, dict):
        pac_id = paciente.get("_id")
    else:
        pac_id = paciente

    if pac_id is None:
        if isinstance(med, dict):
            return med.get("tipo") == "R"
        return True

    if not isinstance(pac_id, _OID):
        try:
            pac_id = _OID(pac_id)
        except Exception:
            if isinstance(med, dict):
                return med.get("tipo") == "R"
            return True

    # --- obtener id y tipo de medicamento ---
    if isinstance(med, dict):
        med_id = med.get("id")
        med_tipo = med.get("tipo", None)
    else:
        med_id = med
        med_tipo = None

    try:
        med_id_int = int(med_id)
    except (TypeError, ValueError):
        return True

    if med_tipo is not None and med_tipo != "R":
        return False

    return not tiene_receta_activa(pac_id, med_id_int)


# ==========================
# LOG DE DISPENSADO
# ==========================

def log_dispense(
    med: Dict[str, Any],
    dni_hash: Optional[str],
    ok: bool,
    error: Optional[str] = None,
    meta: Optional[Dict[str, Any]] = None,
    **extra: Any,
) -> None:
    """
    Guarda un log de dispensado en la colección 'dispenses'.

    Campos básicos:
      - ts, medicamento_id, dni_hash, ok
    Extra:
      - med_nombre (si está en med)
      - meta (dict arbitrario)
      - error (string)
      - mission_id, error_code (si se pasan como kwargs)
    """
    doc: Dict[str, Any] = {
        "ts": datetime.now(timezone.utc),
        "medicamento_id": int(med.get("id", -1)),
        "dni_hash": dni_hash,
        "ok": bool(ok),
    }

    if "nombre" in med:
        doc["med_nombre"] = med["nombre"]

    if meta:
        doc["meta"] = meta

    if error:
        doc["error"] = str(error)

    # Campos extra opcionales
    mission_id = extra.get("mission_id")
    if mission_id:
        doc["mission_id"] = str(mission_id)

    error_code = extra.get("error_code")
    if error_code:
        doc["error_code"] = str(error_code)

    _db.dispenses.insert_one(doc)

def apply_mission_result(
    med_id: int,
    dni_hash: Optional[str],
    success: bool,
    mission_id: Optional[str] = None,
    error_code: str = "",
    error_message: str = "",
    meta: Optional[Dict[str, Any]] = None,
) -> None:
    """
    Aplica el resultado de una misión de dispensado:

    - Si success=True:
        * decrementa el stock (si hay)
        * registra log ok=True
    - Si success=False:
        * NO toca el stock
        * registra log ok=False con error_code / error_message

    Deja toda la lógica de negocio encapsulada aquí.
    """
    med = lookup_medicamento_by_id(med_id) or {"id": med_id}

    if success:
        dec_stock_if_available(med_id, 1)

    log_dispense(
        med,
        dni_hash,
        ok=success,
        error=error_message or None,
        meta=meta,
        mission_id=mission_id,
        error_code=error_code,
    )
