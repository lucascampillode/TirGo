#!/usr/bin/env python3
"""
Semilla de datos de demo para TirGoPharma.

- Medicamentos básicos (L y R)
- Paciente de demo
- Recetas activas / inactivas
"""

from datetime import datetime, timezone

from .mongo_client import get_db
from .storage_mongo import h_dni, init_db_if_needed


def seed():
    db = get_db()
    init_db_if_needed()

    # ----------------------
    # Medicamentos de ejemplo
    # ----------------------
    meds = [
        {
            "id": 1,
            "nombre": "Paracetamol 1g",
            "nombre_norm": "paracetamol 1g",
            "tipo": "L",
            "bin_id": 1,
            "stock": 10,
            "img_url": None,
        },
        {
            "id": 2,
            "nombre": "Ibuprofeno 600mg",
            "nombre_norm": "ibuprofeno 600mg",
            "tipo": "R",
            "bin_id": 2,
            "stock": 5,
            "img_url": None,
        },
        {
            "id": 3,
            "nombre": "Omeprazol 20mg",
            "nombre_norm": "omeprazol 20mg",
            "tipo": "R",
            "bin_id": 3,
            "stock": 3,
            "img_url": None,
        },
    ]

    for m in meds:
        db.medicamentos.update_one(
            {"id": m["id"]},
            {"$set": m},
            upsert=True,
        )

    # ----------------------
    # Paciente de demo
    # ----------------------
    dni_demo = "49582367W"
    pac = {
        "nombre": "Katrin",
        "apellidos": "Muñoz",
        "nombre_norm": "katrin",
        "apellidos_norm": "munoz",
        "dni_hash": h_dni(dni_demo),
        "created_at": datetime.now(timezone.utc),
    }

    res = db.pacientes.update_one(
        {"dni_hash": pac["dni_hash"]},
        {"$setOnInsert": pac},
        upsert=True,
    )

    if res.upserted_id is not None:
        pac_id = res.upserted_id
    else:
        pac_id = db.pacientes.find_one({"dni_hash": pac["dni_hash"]})["_id"]

    # ----------------------
    # Recetas de demo
    # ----------------------
    recetas = [
        {
            "paciente_id": pac_id,
            "medicamento_id": 2,  # Ibuprofeno R
            "activa": True,
            "created_at": datetime.now(timezone.utc),
        },
        {
            "paciente_id": pac_id,
            "medicamento_id": 3,  # Omeprazol R (inactiva)
            "activa": False,
            "created_at": datetime.now(timezone.utc),
        },
    ]

    for r in recetas:
        db.recetas.update_one(
            {
                "paciente_id": r["paciente_id"],
                "medicamento_id": r["medicamento_id"],
            },
            {"$set": r},
            upsert=True,
        )

    print("✅ Datos de demo insertados/actualizados correctamente.")


if __name__ == "__main__":
    seed()
