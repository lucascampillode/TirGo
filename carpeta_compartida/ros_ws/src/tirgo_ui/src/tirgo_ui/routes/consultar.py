from flask import Blueprint, render_template, request, redirect, url_for, flash
from .. import storage_mongo as store
from .. import rosio

bp = Blueprint("consultar", __name__, url_prefix="/consultar")


def _get_db():
    # intenta sacar la db ya autenticada que usa storage_mongo
    for name in ("db", "mongo", "database", "mongo_db", "_db", "_mongo"):
        if hasattr(store, name):
            return getattr(store, name)

    for client_name in ("client", "mongo_client"):
        if hasattr(store, client_name):
            cli = getattr(store, client_name)
            try:
                return cli["tirgo"]
            except Exception:
                pass
    return None


@bp.get("/", endpoint="consultar")
def consultar_get():
    return render_template("consultar.html")


@bp.post("/", endpoint="consultar_post")
def consultar_post():
    nombre = request.form.get("nombre", "").strip()
    apellidos = request.form.get("apellidos", "").strip()
    dni = request.form.get("dni", "").strip()
    rosio.pub_state('IDENTIFYING')

    # 1) encontrar paciente con tu helper (esto ya te funcionaba)
    pac = store.find_paciente_by_dni(dni)
    if not pac:
        return render_template(
            "consultar_paciente_no_existente.html",
            nombre=nombre, apellidos=apellidos, dni=dni
        )

    db = _get_db()
    meds = []
    msg = None

    if db is not None:
        # 2) recetas activas de ese paciente
        recetas = list(db.recetas.find({
            "paciente_id": pac["_id"],
            "activa": True
        }))

        # por si alguna quedó con el id en string
        if not recetas:
            recetas = list(db.recetas.find({
                "paciente_id": str(pac["_id"]),
                "activa": True
            }))

        # por si algún día guardas por dni_hash
        if not recetas and "dni_hash" in pac:
            recetas = list(db.recetas.find({
                "dni_hash": pac["dni_hash"],
                "activa": True
            }))

        if recetas:
            for r in recetas:
                med_doc = db.medicamentos.find_one({"id": r["medicamento_id"]})
                if med_doc:
                    # guardamos el id de la receta por si luego quieres usarlo
                    med_doc["_receta_id"] = str(r["_id"])
                    meds.append(med_doc)
        else:
            msg = "Este paciente no tiene recetas activas."
    else:
        # fallback por si un día no hay db
        meds = store.permitted_meds_for_patient(pac["_id"]) or store.permitted_meds_for_patient(str(pac["_id"]))
        if not meds:
            msg = "Este paciente no tiene recetas activas."

    has_meds = bool(meds)

    return render_template(
        "consultar_result.html",
        paciente=pac,
        meds=meds,
        has_meds=has_meds,
        msg=msg
    )


@bp.post("/crear_paciente", endpoint="crear_paciente")
def crear_paciente():
    nombre = request.form.get("nombre", "")
    apellidos = request.form.get("apellidos", "")
    dni = request.form.get("dni", "")
    try:
        pac = store.create_paciente_if_allowed(nombre, apellidos, dni)
    except ValueError as e:
        flash(str(e), "error")
        return redirect(url_for("consultar.consultar"))

    db = _get_db()
    meds = []
    msg = None

    if db is not None:
        recetas = list(db.recetas.find({
            "paciente_id": pac["_id"],
            "activa": True
        }))
        if not recetas:
            recetas = list(db.recetas.find({
                "paciente_id": str(pac["_id"]),
                "activa": True
            }))
        if not recetas and "dni_hash" in pac:
            recetas = list(db.recetas.find({
                "dni_hash": pac["dni_hash"],
                "activa": True
            }))

        if recetas:
            for r in recetas:
                med_doc = db.medicamentos.find_one({"id": r["medicamento_id"]})
                if med_doc:
                    med_doc["_receta_id"] = str(r["_id"])
                    meds.append(med_doc)
        else:
            msg = "Paciente creado, pero no tiene recetas activas."
    else:
        meds = store.permitted_meds_for_patient(pac["_id"]) or store.permitted_meds_for_patient(str(pac["_id"]))
        if not meds:
            msg = "Paciente creado, pero no tiene recetas activas."

    has_meds = bool(meds)

    flash("Paciente creado correctamente", "ok")
    return render_template(
        "consultar_result.html",
        paciente=pac,
        meds=meds,
        has_meds=has_meds,
        msg=msg
    )


@bp.post("/cancelar_creacion", endpoint="cancelar_creacion")
def cancelar_creacion():
    flash("Operación cancelada", "info")
    return redirect(url_for("consultar.consultar"))
