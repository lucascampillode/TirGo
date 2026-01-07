import re
from flask import Blueprint, render_template, request, redirect, url_for, flash
from .. import storage_mongo as store
from .. import rosio

bp = Blueprint("consultar", __name__, url_prefix="/consultar")

# Tabla oficial para la letra de control del DNI
_DNI_LETTERS = "TRWAGMYFPDXBNJZSQVHLCKE"

# Regex para nombres/apellidos "humanos" (letras, espacios, apóstrofos, guiones)
_NAME_REGEX = re.compile(r"^[A-Za-zÁÉÍÓÚÜÑáéíóúüñ\s'-]+$")

# Longitud máxima razonable para nombre/apellidos
_MAX_NAME_LEN = 80


def _normalize_dni(dni_raw: str) -> str:
    """
    Normaliza la cadena de DNI:
    - Elimina espacios y guiones.
    - Pasa a mayúsculas.
    """
    return dni_raw.strip().upper().replace(" ", "").replace("-", "")


def _is_valid_dni(dni: str) -> bool:
    """
    Valida que el DNI:
    - Tiene formato 8 dígitos + 1 letra.
    - La letra de control es correcta según la tabla oficial.
    """
    if not re.fullmatch(r"\d{8}[A-Z]", dni):
        return False

    number = int(dni[:8])
    expected_letter = _DNI_LETTERS[number % 23]
    return dni[8] == expected_letter


def _normalize_name_for_compare(text: str) -> str:
    """
    Normaliza un nombre/apellidos para comparación:
    - strip()
    - colapsa múltiples espacios internos a uno
    - casefold() para ignorar mayúsculas/minúsculas y acentos
    """
    text = text.strip()
    if not text:
        return ""
    # Colapsar espacios múltiples a uno
    text = re.sub(r"\s+", " ", text)
    return text.casefold()


def _is_valid_name(text: str) -> bool:
    """
    Comprueba que un nombre/apellidos:
    - No es vacío.
    - No excede longitud máxima.
    - Contiene solo letras, espacios, apóstrofos o guiones.
    """
    text = text.strip()
    if not text:
        return False
    if len(text) > _MAX_NAME_LEN:
        return False
    return _NAME_REGEX.fullmatch(text) is not None


def _get_db():
    """
    Intenta recuperar el handler de base de datos que ya usa storage_mongo,
    evitando reabrir conexiones. Devuelve None si no se encuentra.
    """
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


def _get_meds_for_patient(pac: dict, msg_if_empty: str):
    """
    Busca las recetas activas de un paciente y resuelve los documentos
    de medicamentos correspondientes.

    Devuelve:
      - meds: lista de docs de medicamentos (cada uno con '_receta_id' añadido)
      - msg: mensaje informativo si no hay recetas activas
    """
    db = _get_db()
    meds = []
    msg = None

    if db is not None:
        # 1) recetas activas asociadas al paciente
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
                med_id = r.get("medicamento_id")
                med_doc = None

                if med_id is not None:
                    # Intento 1: tal cual
                    med_doc = db.medicamentos.find_one({"id": med_id})

                    # Intento 2: si es str de dígitos, probar como int
                    if med_doc is None and isinstance(med_id, str) and med_id.isdigit():
                        med_doc = db.medicamentos.find_one({"id": int(med_id)})

                    # Intento 3: si es int, probar como str
                    if med_doc is None and isinstance(med_id, int):
                        med_doc = db.medicamentos.find_one({"id": str(med_id)})

                if med_doc:
                    # guardamos el id de la receta por si luego quieres usarlo en la UI
                    med_doc["_receta_id"] = str(r["_id"])
                    meds.append(med_doc)
        else:
            msg = msg_if_empty
    else:
        # Fallback por si no hay acceso directo a la DB
        meds = (
            store.permitted_meds_for_patient(pac["_id"])
            or store.permitted_meds_for_patient(str(pac["_id"]))
        )
        if not meds:
            msg = msg_if_empty

    return meds, msg


@bp.get("/", endpoint="consultar")
def consultar_get():
    rosio.set_ui_menu("consultar")
    return render_template("consultar.html")


@bp.post("/", endpoint="consultar_post")
def consultar_post():
    nombre = request.form.get("nombre", "").strip()
    apellidos = request.form.get("apellidos", "").strip()
    dni_raw = request.form.get("dni", "").strip()

    # Normalizar DNI (quitar espacios / guiones y pasar a mayúsculas)
    dni = _normalize_dni(dni_raw)

    # Validación mínima: DNI obligatorio
    if not dni:
        flash("Debes introducir un DNI para realizar la consulta.", "error")
        return redirect(url_for("consultar.consultar"))

    # Validar formato y letra de control
    if not _is_valid_dni(dni):
        flash("DNI no válido. Debe tener 8 números y una letra correcta.", "error")
        return redirect(url_for("consultar.consultar"))

    # Validar nombre/apellidos SOLO si el usuario los ha escrito
    if nombre and not _is_valid_name(nombre):
        flash("El nombre solo puede contener letras, espacios, apóstrofos y guiones (máx. 80 caracteres).", "error")
        return redirect(url_for("consultar.consultar"))

    if apellidos and not _is_valid_name(apellidos):
        flash("Los apellidos solo puede contener letras, espacios, apóstrofos y guiones (máx. 80 caracteres).", "error")
        return redirect(url_for("consultar.consultar"))

    rosio.pub_state('IDENTIFYING')

    # 1) encontrar paciente por DNI
    pac = store.find_paciente_by_dni(dni)
    if not pac:
        rosio.pub_state('PATIENT_NOT_FOUND')
        return render_template(
            "consultar_paciente_no_existente.html",
            nombre=nombre,
            apellidos=apellidos,
            dni=dni  # ya normalizado
        )

    # 2) Comprobar coherencia de nombre/apellidos si el usuario los ha proporcionado
    input_nombre = _normalize_name_for_compare(nombre) if nombre else ""
    input_apellidos = _normalize_name_for_compare(apellidos) if apellidos else ""
    pac_nombre = _normalize_name_for_compare(str(pac.get("nombre", "")))
    pac_apellidos = _normalize_name_for_compare(str(pac.get("apellidos", "")))

    if (input_nombre and input_nombre != pac_nombre) or \
       (input_apellidos and input_apellidos != pac_apellidos):
        # El DNI existe, pero el nombre/apellidos no coinciden -> NO mostramos la ficha
        rosio.pub_state('PATIENT_DATA_MISMATCH')
        flash("Los datos introducidos (nombre/apellidos) no coinciden con el DNI registrado.", "error")
        return redirect(url_for("consultar.consultar"))

    # 3) obtener medicamentos (recetas activas) para ese paciente
    meds, msg = _get_meds_for_patient(pac, "Este paciente no tiene recetas activas.")
    has_meds = bool(meds)

    if has_meds:
        rosio.pub_state('PATIENT_FOUND')
    else:
        rosio.pub_state('NO_RECIPES')

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
    dni_raw = request.form.get("dni", "")

    dni = _normalize_dni(dni_raw)

    # Validar nombre y apellidos (aquí sí son obligatorios y correctos)
    if not _is_valid_name(nombre):
        flash("El nombre solo puede contener letras, espacios, apóstrofos y guiones (máx. 80 caracteres).", "error")
        return redirect(url_for("consultar.consultar"))

    if not _is_valid_name(apellidos):
        flash("Los apellidos solo puede contener letras, espacios, apóstrofos y guiones (máx. 80 caracteres).", "error")
        return redirect(url_for("consultar.consultar"))

    # Validar DNI
    if not dni:
        flash("Debes introducir un DNI para crear el paciente.", "error")
        return redirect(url_for("consultar.consultar"))

    if not _is_valid_dni(dni):
        flash("DNI no válido. Debe tener 8 números y una letra correcta.", "error")
        return redirect(url_for("consultar.consultar"))

    # Si ya existe un paciente con ese DNI, no permitimos crear otro
    existing = store.find_paciente_by_dni(dni)
    if existing:
        rosio.pub_state('PATIENT_DUPLICATE_DNI')
        flash("Ya existe un paciente registrado con ese DNI.", "error")
        return redirect(url_for("consultar.consultar"))

    try:
        pac = store.create_paciente_if_allowed(nombre, apellidos, dni)
    except ValueError as e:
        flash(str(e), "error")
        return redirect(url_for("consultar.consultar"))

    rosio.pub_state('PATIENT_CREATED')

    # Reutilizamos la misma lógica de búsqueda de recetas activas
    meds, msg = _get_meds_for_patient(
        pac,
        "Paciente creado, pero no tiene recetas activas."
    )
    has_meds = bool(meds)

    if has_meds:
        rosio.pub_state('PATIENT_FOUND')
    else:
        rosio.pub_state('NO_RECIPES')

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
