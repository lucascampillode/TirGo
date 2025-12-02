# src/tirgo_ui/routes/leer.py
from flask import Blueprint, request, render_template, redirect, url_for, abort, flash, current_app, jsonify
from bson import ObjectId

from ..storage_mongo import (
    meds_disponibles,
    lookup_medicamento_by_id,
    dec_stock_if_available,
    h_dni,
    create_paciente_if_allowed,
    paciente_necesita_restringido,
    tiene_receta_activa,
)
from .. import session, rosio

# Reutilizamos la validación de DNI / nombre de consultar.py
from .consultar import _normalize_dni, _is_valid_dni, _is_valid_name

bp = Blueprint('leer', __name__)


# --- Utilidad: obtener handler de DB por si hace falta tocar recetas ---
def _get_db_from_storage():
    from .. import storage_mongo as store
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


def _check_stock_and_render_if_empty(med: dict, med_id: int):
    """
    Comprueba el stock intentando descontar 1 unidad con dec_stock_if_available.

    - Si NO se puede descontar (no hay stock suficiente), devuelve directamente
      un render_template('status.html', ... NO_STOCK ...).
    - Si SÍ se descuenta, devuelve None y la ruta continúa (el stock ya ha bajado).

    Es decir: en cuanto el usuario confirma el pedido (Pedir / Recoger / Ident),
    se quita 1 de stock aquí.
    """
    try:
        ok = dec_stock_if_available(med_id, 1)
    except Exception:
        ok = False

    if not ok:
        med_name = med.get('nombre', 'este medicamento')
        rosio.pub_error('NO_STOCK', f"Sin stock de {med_name}")
        msg = (
            f"No queda stock de «{med_name}». "
            "Estará disponible aproximadamente en 3 días. "
            "Si las molestias son importantes o empeoran, acude a tu médico."
        )
        return render_template(
            'status.html',
            state='NO_STOCK',
            body_msg=msg,
            error=True,
            show_retry=False
        )
    return None


def _resolve_bin_id(med: dict, fallback_med_id: int) -> int:
    """
    Obtiene el bin_id que usará la misión.
    Intenta extraerlo de med['bin_id'], y si falla, usa el med_id como fallback.
    """
    try:
        return int(med.get('bin_id'))
    except Exception:
        return int(fallback_med_id)


# ----------------------------- RUTAS -----------------------------

@bp.route('/leer', methods=['GET'], endpoint='leer')
def leer_get():
    """Vista de selección de medicamento (libre o con receta)."""
    try:
        if not session.is_active():
            return redirect(url_for('main.index'))
        rosio.pub_state('MODE_SELECTED')

        try:
            meds = meds_disponibles()
        except Exception:
            current_app.logger.exception("meds_disponibles() falló")
            meds = []
            flash("No puedo listar medicamentos ahora mismo.", "error")

        return render_template('leer.html', meds=meds), 200

    except Exception as e:
        current_app.logger.exception("leer_get explotó")
        rosio.pub_error("LEER_GET_ERR", str(e))
        flash("Ha ocurrido un error cargando la vista de pedido.", "error")
        # Fallback básico (sin tumbar la app)
        return render_template('leer.html', meds=[]), 200


@bp.route('/leer', methods=['POST'], endpoint='leer_post')
def leer_post():
    """Procesa la petición de leer (desde Diagnóstico o desde la propia vista)."""
    try:
        if not session.is_active():
            return redirect(url_for('main.index'))

        med_id_raw = request.form.get('med_id')
        try:
            med_id = int(med_id_raw)
        except (TypeError, ValueError):
            rosio.pub_error('BAD_REQ', 'Selección inválida')
            flash('Selección inválida.', 'error')
            return redirect(url_for('leer.leer'))

        med = lookup_medicamento_by_id(med_id)
        if not med:
            rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
            flash('No he encontrado ese medicamento.', 'error')
            return redirect(url_for('leer.leer'))

        tipo = (med.get('tipo') or '').strip().upper()

        try:
            current_app.logger.info(
                f"[leer_post] med_id={med_id} -> nombre={med.get('nombre')} tipo={tipo} bin={med.get('bin_id')}"
            )
        except Exception:
            pass

        # --- LIBRE: dispensa directa ---
        if tipo != 'R':
            # Aquí ya intentamos descontar 1 de stock.
            no_stock_resp = _check_stock_and_render_if_empty(med, med_id)
            if no_stock_resp is not None:
                return no_stock_resp

            # Lanzar misión vía Action Server (sin paciente explícito)
            bin_id = _resolve_bin_id(med, med_id)
            rosio.start_mission_async("", bin_id)
            rosio.pub_state('DISPENSING')

            session.end_session()
            return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')

        # --- RESTRINGIDO: pasar a identificación ---
        rosio.pub_state('IDENTIFYING')
        return render_template('leer_ident.html', med=med)

    except Exception as e:
        current_app.logger.exception("leer_post explotó")
        rosio.pub_error("LEER_POST_ERR", str(e))
        flash("Ha ocurrido un error procesando tu petición.", "error")
        return redirect(url_for('leer.leer'))


@bp.route('/recoger/<int:med_id>')
def recoger(med_id: int):
    """Dispensa directa cuando venimos de 'Consultar' (permitidos) para no-R."""
    if not session.is_active():
        return redirect(url_for('main.index'))

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        abort(404)

    # Aquí ya intentamos descontar 1 de stock.
    no_stock_resp = _check_stock_and_render_if_empty(med, med_id)
    if no_stock_resp is not None:
        return no_stock_resp

    # Informar a la UI y lanzar misión vía Action Server
    bin_id = _resolve_bin_id(med, med_id)
    try:
        rosio.pub_status({
            'msg': 'Orden enviada a TIAGO y dispensador',
            'bin_id': bin_id,
            'med': med.get('nombre'),
        })
    except Exception:
        pass

    rosio.start_mission_async("", bin_id)
    rosio.pub_state('DISPENSING')

    session.end_session()
    return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')


@bp.route('/leer/ident', methods=['POST'])
def leer_ident():
    """Identificación para tipo R o cuando venimos del flujo 'Consultar'."""
    if not session.is_active():
        return redirect(url_for('main.index'))

    med_id_raw = request.form.get('med_id')
    if not med_id_raw:
        rosio.pub_error('BAD_REQ', 'Falta med_id')
        return redirect(url_for('leer.leer'))

    # Validar med_id igual que en leer_post
    try:
        med_id = int(med_id_raw)
    except (TypeError, ValueError):
        rosio.pub_error('BAD_REQ', 'Selección inválida')
        flash('Selección inválida.', 'error')
        return redirect(url_for('leer.leer'))

    # Si venimos de 'Consultar' con paciente/receta ya decididos:
    skip_ident    = request.form.get('skip_ident')
    paciente_id_s = request.form.get('paciente_id')
    receta_id_s   = request.form.get('receta_id')

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        abort(404)

    # ---- Camino 'Consultar' (sin pedir datos otra vez) ----
    if skip_ident == "1" and paciente_id_s:
        try:
            pid = ObjectId(paciente_id_s)
        except Exception:
            pid = None

        if pid is not None:
            necesita     = paciente_necesita_restringido(pid)
            tiene_receta = tiene_receta_activa(pid, med_id)

            if (med.get('tipo') or '').strip().upper() == 'R' and not (tiene_receta or necesita):
                rosio.pub_error('RESTRICTED', 'No autorizado para este R')
                return render_template(
                    'status.html',
                    state='ERROR',
                    body_msg='Este medicamento requiere receta válida. Visita a tu médico.',
                    error=True,
                    show_retry=False
                )

            # Aquí ya intentamos descontar 1 de stock.
            no_stock_resp = _check_stock_and_render_if_empty(med, med_id)
            if no_stock_resp is not None:
                return no_stock_resp

            # Lanzar misión con ID de paciente (ObjectId en string)
            bin_id = _resolve_bin_id(med, med_id)
            rosio.start_mission_async(str(pid), bin_id)
            rosio.pub_state('DISPENSING')

            # Desactivar la receta usada (si venía)
            if receta_id_s:
                db = _get_db_from_storage()
                if db is not None:
                    try:
                        db.recetas.update_one({"_id": ObjectId(receta_id_s)},
                                              {"$set": {"activa": False}})
                    except Exception as e:
                        current_app.logger.warning("[leer] no se pudo desactivar la receta: %s", e)

            session.end_session()
            return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')

    # ---- Camino clásico: formulario de identificación ----
    nombre    = (request.form.get('nombre') or '').strip()
    apellidos = (request.form.get('apellidos') or '').strip()
    dni_raw   = (request.form.get('dni') or '').strip()
    dni       = _normalize_dni(dni_raw)

    # Validar nombre, apellidos y DNI como en consultar.py
    if not _is_valid_name(nombre):
        rosio.pub_error('ID_FAIL', 'Nombre no válido')
        return render_template('leer_ident.html', med=med, msg='Nombre no válido.', error=True)

    if not _is_valid_name(apellidos):
        rosio.pub_error('ID_FAIL', 'Apellidos no válidos')
        return render_template('leer_ident.html', med=med, msg='Los apellidos no son válidos.', error=True)

    if not dni or not _is_valid_dni(dni):
        rosio.pub_error('ID_FAIL', 'DNI no válido')
        return render_template('leer_ident.html', med=med, msg='DNI no válido.', error=True)

    try:
        pac = create_paciente_if_allowed(nombre, apellidos, dni)
    except ValueError as e:
        rosio.pub_error('ID_FAIL', 'No se pudo crear/validar paciente')
        return render_template('leer_ident.html', med=med, msg=str(e), error=True)

    pid = pac["_id"] if pac else None
    if not pid:
        rosio.pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('leer_ident.html', med=med, msg='Datos no válidos', error=True)

    necesita     = paciente_necesita_restringido(pid)
    tiene_receta = tiene_receta_activa(pid, med_id)

    if (med.get('tipo') or '').strip().upper() == 'R' and not (tiene_receta or necesita):
        rosio.pub_error('RESTRICTED', 'No autorizado para este R')
        return render_template(
            'status.html',
            state='ERROR',
            body_msg='Este medicamento requiere receta válida. Visita a tu médico.',
            error=True,
            show_retry=False
        )

    # Hash del DNI para la misión (no mandamos el DNI en claro)
    patient_hash = h_dni(dni)

    # Aquí ya intentamos descontar 1 de stock.
    no_stock_resp = _check_stock_and_render_if_empty(med, med_id)
    if no_stock_resp is not None:
        return no_stock_resp

    # Lanzar misión con hash de paciente
    bin_id = _resolve_bin_id(med, med_id)
    rosio.start_mission_async(patient_hash, bin_id)
    rosio.pub_state('DISPENSING')

    session.end_session()
    return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')


# --- API para estado de misión (polling desde la web) ---

@bp.route("/leer/mission_status", methods=["GET"])
def leer_mission_status():
    """Devuelve el estado actual de la misión para el polling de la UI."""
    return jsonify(rosio.get_mission_status())
