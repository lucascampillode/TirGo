# src/tirgo_ui/routes/leer.py
from flask import Blueprint, request, render_template, redirect, url_for, abort, flash, current_app
from ..storage_mongo import (
    meds_disponibles,
    lookup_medicamento_by_id,
    get_stock,
    h_dni,
    create_paciente_if_allowed,
    paciente_necesita_restringido,
    tiene_receta_activa,
)
from ..services import dispense_physical
from .. import session, rosio
import threading
from bson import ObjectId

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

        med_id = request.form.get('med_id')
        try:
            med_id = int(med_id)
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
            try:
                stock_val = int(get_stock(med['id']))
            except Exception:
                stock_val = 0
            if stock_val <= 0:
                rosio.pub_error('NO_STOCK', f"Sin stock de {med['nombre']}")
                msg = (
                    f"No queda stock de «{med['nombre']}». "
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
            try:
                if rosio.pub_mission_start:
                    rosio.pub_mission_start.publish('go_pickup')
                if rosio.pub_dispense_req:
                    rosio.pub_dispense_req.publish(int(med['bin_id']))
            except Exception:
                pass

            threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
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

    try:
        stock_val = int(get_stock(med_id))
    except Exception:
        stock_val = 0

    if stock_val <= 0:
        rosio.pub_error('NO_STOCK', f"Sin stock de {med.get('nombre','')}")
        msg = (
            f"No queda stock de «{med.get('nombre','este medicamento')}». "
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

    try:
        rosio.pub_status({'msg': 'Orden enviada a TIAGO y dispensador',
                          'bin_id': int(med['bin_id']), 'med': med['nombre']})
        if rosio.pub_mission_start:
            rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:
            rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
    session.end_session()
    return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')


@bp.route('/leer/ident', methods=['POST'])
def leer_ident():
    """Identificación para tipo R o cuando venimos del flujo 'Consultar'."""
    if not session.is_active():
        return redirect(url_for('main.index'))

    med_id_raw = request.form.get('med_id')
    if not med_id_raw:
        return redirect(url_for('leer.leer'))
    med_id = int(med_id_raw)

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

            try:
                stock_val = int(get_stock(med_id))
            except Exception:
                stock_val = 0

            if stock_val <= 0:
                rosio.pub_error('NO_STOCK', f"Sin stock de {med.get('nombre','')}")
                msg = (
                    f"No queda stock de «{med.get('nombre','este medicamento')}». "
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

            try:
                if rosio.pub_mission_start:
                    rosio.pub_mission_start.publish('go_pickup')
                if rosio.pub_dispense_req:
                    rosio.pub_dispense_req.publish(int(med['bin_id']))
            except Exception:
                pass

            threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()

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
    dni       = (request.form.get('dni') or '').strip()

    pac = create_paciente_if_allowed(nombre, apellidos, dni)
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

    try:
        stock_val = int(get_stock(med_id))
    except Exception:
        stock_val = 0

    if stock_val <= 0:
        rosio.pub_error('NO_STOCK', f"Sin stock de {med.get('nombre','')}")
        msg = (
            f"No queda stock de «{med.get('nombre','este medicamento')}». "
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

    try:
        if rosio.pub_mission_start:
            rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:
            rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, h_dni(dni)), daemon=True).start()
    session.end_session()
    return render_template('status.html', state='DISPENSING', body_msg='Preparando tu producto')
