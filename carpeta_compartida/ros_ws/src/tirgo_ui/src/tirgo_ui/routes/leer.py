from flask import Blueprint, request, render_template, redirect, url_for, abort, flash
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
from bson import ObjectId  # 👈 lo teníamos ya

bp = Blueprint('leer', __name__)

# 👇 añadimos esto para poder acceder a la db y editar la receta
def _get_db_from_storage():
    from .. import storage_mongo as store
    # probamos los nombres típicos que usa tu proyecto
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


@bp.route('/leer', methods=['GET'], endpoint='leer')
def leer_get():
    if not session.is_active():
        return redirect(url_for('main.index'))
    rosio.pub_state('MODE_SELECTED')
    meds = meds_disponibles()
    return render_template('leer.html', meds=meds)


@bp.route('/leer', methods=['POST'], endpoint='leer_post')
def leer_post():
    if not session.is_active():
        return redirect(url_for('main.index'))

    med_id = request.form.get('med_id')
    try:
        med_id = int(med_id)
    except (TypeError, ValueError):
        rosio.pub_error('BAD_REQ', 'Selección inválida')
        return redirect(url_for('leer.leer_get'))

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        return redirect(url_for('leer.leer_get'))

    if med.get('tipo') != 'R':
        if get_stock(med['id']) <= 0:
            rosio.pub_error('NO_STOCK', 'Sin stock')
            flash('Sin stock', 'error')
            return redirect(url_for('leer.leer_get'))
        try:
            if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
            if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
        except Exception:
            pass
        threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
        session.end_session()
        return render_template('status.html', state='DISPENSING', msg='Preparando tu producto')

    rosio.pub_state('IDENTIFYING')
    return render_template('leer_ident.html', med=med)


@bp.route('/recoger/<int:med_id>')
def recoger(med_id: int):
    if not session.is_active():
        return redirect(url_for('main.index'))

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        abort(404)

    if get_stock(med_id) <= 0:
        rosio.pub_error('NO_STOCK', 'Sin stock')
        return render_template('status.html', state='ERROR', msg='Sin stock', error=True)

    try:
        rosio.pub_status({'msg':'Orden enviada a TIAGO y dispensador',
                          'bin_id': int(med['bin_id']), 'med': med['nombre']})
        if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
    session.end_session()
    return render_template('status.html', state='DISPENSING', msg='Preparando tu producto')


@bp.route('/leer/ident', methods=['POST'])
def leer_ident():
    if not session.is_active():
        return redirect(url_for('main.index'))

    med_id_raw = request.form.get('med_id')
    if not med_id_raw:
        return redirect(url_for('leer.leer_get'))
    med_id = int(med_id_raw)

    # 👇 vienen cuando se pide desde consultar
    skip_ident    = request.form.get('skip_ident')
    paciente_id_s = request.form.get('paciente_id')
    receta_id_s   = request.form.get('receta_id')

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe'); abort(404)

    # ---------- CAMINO NUEVO: venimos de consultar ----------
    if skip_ident == "1" and paciente_id_s:
        try:
            pid = ObjectId(paciente_id_s)
        except Exception:
            pid = None

        if pid is not None:
            necesita     = paciente_necesita_restringido(pid)
            tiene_receta = tiene_receta_activa(pid, med_id)

            if med.get('tipo') == 'R' and not (tiene_receta or necesita):
                rosio.pub_error('RESTRICTED', 'No autorizado para este R')
                return render_template('status.html', state='ERROR',
                                       msg='Este medicamento requiere receta válida. Visita a tu médico.',
                                       error=True)

            if get_stock(med_id) <= 0:
                rosio.pub_error('NO_STOCK', 'Sin stock')
                return render_template('status.html', state='ERROR', msg='Sin stock', error=True)

            # DISPENSAR
            try:
                if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
                if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
            except Exception:
                pass

            threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()

            # 👇 DESACTIVAR LA RECETA QUE HEMOS USADO
            if receta_id_s:
                db = _get_db_from_storage()
                if db is not None:
                    try:
                        db.recetas.update_one(
                            {"_id": ObjectId(receta_id_s)},
                            {"$set": {"activa": False}}
                        )
                    except Exception as e:
                        # si falla no rompemos la dispensación
                        print("[leer] no se pudo desactivar la receta:", e)

            session.end_session()
            return render_template('status.html', state='DISPENSING', msg='Preparando tu producto')

    # ---------- CAMINO ANTIGUO: formulario de identificación ----------
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

    if med.get('tipo') == 'R' and not (tiene_receta or necesita):
        rosio.pub_error('RESTRICTED', 'No autorizado para este R')
        return render_template('status.html', state='ERROR',
                               msg='Este medicamento requiere receta válida. Visita a tu médico.',
                               error=True)

    if get_stock(med_id) <= 0:
        rosio.pub_error('NO_STOCK', 'Sin stock')
        return render_template('status.html', state='ERROR', msg='Sin stock', error=True)

    try:
        if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, h_dni(dni)), daemon=True).start()
    session.end_session()
    return render_template('status.html', state='DISPENSING', msg='Preparando tu producto')
