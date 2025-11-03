from flask import Blueprint, request, render_template, redirect, url_for, abort
from ..storage_mongo import (
    lookup_medicamento_by_name,
    lookup_medicamento_by_id,
    get_stock,
    h_dni,
    find_or_create_paciente,
    paciente_necesita_restringido,
    tiene_receta_activa,
)
from ..services import dispense_physical
from .. import session, rosio
import threading

bp = Blueprint('leer', __name__)

@bp.route('/leer', methods=['GET','POST'])
def leer():
    if not session.is_active():
        return redirect(url_for('main.index'))

    if request.method == 'GET':
        rosio.pub_state('MODE_SELECTED')
        return render_template('leer.html')

    name = (request.form.get('med') or '').strip()
    med = lookup_medicamento_by_name(name)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        return render_template('leer.html', msg='Medicamento no encontrado', error=True)

    if med.get('tipo') != 'R':
        if get_stock(med['id']) <= 0:
            rosio.pub_error('NO_STOCK', 'Sin stock')
            return render_template('leer.html', msg='Sin stock', error=True)
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

    med_id    = int(request.form.get('med_id'))
    nombre    = (request.form.get('nombre') or '').strip()
    apellidos = (request.form.get('apellidos') or '').strip()
    dni       = (request.form.get('dni') or '').strip()

    med = lookup_medicamento_by_id(med_id)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe'); abort(404)

    pid = find_or_create_paciente(nombre, apellidos, dni)
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
