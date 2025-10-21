from flask import Blueprint, request, render_template, redirect, url_for, abort
from ..storage import lookup_medicamento_by_name, get_stock, db, h_dni, find_or_create_paciente
from ..services import dispense_physical
from ..config import DB_PATH
from .. import hotword, rosio
import threading

bp = Blueprint('leer', __name__)

@bp.route('/leer', methods=['GET','POST'])
def leer():
    if not hotword.is_active():
        return redirect(url_for('main.index'))
    if request.method == 'GET':
        rosio.pub_state('MODE_SELECTED')
        return render_template('leer.html', db_path=DB_PATH)

    name = request.form.get('med','').strip()
    med = lookup_medicamento_by_name(name)
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe')
        return render_template('leer.html', msg='Medicamento no encontrado', error=True, db_path=DB_PATH)

    if med['tipo'] != 'R':
        if get_stock(med['id']) <= 0:
            rosio.pub_error('NO_STOCK', 'Sin stock')
            return render_template('leer.html', msg='Sin stock', error=True, db_path=DB_PATH)
        try:
            if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
            if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
        except Exception:
            pass
        threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
        return render_template('status.html', state='DISPENSING', msg='Preparando tu producto', db_path=DB_PATH)

    rosio.pub_state('IDENTIFYING')
    return render_template('leer_ident.html', med=med, db_path=DB_PATH)

@bp.route('/recoger/<int:med_id>')
def recoger(med_id: int):
    if not hotword.is_active():
        return redirect(url_for('main.index'))
    conn = db(); med = conn.execute('SELECT * FROM medicamentos WHERE id=?', (med_id,)).fetchone(); conn.close()
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe'); abort(404)
    if get_stock(med_id) <= 0:
        rosio.pub_error('NO_STOCK', 'Sin stock')
        return render_template('status.html', state='ERROR', msg='Sin stock', error=True, db_path=DB_PATH)

    try:
        rosio.pub_status({'msg':'Orden enviada a TIAGO y dispensador',
                          'bin_id': int(med['bin_id']), 'med': med['nombre']})
        if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, None), daemon=True).start()
    return render_template('status.html', state='DISPENSING', msg='Preparando tu producto', db_path=DB_PATH)

@bp.route('/leer/ident', methods=['POST'])
def leer_ident():
    if not hotword.is_active():
        return redirect(url_for('main.index'))
    med_id = int(request.form.get('med_id'))
    nombre = request.form.get('nombre','').strip()
    apellidos = request.form.get('apellidos','').strip()
    dni = request.form.get('dni','').strip()

    conn = db(); med = conn.execute('SELECT * FROM medicamentos WHERE id=?', (med_id,)).fetchone(); conn.close()
    if not med:
        rosio.pub_error('NOT_FOUND', 'Medicamento no existe'); abort(404)

    pid = find_or_create_paciente(nombre, apellidos, dni)
    if not pid:
        rosio.pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('leer_ident.html', med=med, msg='Datos no válidos', error=True, db_path=DB_PATH)

    conn = db()
    necesita = conn.execute('SELECT necesita_restringido FROM pacientes WHERE id=?',(pid,)).fetchone()
    necesita = int(necesita['necesita_restringido']) if necesita else 0
    r = conn.execute('SELECT 1 FROM recetas WHERE paciente_id=? AND medicamento_id=? AND activa=1',(pid, med_id)).fetchone()
    tiene_receta = 1 if r else 0
    conn.close()

    if med['tipo'] == 'R' and not (tiene_receta or necesita):
        rosio.pub_error('RESTRICTED', 'No autorizado para este R')
        return render_template('status.html', state='ERROR',
                               msg='Este medicamento requiere receta válida. Visita a tu médico.',
                               error=True, db_path=DB_PATH)

    if get_stock(med_id) <= 0:
        rosio.pub_error('NO_STOCK', 'Sin stock')
        return render_template('status.html', state='ERROR', msg='Sin stock', error=True, db_path=DB_PATH)

    try:
        if rosio.pub_mission_start: rosio.pub_mission_start.publish('go_pickup')
        if rosio.pub_dispense_req:  rosio.pub_dispense_req.publish(int(med['bin_id']))
    except Exception:
        pass

    threading.Thread(target=dispense_physical, args=(med, h_dni(dni)), daemon=True).start()
    return render_template('status.html', state='DISPENSING', msg='Preparando tu producto', db_path=DB_PATH)
