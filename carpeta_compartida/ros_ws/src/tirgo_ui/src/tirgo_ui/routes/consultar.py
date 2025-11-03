from flask import Blueprint, request, render_template, redirect, url_for
from ..storage_mongo import find_or_create_paciente, permitted_meds_for_patient, h_dni
from .. import session, rosio

bp = Blueprint('consultar', __name__)

@bp.route('/consultar', methods=['GET','POST'])
def consultar():
    if not session.is_active():
        return redirect(url_for('main.index'))

    if request.method == 'GET':
        rosio.pub_state('IDENTIFYING')
        return render_template('consultar.html')

    nombre    = (request.form.get('nombre') or '').strip()
    apellidos = (request.form.get('apellidos') or '').strip()
    dni       = (request.form.get('dni') or '').strip()

    pid = find_or_create_paciente(nombre, apellidos, dni)
    if not pid:
        rosio.pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('consultar.html', msg='Datos no válidos', error=True)

    allowed = permitted_meds_for_patient(pid)
    rosio.pub_state('VALIDATED')

    return render_template(
        'consultar_result.html',
        paciente={'nombre': nombre, 'apellidos': apellidos, 'dni_hash': h_dni(dni)},
        allowed=allowed
    )
