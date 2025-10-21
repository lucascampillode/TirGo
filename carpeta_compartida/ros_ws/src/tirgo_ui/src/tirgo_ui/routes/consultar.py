from flask import Blueprint, request, render_template, redirect, url_for
from ..storage import find_or_create_paciente, permitted_meds_for_patient, h_dni
from ..config import DB_PATH
from .. import hotword, rosio

bp = Blueprint('consultar', __name__)

@bp.route('/consultar', methods=['GET','POST'])
def consultar():
    if not hotword.is_active():
        return redirect(url_for('main.index'))
    if request.method == 'GET':
        rosio.pub_state('IDENTIFYING')
        return render_template('consultar.html', db_path=DB_PATH)
    nombre = request.form.get('nombre','').strip()
    apellidos = request.form.get('apellidos','').strip()
    dni = request.form.get('dni','').strip()
    pid = find_or_create_paciente(nombre, apellidos, dni)
    if not pid:
        rosio.pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('consultar.html', msg='Datos no válidos', error=True, db_path=DB_PATH)
    allowed = permitted_meds_for_patient(pid)
    rosio.pub_state('VALIDATED')
    return render_template('consultar_result.html',
                           paciente={'nombre':nombre,'apellidos':apellidos,'dni_hash':h_dni(dni)},
                           allowed=allowed, db_path=DB_PATH)
