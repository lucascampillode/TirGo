# src/tirgo_ui/routes/diagnostico.py
from flask import Blueprint, request, render_template, redirect, url_for
from .. import session, rosio

bp = Blueprint('diagnostico', __name__)

@bp.route('/diagnostico', methods=['GET', 'POST'])
def diagnostico():
    if not session.is_active():
        return redirect(url_for('main.index'))

    if request.method == 'GET':
        rosio.pub_state('DIAG_START')
        return render_template('diagnostico.html')

    # --- POST: tu lógica de diagnóstico aquí ---
    decision = (request.form.get('decision') or '').strip().lower()
    if decision == 'finalizar':
        # si aquí consideras que termina la operación:
        session.end_session()
        return redirect(url_for('main.index'))

    # si no finaliza, seguir en la misma pantalla o redirigir:
    return render_template('diagnostico.html', msg='Acción registrada')
