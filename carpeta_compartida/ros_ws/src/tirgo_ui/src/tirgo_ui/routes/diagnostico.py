# src/tirgo_ui/routes/diagnostico.py
from flask import Blueprint, request, render_template, redirect, url_for
from .. import session, rosio

bp = Blueprint('diagnostico', __name__)


@bp.route('/diagnostico', methods=['GET', 'POST'])
def diagnostico():
    """
    Vista de diagnóstico guiado.

    - GET: muestra el cuestionario y publica estado DIAG_START.
    - POST: opcionalmente podría usarse para un botón de 'finalizar', pero
      el flujo normal del wizard NO hace POST aquí; sólo hace POST a /leer.
    """
    if not session.is_active():
        return redirect(url_for('main.index'))

    if request.method == 'GET':
        rosio.pub_state('DIAG_START')
        return render_template('diagnostico.html')

    # POST opcional por si algún día añades un botón de 'finalizar diagnóstico'
    decision = (request.form.get('decision') or '').strip().lower()
    if decision == 'finalizar':
        session.end_session()
        return redirect(url_for('main.index'))

    # Por defecto, vuelve a mostrar la vista
    return render_template('diagnostico.html', msg='Acción registrada')
