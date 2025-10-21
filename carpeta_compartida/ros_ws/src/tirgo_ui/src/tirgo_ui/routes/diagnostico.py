from flask import Blueprint, render_template, request, url_for, redirect
from .. import hotword, rosio
from ..config import DB_PATH

bp = Blueprint('diagnostico', __name__)

@bp.route('/diagnostico', methods=['GET','POST'])
def diagnostico():
    if not hotword.is_active():
        return redirect(url_for('main.index'))
    if request.method == 'GET':
        rosio.pub_state('MODE_SELECTED')
        return render_template('diagnostico.html', db_path=DB_PATH)
    # si quieres procesar server-side, añade aquí la lógica;
    # por ahora lo hace el JS y redirige a /leer con la recomendación.
    return redirect(url_for('leer.leer'))
