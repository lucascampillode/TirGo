from flask import Blueprint, jsonify, render_template
from .. import hotword, rosio
from ..config import WINDOW_SECS, DB_PATH

bp = Blueprint('main', __name__)

@bp.after_app_request
def no_cache(resp):
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    return resp

@bp.get('/healthz')
def healthz():
    import os
    return jsonify({'ok': True, 'db': os.path.exists(DB_PATH)})

@bp.get('/state')
def state():
    flags = rosio.flags()
    return jsonify({'hotword': hotword.is_active(),
                    'remaining': hotword.remaining(),
                    'window_secs': WINDOW_SECS,
                    **flags})

@bp.post('/simulate_hola')
def simulate_hola():
    hotword.bump_now()
    return jsonify({'ok': True, 'hotword': True})

@bp.route('/')
def index():
    rosio.pub_state('MODE_SELECTED')
    if not hotword.is_active():
        return render_template('await_hotword.html', window_secs=WINDOW_SECS, db_path=DB_PATH)
    return render_template('menu.html', db_path=DB_PATH)
