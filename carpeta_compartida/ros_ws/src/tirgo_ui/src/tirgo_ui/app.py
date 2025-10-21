from flask import Flask
from .config import TEMPLATE_DIR, STATIC_DIR
from .storage import init_db_if_needed
from .routes import all_blueprints
from . import rosio

APP: Flask = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)

def _register():
    init_db_if_needed()
    rosio.init()
    for bp in all_blueprints:
        APP.register_blueprint(bp)

_register()

if __name__ == '__main__':
    rosio.pub_state('IDLE')
    import os
    APP.run(host='0.0.0.0', port=int(os.environ.get('PORT','8000')), debug=False, use_reloader=False)
