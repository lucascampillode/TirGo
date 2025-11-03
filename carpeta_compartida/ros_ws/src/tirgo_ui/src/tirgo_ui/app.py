from flask import Flask, jsonify
from .config import TEMPLATE_DIR, STATIC_DIR, PORT, DEBUG
from .storage_mongo import init_db_if_needed
from .routes import all_blueprints
from . import rosio

APP: Flask = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)

def _register():
    rosio.init()  # pubs/subs ROS
    try:
        init_db_if_needed()  # índices/semillas idempotentes
    except Exception as e:
        print("[storage_mongo.init_db_if_needed] warning:", e)
    for bp in all_blueprints:
        APP.register_blueprint(bp)

# --- healthcheck DB (opcional) ---
try:
    from .mongo_client import get_client, get_db
except Exception:
    get_client = lambda: None  # type: ignore
    get_db     = lambda: None  # type: ignore

@APP.route('/api/db_ping')
def db_ping():
    try:
        c = get_client()
        db = get_db()
        info = c.server_info() if c else {}
        return jsonify({
            "ok": True,
            "version": info.get("version"),
            "counts": {
                "pacientes": db.pacientes.estimated_document_count() if db else None,
                "medicamentos": db.medicamentos.estimated_document_count() if db else None,
                "recetas": db.recetas.estimated_document_count() if db else None,
            }
        }), 200
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

_register()

if __name__ == '__main__':
    rosio.pub_state('IDLE')
    APP.run(host='0.0.0.0', port=PORT, debug=DEBUG, use_reloader=False)
