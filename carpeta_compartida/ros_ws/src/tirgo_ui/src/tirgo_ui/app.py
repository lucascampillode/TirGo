import os
import logging
from flask import Flask, jsonify

from .config import TEMPLATE_DIR, STATIC_DIR, PORT, DEBUG
from .storage_mongo import init_db_if_needed
from .routes import all_blueprints
from . import rosio

# --- logging básico ---
logger = logging.getLogger(__name__)
if not logging.getLogger().hasHandlers():
    logging.basicConfig(level=logging.INFO)


APP: Flask = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)
APP.secret_key = os.environ.get("FLASK_SECRET_KEY", "dev-secret")  # cambia "dev-secret" en prod


def _check_security_settings() -> None:
    """
    Comprueba parámetros sensibles de configuración y muestra avisos
    si la aplicación se ha levantado con valores inseguros.

    - FLASK_SECRET_KEY: protege las cookies de sesión.
    - TIRGO_PEPPER: se usa para hashear el DNI de los pacientes.
    """
    secret = APP.secret_key
    if not secret or secret == "dev-secret":
        logger.warning(
            "[SECURITY] FLASK_SECRET_KEY no está definida o está usando el valor por defecto "
            "('dev-secret'). En despliegue real debe establecerse a un valor aleatorio y secreto."
        )

    pepper = os.environ.get("TIRGO_PEPPER", "")
    if not pepper:
        logger.warning(
            "[SECURITY] La variable de entorno TIRGO_PEPPER no está definida. "
            "Los DNI se seguirán hasheando, pero sin un 'pepper' fuerte la protección es menor."
        )

    if not DEBUG and (secret == "dev-secret" or not pepper):
        logger.warning(
            "[SECURITY] La aplicación está en DEBUG=False pero se han detectado parámetros "
            "de seguridad débiles. Revisa FLASK_SECRET_KEY y TIRGO_PEPPER antes de un despliegue real."
        )


def _register() -> None:
    # Comprobar seguridad antes de levantar nada serio
    _check_security_settings()

    # pubs/subs ROS
    rosio.init()

    # índices/semillas idempotentes en Mongo
    try:
        init_db_if_needed()
    except Exception as e:
        print("[storage_mongo.init_db_if_needed] warning:", e)

    # registrar blueprints de Flask
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
