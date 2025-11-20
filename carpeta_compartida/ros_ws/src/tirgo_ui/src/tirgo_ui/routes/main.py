# src/tirgo_ui/routes/main.py
from flask import Blueprint, render_template, redirect, url_for, jsonify
from jinja2 import TemplateNotFound
import os
from .. import session
from .. import rosio   # para saber en qué estado está la conversación

bp = Blueprint("main", __name__)

@bp.get("/")
def index():
    # si no hay sesión activa -> mostrar pantalla de hotword / conversación
    if not session.is_active():
        conv_state = rosio.conv_state()
        try:
            # le pasamos el estado para que la plantilla pueda decir:
            # "Tiago: ¿quieres empezar?"
            return render_template("await_hotword.html", conv_state=conv_state)
        except TemplateNotFound:
            try:
                # fallback al index_v2 pero con view explícita
                if conv_state == "await_confirm":
                    # estamos en la parte de "te he hablado, dime que sí"
                    return render_template("index_v2.html",
                                            view="await_hotword",
                                            msg="Tiago: estoy aquí para ayudarte, dime 'sí' o 'adelante' para empezar.",
                                            error=False,
                                            window_secs=10)
                else:
                    return render_template("index_v2.html",
                                            view="await_hotword",
                                            window_secs=10)
            except TemplateNotFound:
                # último fallback super básico
                if conv_state == "await_confirm":
                    return "<h3>Tiago: ¿quieres empezar? Dime 'sí'.</h3>", 200
                return "<h3>Di la hotword para desbloquear</h3>", 200

    # si hay sesión activa -> mostrar menú
    return render_template("menu.html")

@bp.get("/session_status")
def session_status():
    return jsonify({"active": session.is_active()})

@bp.get("/voice_nav")
def voice_nav():
    """Devuelve y limpia el intent de navegación por voz."""
    from .. import rosio
    nav = rosio.get_and_clear_voice_nav()
    return jsonify({"nav": nav})

@bp.post("/simulate_hola")
def simulate_hola():
    if os.environ.get("TIRGO_DEV") != "1":
        return ("", 404)
    session.start_session(op_name="dev_sim")
    return jsonify({"ok": True, "active": True})

@bp.post("/cancelar")
def cancelar_operacion():
    session.end_session()
    rosio.reset_conv()   # volvemos al estado idle también
    return redirect(url_for("main.index"))

# ✨ para poder volver desde el logo
@bp.get("/reset_hotword")
def reset_hotword():
    # cerramos la sesión actual (si la hay)
    session.end_session()
    rosio.reset_conv()   # reseteamos la conversación
    # y volvemos al index, que ya enseñará await_hotword
    return redirect(url_for("main.index"))
