# src/tirgo_ui/routes/main.py
from flask import Blueprint, render_template, redirect, url_for, jsonify
from jinja2 import TemplateNotFound
import os
from .. import session
from .. import rosio   # para saber en qué estado está la conversación

bp = Blueprint("main", __name__)


@bp.get("/")
def index():
    # ¿estamos en modo desarrollo?
    dev_mode = os.environ.get("TIRGO_DEV", "1") == "1"

    # si no hay sesión activa -> mostrar pantalla de hotword / conversación
    if not session.is_active():
        conv_state = rosio.conv_state()
        try:
            # plantilla de espera de hotword
            return render_template(
                "await_hotword.html",
                conv_state=conv_state,
                dev_mode=dev_mode,
            )
        except TemplateNotFound:
            try:
                # fallback al index_v2 pero con view explícita
                if conv_state == "await_confirm":
                    # estamos en la parte de "te he hablado, dime que sí"
                    return render_template(
                        "index_v2.html",
                        view="await_hotword",
                        msg="Tiago: estoy aquí para ayudarte, dime 'sí' o 'adelante' para empezar.",
                        error=False,
                        window_secs=10,
                        dev_mode=dev_mode,
                    )
                else:
                    return render_template(
                        "index_v2.html",
                        view="await_hotword",
                        window_secs=10,
                        dev_mode=dev_mode,
                    )
            except TemplateNotFound:
                # último fallback super básico
                if conv_state == "await_confirm":
                    return "<h3>Tiago: ¿quieres empezar? Dime 'sí'.</h3>", 200
                return "<h3>Di la hotword para desbloquear</h3>", 200

    # si hay sesión activa -> mostrar menú
    rosio.set_ui_menu("home")
    return render_template("menu.html")


@bp.get("/session_status")
def session_status():
    """Endpoint genérico para saber si hay sesión activa."""
    return jsonify({"active": session.is_active()})


@bp.get("/hotword_status")
def hotword_status():
    """
    Endpoint específico para la pantalla de hotword.

    La idea es que await_hotword.html haga polling aquí.
    En cuanto 'ready' sea True, el front puede redirigir a redirect_url.
    """
    active = session.is_active()
    return jsonify({
        "ready": bool(active),
        # cuando hay sesión activa, index() ya devuelve el menú
        "redirect_url": url_for("main.index") if active else None,
    })


@bp.get("/voice_nav")
def voice_nav():
    """Devuelve y limpia el intent de navegación por voz."""
    from .. import rosio
    nav = rosio.get_and_clear_voice_nav()
    return jsonify({"nav": nav})


@bp.post("/simulate_hola")
def simulate_hola():
    """
    Botón "Simular 'hola' (demo)":
    - SOLO funciona si TIRGO_DEV=1
    - Arranca sesión y redirige directamente al menú (sin JSON feo)
    """
    if os.environ.get("TIRGO_DEV") != "1":
        return ("", 404)

    session.start_session(op_name="dev_sim")
    return redirect(url_for("main.index"))


@bp.post("/cancelar")
def cancelar_operacion():
    """Botón de cancelar / volver atrás desde el flujo principal."""
    session.end_session()
    rosio.reset_conv()   # volvemos al estado idle también
    rosio.set_ui_menu("home")
    return redirect(url_for("main.index"))


# ✨ para poder volver desde el logo
@bp.get("/reset_hotword")
def reset_hotword():
    """
    Cierra la sesión actual (si la hay), resetea la conversación en ROS
    y vuelve al index, que ya enseñará await_hotword.
    """
    session.end_session()
    rosio.reset_conv()
    rosio.set_ui_menu("home")
    return redirect(url_for("main.index"))
