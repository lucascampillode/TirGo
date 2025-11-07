from flask import Blueprint, render_template, redirect, url_for, jsonify
from jinja2 import TemplateNotFound
import os
from .. import session

bp = Blueprint("main", __name__)

@bp.get("/")
def index():
    # si no hay sesión activa -> mostrar pantalla de hotword
    if not session.is_active():
        try:
            return render_template("await_hotword.html")
        except TemplateNotFound:
            try:
                return render_template("index_v2.html")
            except TemplateNotFound:
                return "<h3>Di la hotword para desbloquear</h3>", 200
    # si hay sesión activa -> mostrar menú
    return render_template("menu.html")

@bp.get("/session_status")
def session_status():
    return jsonify({"active": session.is_active()})

@bp.post("/simulate_hola")
def simulate_hola():
    if os.environ.get("TIRGO_DEV") != "1":
        return ("", 404)
    session.start_session(op_name="dev_sim")
    return jsonify({"ok": True, "active": True})

@bp.post("/cancelar")
def cancelar_operacion():
    session.end_session()
    return redirect(url_for("main.index"))

# ✨ NUEVO: para poder volver desde el logo
@bp.get("/reset_hotword")
def reset_hotword():
    # cerramos la sesión actual (si la hay)
    session.end_session()
    # y volvemos al index, que ya enseñará await_hotword
    return redirect(url_for("main.index"))
