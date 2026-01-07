from flask import Blueprint

tirgo_ui = Blueprint(
    "tirgo_ui",               # ← este es el nombre que usaremos en url_for(...)
    __name__,
    template_folder="templates",
    static_folder="static"    # ← apunta a src/tirgo_ui/static
)
