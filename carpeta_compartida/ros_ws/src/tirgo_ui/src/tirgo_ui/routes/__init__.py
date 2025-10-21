from .main import bp as bp_main
from .consultar import bp as bp_consultar
from .leer import bp as bp_leer
from .diagnostico import bp as bp_diag

all_blueprints = [bp_main, bp_consultar, bp_leer, bp_diag]
