"""
services.py — Modo experimental / legacy.

Actualmente NO se usa en el flujo principal de TirgoPharma.
Se conserva solo para pruebas en las que tirgo_ui controla directamente
el dispensador sin pasar por la misión ROS /tirgo/mission.
"""


import time
from typing import Optional

from .storage_mongo import (
    get_stock,
    dec_stock_if_available,
    log_dispense,
)
from . import rosio


def _do_physical_dispense(med: dict) -> None:
    """
    Aquí invocarías al hardware real. De momento simulamos con un sleep.

    Esta función es útil para tests o escenarios donde tirgo_ui
    controle directamente el dispensador (sin misión ROS completa).
    """
    time.sleep(1.0)


def dispense_physical(med: dict, dni_hash: Optional[str] = None) -> bool:
    """
    Secuencia de dispensado monolítica:

    - Comprueba stock.
    - Intenta "mover hardware" (simulado).
    - Si todo va bien, descuenta stock.
    - Registra log en Mongo.

    NO se usa en el flujo principal actual de TirgoPharma, donde
    el stock se descuenta directamente al pulsar 'Pedir' en leer.py.
    """
    med_id = int(med["id"])

    # 0) Comprobar stock ANTES de mover hardware
    stock = get_stock(med_id)
    if stock is not None and stock <= 0:
        rosio.pub_error("NO_STOCK", "Sin stock disponible para el medicamento.")
        log_dispense(med, dni_hash, ok=False, error="NO_STOCK_BEFORE_HW")
        return False

    # 1) Intentar el dispensado físico (simulado)
    try:
        _do_physical_dispense(med)
    except Exception as e:
        log_dispense(med, dni_hash, ok=False, error=f"HARDWARE_ERROR: {e}")
        rosio.pub_error("HARDWARE_ERROR", "Fallo en el dispensador")
        return False

    # 2) Si el dispensado ha ido bien, AHORA sí descontamos stock
    if not dec_stock_if_available(med_id, 1):
        log_dispense(med, dni_hash, ok=False, error="NO_STOCK_AFTER_HW")
        rosio.pub_error("NO_STOCK", "Sin stock al confirmar dispensación")
        return False

    # 3) Log OK
    meta = {"bin_id": int(med.get("bin_id", 0))}
    log_dispense(med, dni_hash, ok=True, meta=meta)
    rosio.pub_status({"msg": "Dispensado", "med": med.get("nombre")})
    return True
