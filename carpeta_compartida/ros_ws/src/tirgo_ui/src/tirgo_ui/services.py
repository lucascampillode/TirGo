import time
from typing import Optional

from .storage_mongo import (
    get_stock,
    dec_stock_if_available,
    inc_stock,
    log_dispense,
)
from . import rosio

def _do_physical_dispense(med: dict) -> None:
    """
    Aquí invocarías al hardware real. De momento simulamos.
    """
    time.sleep(1.0)

def dispense_physical(med: dict, dni_hash: Optional[str] = None) -> bool:
    """
    Secuencia de dispensado con reserva de stock, ejecución física, log y rollback si falla.
    - med: dict con al menos 'id' y opcionalmente 'nombre', 'bin_id'
    - dni_hash: hash del DNI si aplica (restringidos), o None para libres
    """
    med_id = int(med["id"])

    # 1) Reservar stock de forma atómica
    if not dec_stock_if_available(med_id, 1):
        rosio.pub_error('NO_STOCK', 'Sin stock')
        log_dispense(med, dni_hash, ok=False, error="NO_STOCK")
        return False

    try:
        # 2) Ejecutar el dispensado físico (puede lanzar excepción)
        _do_physical_dispense(med)

        # 3) Log OK
        meta = {"bin_id": int(med.get("bin_id", 0))}
        log_dispense(med, dni_hash, ok=True, meta=meta)

        # 4) Publicar estado
        rosio.pub_status({"msg": "Dispensado", "med": med.get("nombre")})
        return True

    except Exception as e:
        # Rollback de stock si falló el hardware
        inc_stock(med_id, 1)
        log_dispense(med, dni_hash, ok=False, error=f"HW_FAIL: {e}")
        rosio.pub_error("HW_FAIL", "Fallo en dispensador")
        return False
