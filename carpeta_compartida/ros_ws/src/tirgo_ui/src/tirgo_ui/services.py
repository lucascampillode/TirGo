from typing import Optional, Dict, Any
import time
from .storage_mongo import get_stock, dec_stock_if_available, log_dispense
# Si quieres revertir en caso de fallo hardware:
# from .storage_mongo import inc_stock

def _do_physical_dispense(med: Dict[str, Any]) -> bool:
    # TODO: integra tu driver real (GPIO/serie). Simulamos retardo:
    time.sleep(1.0)
    return True

def dispense_physical(med: Dict[str, Any], dni_hash: Optional[str]) -> bool:
    med_id = int(med["id"])
    if get_stock(med_id) <= 0:
        return False

    if not dec_stock_if_available(med_id, units=1):
        return False

    hw_ok = _do_physical_dispense(med)
    # if not hw_ok: inc_stock(med_id, 1)  # opcional
    try:
        log_dispense(med, dni_hash)
    except Exception:
        pass
    return hw_ok
