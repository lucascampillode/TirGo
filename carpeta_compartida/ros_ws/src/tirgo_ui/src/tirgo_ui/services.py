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
    Aquí invocarías al hardware real. De momento simulamos.
    En la versión final esto debería coordinarse con la misión / dispensador.
    """
    time.sleep(1.0)


def dispense_physical(med: dict, dni_hash: Optional[str] = None) -> bool:
    """
    Secuencia de dispensado con ejecución física y descuento de stock
    SOLO cuando la operación ha terminado correctamente.

    Política nueva:
    - NO se descuenta stock en el momento del clic.
    - Se intenta primero el dispensado físico.
    - Si el hardware va bien, entonces se descuenta 1 unidad de stock.
    - Si falla el hardware, no se toca el stock.
    - Si al intentar descontar ya no hay stock (carrera rara), se registra como error.

    Parámetros:
    - med: dict con al menos 'id' y opcionalmente 'nombre', 'bin_id'
    - dni_hash: hash del DNI si aplica (restringidos), o None para libres
    """
    med_id = int(med["id"])

    # 0) Comprobación rápida de stock antes de arrancar hardware
    # (evitamos lanzar el dispensador sabiendo que no hay stock)
    stock = get_stock(med_id)
    if stock is not None and stock <= 0:
        rosio.pub_error('NO_STOCK', 'Sin stock')
        log_dispense(med, dni_hash, ok=False, error="NO_STOCK")
        return False

    # 1) Ejecutar el dispensado físico
    try:
        _do_physical_dispense(med)
    except Exception as e:
        # No tocamos stock si falla el hardware
        log_dispense(med, dni_hash, ok=False, error=f"HW_FAIL: {e}")
        rosio.pub_error("HW_FAIL", "Fallo en dispensador")
        return False

    # 2) Si el dispensado físico ha ido bien, ahora SÍ descontamos 1 de stock
    if not dec_stock_if_available(med_id, 1):
        # Situación de carrera: había stock al empezar, pero alguien se lo ha llevado entre medias
        log_dispense(med, dni_hash, ok=False, error="NO_STOCK_AFTER_DISPENSE")
        rosio.pub_error("NO_STOCK", "Sin stock al confirmar dispensación")
        return False

    # 3) Log OK + feedback a la UI
    meta = {"bin_id": int(med.get("bin_id", 0))}
    log_dispense(med, dni_hash, ok=True, meta=meta)
    rosio.pub_status({"msg": "Dispensado", "med": med.get("nombre")})
    return True
