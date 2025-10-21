import threading, time
from typing import Optional
from . import rosio
from .storage import db, get_stock, dec_stock_if_available, log_dispense

_nav_lock = threading.Lock()

def dispense_physical(med, dni_hash: Optional[str] = None) -> None:
    with _nav_lock:
        rosio.pub_state('DISPENSING')
        conn = db()
        try:
            if get_stock(med['id']) <= 0:
                rosio.pub_error('NO_STOCK', 'Sin stock al dispensar'); rosio.pub_state('ERROR'); return
            if not dec_stock_if_available(conn, med['id']):
                rosio.pub_error('NO_STOCK', 'Sin stock (race)'); rosio.pub_state('ERROR'); return
            log_dispense(conn, med['id'], dni_hash, 'ok', med['nombre'])
            conn.commit()
        finally:
            conn.close()
        rosio.pub_state('READY')
        time.sleep(0.2)
