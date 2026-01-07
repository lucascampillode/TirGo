# src/tirgo_ui/session.py
import time
from typing import Optional, Dict

_state: Dict[str, object] = {
    "active": False,
    "op_name": None,
    "started_at": 0.0,
}

def start_session(op_name: str = "generic") -> None:
    _state["active"] = True
    _state["op_name"] = op_name
    _state["started_at"] = time.time()

def end_session() -> None:
    _state["active"] = False
    _state["op_name"] = None
    _state["started_at"] = 0.0

def is_active() -> bool:
    return bool(_state.get("active", False))

def current() -> Optional[Dict[str, object]]:
    if not is_active():
        return None
    return dict(_state)
