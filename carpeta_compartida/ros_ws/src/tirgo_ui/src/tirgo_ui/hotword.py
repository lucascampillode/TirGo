import time, unicodedata, re
from typing import Dict
from .config import HOTWORD, WINDOW_SECS

_state: Dict[str, float] = {"hotword_ts": 0.0}

def _normalize(text: str) -> str:
    t = unicodedata.normalize("NFD", text).encode("ascii", "ignore").decode("ascii")
    return t.lower().strip()

def contains_hotword(text: str) -> bool:
    return bool(re.search(r"\b" + re.escape(HOTWORD) + r"\b", _normalize(text)))

def bump_now() -> None:
    _state["hotword_ts"] = time.time()

def is_active() -> bool:
    ts = _state.get("hotword_ts", 0.0)
    return ts > 0.0 and (time.time() - ts) <= WINDOW_SECS

def remaining() -> float:
    ts = _state.get("hotword_ts", 0.0)
    if ts <= 0: return 0.0
    rem = WINDOW_SECS - (time.time() - ts)
    return round(rem if rem > 0 else 0.0, 2)
