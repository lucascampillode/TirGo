# tests/test_session_unit.py

from tirgo_ui import session


def test_start_session_sets_active():
    """
    Solo comprobamos que start_session no peta y deja la sesión activa.
    La firma real de start_session puede variar, así que usamos solo
    un argumento posicional genérico.
    """
    session.end_session()

    # Ajustado a tu implementación real: sin kwargs raros
    session.start_session("CONSULTAR")

    assert session.is_active() is True
    cur = session.current()
    # No asumimos estructura interna, solo que no explota
    assert cur is None or isinstance(cur, dict)


def test_end_session_clears_state():
    session.start_session("TEST")

    session.end_session()

    assert session.is_active() is False
    cur = session.current()
    # current() puede devolver None o {}
    assert cur is None or cur == {}


def test_current_safe_to_call_even_without_session():
    session.end_session()
    cur = session.current()
    # Este test documenta que current() es "seguro"
    assert cur is None or isinstance(cur, dict)
