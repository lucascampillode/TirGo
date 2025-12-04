# tests/test_rosio_helpers_unit.py

from tirgo_ui import rosio


def test_get_and_clear_voice_nav_returns_string_twice():
    """
    No forzamos implementación interna, solo que el método existe
    y devuelve strings (aunque estén vacíos).
    """
    v1 = rosio.get_and_clear_voice_nav()
    v2 = rosio.get_and_clear_voice_nav()

    assert isinstance(v1, str)
    assert isinstance(v2, str)


def test_conv_state_returns_string_or_dict():
    """
    En tu implementación actual conv_state() devuelve 'idle' (str),
    así que documentamos que puede ser str o dict.
    """
    state = rosio.conv_state()
    assert isinstance(state, (str, dict))
