# test/conftest.py
# Indicamos a pytest que ignore el test de integración ROS, que se ejecuta con rostest.


import pytest

# Guardamos info de los tests
_collected_items = []
_test_outcomes = {}
collect_ignore = ["test_mission_flow.py"]


def pytest_collection_modifyitems(session, config, items):
    """
    Hook que se ejecuta tras la colección de tests.
    Guardamos la lista de items para poder resumirlos luego.
    """
    global _collected_items
    _collected_items = list(items)


def pytest_runtest_logreport(report):
    """
    Hook que se ejecuta después de cada fase de un test.
    Nos quedamos solo con el resultado de la fase 'call' (la ejecución del test).
    """
    if report.when == "call":
        _test_outcomes[report.nodeid] = report.outcome


def pytest_sessionfinish(session, exitstatus):
    """
    Al final de la sesión, imprimimos un resumen por test:
    - icono: ✅ / ❌ / ⚠️
    - nombre del test
    - docstring del test (si tiene)
    """
    print("\n" + "=" * 60)
    print("Resumen detallado de tests:")
    print("=" * 60)

    for item in _collected_items:
        nodeid = item.nodeid
        name = item.name
        outcome = _test_outcomes.get(nodeid, "skipped")

        if outcome == "passed":
            icon = "✅"
        elif outcome == "failed":
            icon = "❌"
        else:
            icon = "⚠️"

        doc = (item.obj.__doc__ or "").strip()
        if doc:
            line = f"{icon} {name}: {doc}"
        else:
            line = f"{icon} {name}"

        print(line)

    print("=" * 60)
