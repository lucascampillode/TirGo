# tests/test_storage_mongo_unit.py

import datetime


class DummyStore:
    """
    Dummy minimalista para testear la lógica que usamos
    en la UI con pacientes, meds y recetas.
    """
    def __init__(self):
        self.meds = {
            "1": {"_id": "1", "nombre": "Paracetamol", "stock": 10, "tipo": "N"},
            "2": {"_id": "2", "nombre": "Ansiolítico", "stock": 0, "tipo": "R"},
        }
        self.pacientes = {
            "12345678Z": {"dni": "12345678Z", "nombre": "Paciente Normal"},
        }
        self.recetas = [
            {
                "dni": "12345678Z",
                "med_id": "2",
                "activa": True,
                "fecha_inicio": datetime.date.today(),
            },
        ]

    # ------- MÉTODOS TIPO storage_mongo -------

    def meds_disponibles(self):
        return list(self.meds.values())

    def lookup_medicamento_by_id(self, med_id):
        return self.meds.get(str(med_id))

    def get_stock(self, med_id):
        med = self.meds.get(str(med_id))
        return med["stock"] if med else 0

    def create_paciente_if_allowed(self, dni, nombre):
        # En este dummy siempre permitimos crear si no existe
        if dni not in self.pacientes:
            self.pacientes[dni] = {"dni": dni, "nombre": nombre}
        return self.pacientes[dni]

    def paciente_necesita_restringido(self, paciente, med):
        # Simple: si med es tipo R -> necesita receta
        return med.get("tipo") == "R"

    def tiene_receta_activa(self, paciente, med):
        dni = paciente["dni"]
        mid = med["_id"]
        for r in self.recetas:
            if r["dni"] == dni and r["med_id"] == mid and r.get("activa", False):
                return True
        return False


def test_meds_disponibles_returns_list():
    store = DummyStore()
    meds = store.meds_disponibles()
    assert isinstance(meds, list)
    assert len(meds) >= 2
    assert {"_id": "1", "nombre": "Paracetamol", "stock": 10, "tipo": "N"} in meds


def test_lookup_medicamento_by_id_found_and_not_found():
    store = DummyStore()
    med1 = store.lookup_medicamento_by_id("1")
    assert med1 is not None
    assert med1["nombre"] == "Paracetamol"

    med_unknown = store.lookup_medicamento_by_id("999")
    assert med_unknown is None


def test_get_stock_handles_existing_and_missing():
    store = DummyStore()
    assert store.get_stock("1") == 10
    assert store.get_stock("2") == 0
    assert store.get_stock("999") == 0


def test_create_paciente_if_allowed_creates_new():
    store = DummyStore()
    assert "99999999R" not in store.pacientes

    pac = store.create_paciente_if_allowed("99999999R", "Paciente Nuevo")
    assert pac["dni"] == "99999999R"
    assert pac["nombre"] == "Paciente Nuevo"
    assert "99999999R" in store.pacientes


def test_paciente_necesita_restringido_and_tiene_receta_activa():
    store = DummyStore()
    paciente = store.pacientes["12345678Z"]
    med_R = store.meds["2"]

    assert store.paciente_necesita_restringido(paciente, med_R) is True
    assert store.tiene_receta_activa(paciente, med_R) is True

    med_N = store.meds["1"]
    assert store.paciente_necesita_restringido(paciente, med_N) is False
    assert store.tiene_receta_activa(paciente, med_N) is False
