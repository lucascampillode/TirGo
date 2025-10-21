import os, sqlite3, hashlib, time
from typing import Optional
from .config import DB_PATH, PEPPER

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS pacientes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    dni_hash TEXT NOT NULL,
    nombre TEXT NOT NULL,
    apellidos TEXT NOT NULL,
    necesita_restringido INTEGER DEFAULT 0
);
CREATE UNIQUE INDEX IF NOT EXISTS idx_pac_dni ON pacientes(dni_hash);
CREATE TABLE IF NOT EXISTS medicamentos (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre TEXT NOT NULL,
    tipo TEXT CHECK(tipo IN ('A','B','C','R')) NOT NULL,
    bin_id INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS recetas (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    paciente_id INTEGER NOT NULL,
    medicamento_id INTEGER NOT NULL,
    activa INTEGER DEFAULT 1,
    caducidad TEXT
);
CREATE INDEX IF NOT EXISTS idx_recetas ON recetas(paciente_id, medicamento_id, activa);
CREATE TABLE IF NOT EXISTS stock (
    medicamento_id INTEGER PRIMARY KEY,
    qty INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts INTEGER NOT NULL,
    evento TEXT NOT NULL,
    dni_hash TEXT,
    med_id INTEGER,
    estado TEXT,
    detalle TEXT
);
"""
DEMO_DATA_SQL = """
INSERT INTO medicamentos (nombre, tipo, bin_id) VALUES
  ('Paracetamol 1g', 'A', 1),
  ('Ibuprofeno 400', 'B', 2),
  ('Vitamina C 1g', 'C', 3),
  ('Antibiótico RX', 'R', 4);
INSERT INTO stock (medicamento_id, qty) VALUES
  (1, 5),(2, 4),(3, 3),(4, 2);
"""

def db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db_if_needed():
    need_demo = not os.path.exists(DB_PATH)
    conn = db()
    conn.executescript(SCHEMA_SQL)
    if need_demo:
        conn.executescript(DEMO_DATA_SQL)
    conn.commit(); conn.close()

def h_dni(dni: str) -> str:
    dni = (dni or '').strip().upper()
    raw = dni + PEPPER
    return hashlib.sha256(raw.encode('utf-8')).hexdigest()

def find_or_create_paciente(nombre: str, apellidos: str, dni: str) -> Optional[int]:
    if not (nombre and apellidos and dni): return None
    dni_hash = h_dni(dni)
    conn = db()
    row = conn.execute("SELECT id FROM pacientes WHERE dni_hash=?", (dni_hash,)).fetchone()
    if row:
        pid = row['id']
    else:
        cur = conn.execute(
            "INSERT INTO pacientes (dni_hash, nombre, apellidos) VALUES (?,?,?)",
            (dni_hash, nombre.strip(), apellidos.strip()))
        pid = cur.lastrowid
        conn.commit()
    conn.close()
    return pid

def lookup_medicamento_by_name(name: str):
    conn = db()
    row = conn.execute("SELECT * FROM medicamentos WHERE lower(nombre)=lower(?)", (name.strip(),)).fetchone()
    conn.close()
    return row

def get_stock(med_id: int) -> int:
    conn = db()
    row = conn.execute("SELECT qty FROM stock WHERE medicamento_id=?", (med_id,)).fetchone()
    conn.close()
    return int(row['qty']) if row else 0

def dec_stock_if_available(conn: sqlite3.Connection, med_id: int) -> bool:
    cur = conn.execute("UPDATE stock SET qty = qty - 1 WHERE medicamento_id=? AND qty > 0", (med_id,))
    return cur.rowcount == 1

def permitted_meds_for_patient(paciente_id: int):
    sql = """
    SELECT m.*, s.qty, p.necesita_restringido,
           EXISTS(SELECT 1 FROM recetas r
                  WHERE r.paciente_id=p.id AND r.medicamento_id=m.id AND r.activa=1) AS tiene_receta
    FROM pacientes p
    CROSS JOIN medicamentos m
    LEFT JOIN stock s ON s.medicamento_id=m.id
    WHERE p.id=? AND IFNULL(s.qty,0)>0
      AND (m.tipo<>'R' OR tiene_receta=1 OR p.necesita_restringido=1)
    ORDER BY m.nombre
    """
    conn = db()
    rows = conn.execute(sql, (paciente_id,)).fetchall()
    conn.close()
    return rows

def log_dispense(conn: sqlite3.Connection, med_id: int, dni_hash: Optional[str], estado: str, detalle: str):
    conn.execute('INSERT INTO logs (ts, evento, dni_hash, med_id, estado, detalle) VALUES (?,?,?,?,?,?)',
                 (int(time.time()), 'dispense', dni_hash, med_id, estado, detalle))