#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TirGoPharma – Web Server v2 con hotword
- Oculta opciones hasta que el usuario diga "hola" (por STT en /stt/text, configurable).
- Si no hay ROS master, la web funciona y puedes simular el hotword con /simulate_hola (quítalo si no quieres).
Estructura:
tirgo_ui/
├─ scripts/web_server_v2.py
├─ templates/index_v2.html
└─ static/style_v2.css
"""
import os, sqlite3, threading, time, hashlib, socket, re, unicodedata
from urllib.parse import urlparse
from typing import Optional, Dict, Any, List
from flask import Flask, request, render_template, jsonify, redirect, url_for

# ---------------- Config hotword ----------------
HOTWORD = "hola"
WINDOW_SECS = 10.0
STT_TOPIC_DEFAULT = "/stt/text"  # cámbialo si tu STT publica en otro topic

# ---------------- Rutas de templates/estáticos ----------------
BASE_DIR     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # scripts -> tirgo_ui
template_dir = os.path.join(BASE_DIR, 'templates')
static_dir   = os.path.join(BASE_DIR, 'static')
APP = Flask(__name__, template_folder=template_dir, static_folder=static_dir)

DB_PATH = os.environ.get('TIRGO_DB', os.path.expanduser('~/tirgo.db'))

# ---------------- Estado de hotword ----------------
_state = {"hotword_ts": 0.0}

def _normalize(text: str) -> str:
    t = unicodedata.normalize("NFD", text).encode("ascii", "ignore").decode("ascii")
    return t.lower().strip()

def hotword_active() -> bool:
    ts = _state.get("hotword_ts", 0.0)
    return ts > 0.0 and (time.time() - ts) <= WINDOW_SECS

def hotword_remaining() -> float:
    ts = _state.get("hotword_ts", 0.0)
    if ts <= 0: return 0.0
    rem = WINDOW_SECS - (time.time() - ts)
    return round(rem if rem > 0 else 0.0, 2)

# ---------------- ROS (best-effort) ----------------
try:
    import rospy
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False
    class _DummyPub:  # no-op
        def publish(self, *_args, **_kwargs): pass
    class _DummyRospy:
        def init_node(self, *a, **k): pass
        def loginfo(self, *a, **k): print(*a)
        def signal_shutdown(self, *a, **k): pass
        def get_param(self, *a, **k): return STT_TOPIC_DEFAULT
        def Subscriber(self, *a, **k): return None
        Publisher = lambda *a, **k: _DummyPub()
    rospy = _DummyRospy()     # type: ignore
    String = str              # type: ignore

def ros_master_up() -> bool:
    try:
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        u = urlparse(uri)
        host, port = u.hostname or "localhost", u.port or 11311
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except Exception:
        return False

_pub_state = _pub_status = _pub_error = None

def stt_cb(msg: String):
    try:
        txt = msg.data if hasattr(msg, "data") else str(msg)
    except Exception:
        txt = str(msg)
    txtn = _normalize(txt)
    if re.search(r"\b" + re.escape(HOTWORD) + r"\b", txtn):
        _state["hotword_ts"] = time.time()
        try:
            rospy.loginfo(f"[HOTWORD] detectada en STT: {txt}")
        except Exception:
            pass

def init_ros():
    global _pub_state, _pub_status, _pub_error
    if ROS_AVAILABLE and ros_master_up():
        try:
            rospy.init_node('tirgo_web_server_v2', anonymous=True, disable_signals=True)
        except Exception:
            pass
        # pubs
        _pub_state  = rospy.Publisher('tirgo/ui/state',  String, queue_size=10)
        _pub_status = rospy.Publisher('tirgo/ui/status', String, queue_size=10)
        _pub_error  = rospy.Publisher('tirgo/ui/error',  String, queue_size=10)
        # sub STT
        stt_topic = rospy.get_param("~stt_topic", STT_TOPIC_DEFAULT)
        rospy.Subscriber(stt_topic, String, stt_cb)
        try:
            rospy.loginfo(f"[STT] suscrito a {stt_topic}")
        except Exception:
            pass
    else:
        class DummyPub:
            def publish(self, *_a, **_k): pass
        _pub_state = _pub_status = _pub_error = DummyPub()

def pub_state(s: str):
    try:
        _pub_state.publish(String(data=s) if ROS_AVAILABLE and hasattr(String, "data") else s)
        rospy.loginfo(f"[STATE] {s}")
    except Exception as e:
        print('[STATE publish error]', e)

def pub_status(d: Dict[str, Any]):
    import json
    payload = json.dumps(d, ensure_ascii=False)
    try:
        _pub_status.publish(String(data=payload) if ROS_AVAILABLE and hasattr(String, "data") else payload)
        rospy.loginfo(f"[STATUS] {payload}")
    except Exception as e:
        print('[STATUS publish error]', e)

def pub_error(code: str, msg: str):
    try:
        _pub_error.publish(String(data=code) if ROS_AVAILABLE and hasattr(String, "data") else code)
        rospy.loginfo(f"[ERROR] {code}: {msg}")
    except Exception:
        pass

# ---------------- SQLite ----------------
SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS pacientes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    dni_hash TEXT NOT NULL,
    nombre TEXT NOT NULL,
    apellidos TEXT NOT NULL,
    necesita_restringido INTEGER DEFAULT 0
);
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

def h_dni(dni: str) -> str:
    dni = (dni or '').strip().upper()
    return hashlib.sha256(dni.encode('utf-8')).hexdigest()

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

# ---------------- Lógica de negocio ----------------
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

def lookup_medicamento_by_name(name: str) -> Optional[sqlite3.Row]:
    conn = db()
    row = conn.execute("SELECT * FROM medicamentos WHERE lower(nombre)=lower(?)", (name.strip(),)).fetchone()
    conn.close()
    return row

def get_stock(med_id: int) -> int:
    conn = db()
    row = conn.execute("SELECT qty FROM stock WHERE medicamento_id=?", (med_id,)).fetchone()
    conn.close()
    return int(row['qty']) if row else 0

def permitted_meds_for_patient(paciente_id: int) -> List[sqlite3.Row]:
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

# ---------------- Simulación NAV + DISP ----------------
_nav_lock = threading.Lock()
def simulate_navigate_and_dispense(med: sqlite3.Row, dni_hash: Optional[str] = None) -> None:
    with _nav_lock:
        pub_state('NAVIGATING_TO_STORAGE')
        for i in range(5):
            pub_status({'progress': (i+1)*20, 'eta': f"{(5-i)*2}s", 'battery': 95-i, 'msg': 'Yendo al almacén'})
            time.sleep(1)
        pub_state('ARRIVED'); time.sleep(0.6)
        conn = db()
        try:
            row = conn.execute('SELECT qty FROM stock WHERE medicamento_id=?', (med['id'],)).fetchone()
            qty = int(row['qty']) if row else 0
            if qty <= 0:
                pub_error('NO_STOCK', 'Sin stock al dispensar'); pub_state('ERROR'); return
            conn.execute('UPDATE stock SET qty=? WHERE medicamento_id=?', (qty-1, med['id']))
            conn.execute('INSERT INTO logs (ts, evento, dni_hash, med_id, estado, detalle) VALUES (?,?,?,?,?,?)',
                         (int(time.time()), 'dispense', dni_hash, med['id'], 'ok', med['nombre']))
            conn.commit()
        finally:
            conn.close()
        pub_state('HANDOVER'); time.sleep(0.5); pub_state('DONE')

# ---------------- Rutas Flask ----------------
@APP.after_request
def no_cache(resp):
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    return resp

@APP.get('/healthz')
def healthz():
    return jsonify({'ok': True, 'db': os.path.exists(DB_PATH)})

@APP.get('/state')
def state():
    return jsonify({
        "hotword": hotword_active(),
        "remaining": hotword_remaining(),
        "window_secs": WINDOW_SECS
    })

# (Opcional) simular el “hola” si no tienes STT aún. Borra esta ruta en producción.
@APP.post('/simulate_hola')
def simulate_hola():
    _state["hotword_ts"] = time.time()
    return jsonify({"ok": True, "hotword": True})

@APP.route('/')
def index():
    pub_state('MODE_SELECTED')
    # Si NO hay hotword activo, mostramos la pantalla de “esperando hola”
    if not hotword_active():
        return render_template('index_v2.html', state='IDLE', view='await_hotword',
                               window_secs=WINDOW_SECS, db_path=DB_PATH)
    # Si el hotword está activo → mostramos los modos
    return render_template('index_v2.html', state='MODE_SELECTED', db_path=DB_PATH)

# ---------------- Consultar ----------------
@APP.route('/consultar', methods=['GET','POST'])
def consultar():
    if not hotword_active():
        return redirect(url_for('index'))
    if request.method == 'GET':
        pub_state('IDENTIFYING')
        return render_template('index_v2.html', state='IDENTIFYING', view='consultar', db_path=DB_PATH)
    nombre = request.form.get('nombre','').strip()
    apellidos = request.form.get('apellidos','').strip()
    dni = request.form.get('dni','').strip()
    pid = find_or_create_paciente(nombre, apellidos, dni)
    if not pid:
        pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('index_v2.html', state='IDENTIFYING', view='consultar',
                               msg='Datos no válidos', error=True, db_path=DB_PATH)
    allowed = permitted_meds_for_patient(pid)
    pub_state('VALIDATED')
    return render_template('index_v2.html', state='VALIDATED', view='consultar_result',
                           paciente={'nombre':nombre,'apellidos':apellidos,'dni_hash':h_dni(dni)},
                           allowed=allowed, db_path=DB_PATH)

@APP.route('/recoger/<int:med_id>')
def recoger(med_id: int):
    if not hotword_active():
        return redirect(url_for('index'))
    dni_hash = request.args.get('dni_hash')
    conn = db()
    med = conn.execute('SELECT * FROM medicamentos WHERE id=?', (med_id,)).fetchone()
    conn.close()
    if not med:
        pub_error('NOT_FOUND', 'Medicamento no existe')
        return render_template('index_v2.html', state='ERROR', msg='Medicamento no encontrado', error=True, db_path=DB_PATH)
    if get_stock(med_id) <= 0:
        pub_error('NO_STOCK', 'Sin stock')
        return render_template('index_v2.html', state='ERROR', msg='Sin stock', error=True, db_path=DB_PATH)
    threading.Thread(target=simulate_navigate_and_dispense, args=(med, dni_hash), daemon=True).start()
    return render_template('index_v2.html', state='NAVIGATING_TO_STORAGE', msg='Recogiendo tu producto', db_path=DB_PATH)

# ---------------- Leer ----------------
@APP.route('/leer', methods=['GET','POST'])
def leer():
    if not hotword_active():
        return redirect(url_for('index'))
    if request.method == 'GET':
        pub_state('MODE_SELECTED')
        return render_template('index_v2.html', state='MODE_SELECTED', view='leer', db_path=DB_PATH)
    name = request.form.get('med','').strip()
    med = lookup_medicamento_by_name(name)
    if not med:
        pub_error('NOT_FOUND', 'Medicamento no existe')
        return render_template('index_v2.html', state='MODE_SELECTED', view='leer',
                               msg='Medicamento no encontrado', error=True, db_path=DB_PATH)
    if med['tipo'] != 'R':
        if get_stock(med['id']) <= 0:
            pub_error('NO_STOCK', 'Sin stock')
            return render_template('index_v2.html', state='MODE_SELECTED', view='leer',
                                   msg='Sin stock', error=True, db_path=DB_PATH)
        threading.Thread(target=simulate_navigate_and_dispense, args=(med, None), daemon=True).start()
        return render_template('index_v2.html', state='NAVIGATING_TO_STORAGE', msg='Recogiendo tu producto', db_path=DB_PATH)
    # si R → pedir identificación
    pub_state('IDENTIFYING')
    return render_template('index_v2.html', state='IDENTIFYING', view='leer_ident', med=med, db_path=DB_PATH)

@APP.route('/leer/ident', methods=['POST'])
def leer_ident():
    if not hotword_active():
        return redirect(url_for('index'))
    med_id = int(request.form.get('med_id'))
    nombre = request.form.get('nombre','').strip()
    apellidos = request.form.get('apellidos','').strip()
    dni = request.form.get('dni','').strip()

    conn = db(); med = conn.execute('SELECT * FROM medicamentos WHERE id=?', (med_id,)).fetchone(); conn.close()
    if not med:
        pub_error('NOT_FOUND', 'Medicamento no existe')
        return render_template('index_v2.html', state='ERROR', msg='Medicamento no encontrado', error=True, db_path=DB_PATH)

    pid = find_or_create_paciente(nombre, apellidos, dni)
    if not pid:
        pub_error('ID_FAIL', 'Datos incompletos')
        return render_template('index_v2.html', state='IDENTIFYING', view='leer_ident', med=med,
                               msg='Datos no válidos', error=True, db_path=DB_PATH)

    conn = db()
    necesita = conn.execute('SELECT necesita_restringido FROM pacientes WHERE id=?',(pid,)).fetchone()
    necesita = int(necesita['necesita_restringido']) if necesita else 0
    r = conn.execute('SELECT 1 FROM recetas WHERE paciente_id=? AND medicamento_id=? AND activa=1',(pid, med_id)).fetchone()
    tiene_receta = 1 if r else 0
    conn.close()

    if med['tipo'] == 'R' and not (tiene_receta or necesita):
        pub_error('RESTRICTED', 'No autorizado para este R')
        return render_template('index_v2.html', state='ERROR',
                               msg='Este medicamento requiere receta válida. Visita a tu médico.',
                               error=True, db_path=DB_PATH)

    if get_stock(med_id) <= 0:
        pub_error('NO_STOCK', 'Sin stock')
        return render_template('index_v2.html', state='ERROR', msg='Sin stock', error=True, db_path=DB_PATH)

    threading.Thread(target=simulate_navigate_and_dispense, args=(med, h_dni(dni)), daemon=True).start()
    return render_template('index_v2.html', state='NAVIGATING_TO_STORAGE', msg='Recogiendo tu producto', db_path=DB_PATH)

# ---------------- Diagnóstico ----------------
@APP.route('/diagnostico', methods=['GET','POST'])
def diagnostico():
    if not hotword_active():
        return redirect(url_for('index'))
    if request.method == 'GET':
        pub_state('MODE_SELECTED')
        return render_template('index_v2.html', state='MODE_SELECTED', view='diagnostico', db_path=DB_PATH)
    sx = request.form.get('sx','dolor'); aine = request.form.get('aine','no')
    if sx == 'infeccion':
        pub_error('RESTRICTED', 'Diag sugiere R')
        return render_template('index_v2.html', state='ERROR',
                               msg='La mejor opción puede requerir antibiótico (tipo R). Visita a tu médico para receta.',
                               error=True, db_path=DB_PATH)
    recomendado = 'Paracetamol 1g' if (sx=='dolor' or aine=='si') else 'Vitamina C 1g'
    return render_template('index_v2.html', state='VALIDATED', view='diag_result',
                           recomendado=recomendado, db_path=DB_PATH)

# ---------------- main ----------------
if __name__ == '__main__':
    init_db_if_needed()
    init_ros()
    pub_state('IDLE')
    try:
        APP.run(host='0.0.0.0', port=int(os.environ.get('PORT','9005')),
                debug=False, use_reloader=False)
    except KeyboardInterrupt:
        try:
            if ROS_AVAILABLE:
                rospy.signal_shutdown('ctrl-c')
        except Exception:
            pass
        print("\nAdiós 👋")
