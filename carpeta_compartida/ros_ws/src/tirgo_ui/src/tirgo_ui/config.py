import os

HOTWORD = os.environ.get('TIRGO_HOTWORD', 'hola')
WINDOW_SECS = float(os.environ.get('TIRGO_HOTWORD_WINDOW', '10.0'))
STT_TOPIC_DEFAULT = os.environ.get('TIRGO_STT_TOPIC', '/stt/text')

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR  = os.path.dirname(os.path.dirname(BASE_DIR))
TEMPLATE_DIR = os.path.join(PKG_DIR, 'templates')
STATIC_DIR   = os.path.join(PKG_DIR, 'static')

DB_PATH = os.environ.get('TIRGO_DB', os.path.expanduser('~/tirgo.db'))
PEPPER  = os.environ.get('TIRGO_PEPPER', 'cambia-esto')
