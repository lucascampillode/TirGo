# src/tirgo_ui/mongo_client.py
import os
from pymongo import MongoClient

MONGO_URI = os.environ.get("MONGO_URI", "mongodb://127.0.0.1:27017/tirgo")

_client = MongoClient(MONGO_URI)
_db = _client.get_database()  # usa la DB del URI (tirgo por defecto)

def db():
    """Compatibilidad: acceso directo a la DB."""
    return _db

# 👇 Wrappers para que tu app.py pueda importar get_client / get_db
def get_client():
    return _client

def get_db():
    return _db
