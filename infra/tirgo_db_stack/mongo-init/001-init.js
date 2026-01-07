const appDb = db.getSiblingDB('tirgo');

appDb.createUser({
  user: "tirgo_user",
  pwd:  process.env.TIRGO_DB_PASSWORD || "tirgo_pass_cambia",
  roles: [{ role: "readWrite", db: "tirgo" }]
});

appDb.pacientes.createIndex({ dni_hash: 1 }, { unique: true });
appDb.medicamentos.createIndex({ nombre: 1 }, { unique: true });
appDb.recetas.createIndex({ paciente_id: 1, activa: 1 });
appDb.recetas.createIndex({ medicamento_id: 1, activa: 1 });

appDb.medicamentos.updateOne(
  { nombre: "Amoxicilina 500mg" },
  { $setOnInsert: { nombre: "Amoxicilina 500mg", requiere_receta: true } },
  { upsert: true }
);
appDb.medicamentos.updateOne(
  { nombre: "Ibuprofeno 400mg" },
  { $setOnInsert: { nombre: "Ibuprofeno 400mg", requiere_receta: false } },
  { upsert: true }
);
