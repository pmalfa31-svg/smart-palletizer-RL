# Smart Palletizer RL

Sistema di pallettizzazione robotica simulata: nastro trasportatore circolare,
due bracci robotici sincronizzati (uno per lato), pacchi in 3 formati, un
planner di bin-packing 3D che decide dove posare ogni pacco e una policy RL
che controlla i bracci per eseguire quel piano.

## Architettura

```
engine/           Motore fisico C++ (Bullet, btMultiBody/Featherstone), zero
                   dipendenze da Python. Compila come libreria statica.
  core/            Setup del mondo fisico (PhysicsWorld)
  robot/           Braccio articolato (ArticulatedArm, da specifiche di link
                   gia' parsate — non fa parsing URDF)
  conveyor/        Nastro circolare a slot cinematici (CircularConveyor)
  packaging/       Pacchi in 3 taglie (Parcel)
  planner/         Euristica di bin-packing 3D (PalletPlanner) — vedi sotto

bindings/          L'unico punto che sa di Python (pybind11)

env/               Wrapper Gymnasium (l'ambiente RL vero e proprio, compone
                   i pezzi del motore in uno step()/reset() in stile Gym)

tools/             Preprocessing OFFLINE degli asset (Python):
  urdf_parser.py     legge URDF con xml.etree, niente dipendenze fragili
  mesh_preprocess.py decomposizione convessa (V-HACD via trimesh), una volta
                     per mesh, mai a runtime

configs/           YAML — nessuna costante magica nel codice

tests/             pytest sui pezzi gia' completi (planner). Test C++ del
                   motore (Catch2/GoogleTest) da aggiungere man mano.

assets/            URDF + mesh dei bracci (non versionati nel repo — vedi
                   assets/README.md per dove scaricarli)
```

## Decisioni di design (e perche')

**btMultiBody (Featherstone) invece di catene di `btHingeConstraint`.**
Un braccio a 6 assi e' una catena cinematica articolata: incatenare rigid
body con hinge separati e' instabile e improponibile oltre 2 DOF.
`btMultiBody` e' l'algoritmo pensato apposta per catene cinematiche
articolate, ed e' quello che usa PyBullet sotto per i robot importati da URDF.

**Parsing URDF e decomposizione convessa: offline, in Python, mai a runtime.**
Il C++ non fa mai I/O di asset complessi: riceve numeri gia' pronti
(`JointSpec`) e liste di hull convessi pre-calcolati. Questo tiene il motore
semplice e il build system libero dalla dipendenza fragile dell'URDF
importer nativo di bullet3.

**Mesh concave MAI su corpi dinamici.** Bullet le gestisce bene solo per
corpi statici. Ogni asset dinamico (link del braccio, pallet) passa da
V-HACD prima di diventare collision shape — vedi `tools/mesh_preprocess.py`.

**Planner classico separato dall'RL.** Il bin-packing 3D e' un problema di
ottimizzazione combinatoria (NP-hard) con letteratura consolidata — farlo
imparare a una policy RL da zero sarebbe sia impraticabile nei tempi sia
indifendibile ("perche' la rete ha scelto qui?"). `PalletPlanner` decide
DOVE (euristica heightmap, imparentata con Deepest-Bottom-Left packing);
l'RL impara SOLO come muovere il braccio per arrivarci.

**Nastro circolare a slot cinematici, non attrito-based.** Simulare attrito
belt-pacco su una curva e' instabile a 60Hz discreti (la direzione tangente
cambia continuamente, il pacco deriva). Slot cinematici equispaziati sono
piu' robusti e sufficienti per l'obiettivo (RL che afferra, non fisica del
nastro in se').

## Stato attuale — cosa e' reale, cosa e' scheletro

| Pezzo | Stato |
|---|---|
| `PhysicsWorld` | Completo |
| `Parcel` | Completo |
| `CircularConveyor` | Completo (slot cinematici) |
| `PalletPlanner` | Completo, con test |
| `ArticulatedArm` | Scheletro funzionale, **non ancora compilato/validato** contro l'API reale di btMultiBody — da verificare al primo build |
| Bindings Parcel/ArticulatedArm | Non ancora esposti (serve prima finalizzare il formato dati URDF -> JointSpec) |
| `env/` (Gymnasium wrapper) | Non ancora scritto |
| Render (Panda3D debug / Blender finale) | Non ancora iniziato |
| URDF bracci (UR5/UR10) | Da scaricare, vedi `assets/README.md` |

## Prossimi passi

1. Prima build reale: validare che `ArticulatedArm.cpp` compili contro
   l'API `btMultiBody` effettiva (i nomi esatti dei metodi vanno confermati).
2. Scaricare URDF+mesh UR5 (o UR10), far girare `tools/urdf_parser.py` e
   `tools/mesh_preprocess.py` su un link di prova.
3. Completare i bindings di `Parcel`/`ArticulatedArm` e scrivere la classe
   che compone tutto in un `step()`/`reset()` stile Gym.
4. Scrivere `env/palletizer_env.py`.
