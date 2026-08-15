# Asset CAD

Non versionati nel repo (mesh pesanti). Da scaricare a parte.

## Braccio robotico

Consigliato: **Universal Robots UR5 o UR10**, URDF + mesh ufficiali,
open source, licenza permissiva, usati come standard de facto nella
letteratura RL-robotica (compatibili nativamente con l'importer URDF di
PyBullet/Bullet):

    https://github.com/ros-industrial/universal_robot

Prendi `ur_description/urdf/ur5.urdf.xacro` (va convertito da xacro a URDF
puro — serve `xacro` di ROS, oppure una conversione manuale una tantum) e
la cartella `meshes/` corrispondente.

UR5 (portata ~0.85m) e' probabilmente piu' adatto di UR10 (portata ~1.3m,
piu' pesante da simulare) per una cella di lavoro compatta con nastro
circolare — verificare durante il layout definitivo.

## Pallet

Non serve un CAD "reale" cercato online: un pallet EUR (1200x800mm) e'
geometricamente banale (tavole + traversine), si modella a mano in pochi
minuti in Blender o anche direttamente come box compound in Bullet.

## Dopo aver scaricato

```
assets/
  ur5/
    ur5.urdf
    meshes/*.stl
    collision/          <- generato da tools/mesh_preprocess.py, non a mano
```
