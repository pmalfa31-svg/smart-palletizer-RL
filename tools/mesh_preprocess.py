"""
Decomposizione convessa OFFLINE delle mesh CAD (bracci URDF, pallet, ecc.).

Perche' offline e in Python: V-HACD e' lento (secondi-minuti per mesh) e va
girato UNA VOLTA per asset, non ad ogni avvio del motore. Il C++ carica solo
il risultato gia' pronto (liste di hull convessi), mai la mesh grezza.

Richiede: pip install trimesh vhacdx  (o "pip install trimesh[easy]" a
seconda della piattaforma — verificare quale wheel VHACD gira su Windows
prima di fidarsi ciecamente di questo comando).

Uso:
    python tools/mesh_preprocess.py assets/ur5/meshes/forearm.stl \
        --out assets/ur5/collision/forearm_hulls.json
"""
from __future__ import annotations
import argparse
import json
import trimesh


def decompose(mesh_path: str) -> list[list[list[float]]]:
    mesh = trimesh.load(mesh_path, force="mesh")

    # trimesh espone VHACD tramite mesh.convex_decomposition() se il
    # backend e' installato. Se fallisce, meglio un fallback esplicito
    # (singolo convex hull) che un crash silenzioso — un hull unico e'
    # spesso comunque "abbastanza buono" per link di un braccio robotico
    # (di solito gia' quasi convessi), molto meno per un pallet con vani.
    try:
        pieces = mesh.convex_decomposition()
        hulls = [p.convex_hull for p in pieces]
    except Exception as exc:  # noqa: BLE001 - vogliamo il fallback, non il crash
        print(f"[WARN] convex_decomposition fallita ({exc}); fallback a singolo hull")
        hulls = [mesh.convex_hull]

    return [hull.vertices.tolist() for hull in hulls]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    hulls = decompose(args.mesh_path)
    with open(args.out, "w") as f:
        json.dump({"hulls": hulls}, f)

    n_verts = sum(len(h) for h in hulls)
    print(f"[OK] {len(hulls)} hull convessi, {n_verts} vertici totali -> {args.out}")


if __name__ == "__main__":
    main()
