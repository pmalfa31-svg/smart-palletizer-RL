"""
Decomposizione convessa OFFLINE delle mesh CAD (bracci URDF, pallet, ecc.).

Perche' offline e in Python: V-HACD e' lento (secondi-minuti per mesh) e va
girato UNA VOLTA per asset, non ad ogni avvio del motore. Il C++ carica solo
il risultato gia' pronto (liste di hull convessi), mai la mesh grezza.

Richiede: pip install trimesh vhacdx  (o "pip install trimesh[easy]" a
seconda della piattaforma — verificare quale wheel VHACD gira su Windows
prima di fidarsi ciecamente di questo comando).

Uso su un singolo file:
    python tools/mesh_preprocess.py assets/ur5/meshes/forearm.stl \
        --out assets/ur5/collision/forearm_hulls.json

Uso batch su tutto un URDF (risolve automaticamente i path package://):
    python tools/mesh_preprocess.py --urdf assets/ur5/ur5.urdf \
        --meshes-root assets/ur5/meshes --out-dir assets/ur5/collision
"""
from __future__ import annotations
import argparse
import json
import os
import trimesh

from urdf_parser import parse_urdf


def resolve_mesh_uri(uri: str, local_meshes_root: str) -> str:
    """Converte un URI URDF tipo 'package://ur_description/meshes/ur5/
    collision/base.stl' nel path locale sotto assets/ur5/meshes/, coerente
    con come abbiamo scaricato le mesh (solo la sottocartella
    ur_description/meshes/<robot>/, non l'intero pacchetto ROS).

    package://<pacchetto>/meshes/<nome_robot>/<resto> -> <local_meshes_root>/<resto>
    Se l'URI non e' un package://, la trattiamo gia' come path locale."""
    if not uri.startswith("package://"):
        return uri
    parts = uri[len("package://"):].split("/")
    rest = "/".join(parts[3:])
    return os.path.join(local_meshes_root, rest)


def decompose(mesh_path: str) -> list[list[list[float]]]:
    mesh = trimesh.load(mesh_path, force="mesh")
    try:
        pieces = mesh.convex_decomposition()
        hulls = [p.convex_hull for p in pieces]
    except Exception as exc:  # noqa: BLE001 - vogliamo il fallback, non il crash
        print(f"[WARN] convex_decomposition fallita ({exc}); fallback a singolo hull")
        hulls = [mesh.convex_hull]
    return [hull.vertices.tolist() for hull in hulls]


def process_urdf(urdf_path: str, meshes_root: str, out_dir: str) -> None:
    """Decompone la mesh di COLLISIONE di ogni link dell'URDF (non quella
    visuale — per la fisica ci serve solo quella) e salva un JSON per link."""
    os.makedirs(out_dir, exist_ok=True)
    robot = parse_urdf(urdf_path)

    processed, skipped = 0, 0
    for name, link in robot.links.items():
        if not link.collision_mesh:
            skipped += 1
            continue
        local_path = resolve_mesh_uri(link.collision_mesh, meshes_root)
        if not os.path.exists(local_path):
            print(f"[WARN] mesh non trovata per '{name}': {local_path} (salto)")
            skipped += 1
            continue

        hulls = decompose(local_path)
        out_path = os.path.join(out_dir, f"{name}.json")
        with open(out_path, "w") as f:
            json.dump({"hulls": hulls}, f)

        n_verts = sum(len(h) for h in hulls)
        print(f"[OK] {name}: {len(hulls)} hull, {n_verts} vertici -> {out_path}")
        processed += 1

    print(f"\nFatto: {processed} link processati, {skipped} saltati (senza mesh o non trovata).")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path", nargs="?", help="Path a una singola mesh (modalita' file singolo)")
    parser.add_argument("--out", help="Output per la modalita' file singolo")
    parser.add_argument("--urdf", help="Path a un URDF (modalita' batch)")
    parser.add_argument("--meshes-root", help="Cartella locale delle mesh (modalita' batch)")
    parser.add_argument("--out-dir", help="Cartella di output (modalita' batch)")
    args = parser.parse_args()

    if args.urdf:
        if not (args.meshes_root and args.out_dir):
            parser.error("--urdf richiede anche --meshes-root e --out-dir")
        process_urdf(args.urdf, args.meshes_root, args.out_dir)
        return

    if not (args.mesh_path and args.out):
        parser.error("modalita' file singolo: servono mesh_path e --out")
    hulls = decompose(args.mesh_path)
    with open(args.out, "w") as f:
        json.dump({"hulls": hulls}, f)
    n_verts = sum(len(h) for h in hulls)
    print(f"[OK] {len(hulls)} hull convessi, {n_verts} vertici totali -> {args.out}")


if __name__ == "__main__":
    main()
