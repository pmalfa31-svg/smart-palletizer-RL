r"""
Primo test end-to-end: costruisce il braccio UR5 VERO (da URDF + hull
convessi pre-decomposti) nel motore fisico, e verifica che non esploda
nei primi step di simulazione.

Esegui da dentro tools/ (come urdf_to_joint_specs.py):
    cd tools
    python build_and_test_arm.py ..\assets\ur5\ur5.urdf ..\assets\ur5\collision

Serve palletizer_core compilato nel PYTHONPATH — vedi istruzioni in chat.
"""
import sys
sys.path.insert(0, "../build/lib/Debug")

from urdf_parser import parse_urdf
from urdf_to_joint_specs import build_joint_specs, attach_collision_hulls

import palletizer_core as pc


def main():
    if len(sys.argv) < 3:
        print("Uso: python build_and_test_arm.py <urdf> <cartella_collision>")
        sys.exit(1)

    urdf_path, collision_dir = sys.argv[1], sys.argv[2]

    robot = parse_urdf(urdf_path)
    specs, tool_offset = build_joint_specs(robot)
    attach_collision_hulls(specs, collision_dir)

    n_with_hulls = sum(1 for s in specs if s.convex_hulls)
    print(f"{len(specs)} giunti, {n_with_hulls} con collision shape reale "
          f"({len(specs) - n_with_hulls} vuoti — normale se mesh_preprocess non e' ancora girato su tutto)")

    world = pc.PhysicsWorld()
    world.add_ground_plane(0.0)

    arm = pc.ArticulatedArm(world, "ur5_test", (0.0, 0.0, 0.0))

    for s in specs:
        js = pc.JointSpec()
        js.name = s.name
        js.axis = s.axis
        js.rot_parent_to_this = s.rot_parent_to_this
        js.pivot_in_parent = s.pivot_in_parent
        js.pivot_in_child = (0.0, 0.0, 0.0)
        js.lower_limit = s.lower_limit
        js.upper_limit = s.upper_limit
        js.max_motor_force = s.max_motor_force
        js.link_mass = s.link_mass  # ora reale, dal <inertial> dell'URDF
        js.link_inertia = s.link_inertia  # idem
        js.convex_hulls = s.convex_hulls or []
        arm.add_link(js)

    arm.finalize_build()
    print(f"\nBraccio costruito: {arm.num_links()} link nel motore.")

    pos0, rot0 = arm.get_end_effector_pose()
    print(f"Posa end-effector a riposo (giunti=0): pos={tuple(round(v, 4) for v in pos0)} "
          f"quat={tuple(round(v, 4) for v in rot0)}")

    print("\nFaccio girare 60 step di simulazione (1 secondo) senza comandi motore. "
          "ATTENZIONE: senza un motore che regge la posa, il braccio si affloscia\n"
          "sotto gravita' — e' fisica corretta per un braccio spento, non un bug. "
          "Qui controlliamo solo che la simulazione non esploda (NaN/valori assurdi).")
    for _ in range(60):
        world.step_simulation(1.0 / 60.0, 10)

    pos1, rot1 = arm.get_end_effector_pose()
    print(f"Posa end-effector dopo 1s: pos={tuple(round(v, 4) for v in pos1)} "
          f"quat={tuple(round(v, 4) for v in rot1)}")

    import math
    values_ok = all(math.isfinite(v) for v in (*pos1, *rot1))
    magnitude_ok = all(abs(v) < 10.0 for v in pos1)

    if not values_ok:
        print("\n[ERRORE] Valori NaN/infiniti nella posa — questo SI' e' un bug reale "
              "(pivot/asse/inerzia rotti, non solo gravita').")
    elif not magnitude_ok:
        print(f"\n[ATTENZIONE] Il braccio si e' spostato molto (>10m) — troppo anche per "
              f"un collasso sotto gravita' senza motori. Controllare masse/inerzie placeholder.")
    else:
        print(f"\n[OK] Nessuna esplosione numerica. Il braccio si e' afflosciato sotto "
              f"gravita' (atteso, senza motori attivi) ma resta in valori fisicamente plausibili.")


if __name__ == "__main__":
    main()
