r"""
Primo test end-to-end: costruisce il braccio UR5 VERO (da URDF + hull
convessi pre-decomposti) nel motore fisico, e verifica che non esploda
nei primi step di simulazione.

Esegui da dentro tools/ (come urdf_to_joint_specs.py):
    cd tools
    python build_and_test_arm.py ..\assets\ur5\ur5.urdf ..\assets\ur5\collision [force_override]

Serve palletizer_core compilato nel PYTHONPATH — vedi istruzioni in chat.
"""
import sys
sys.path.insert(0, "../build/lib/Release")

from urdf_parser import parse_urdf
from urdf_to_joint_specs import build_joint_specs, attach_collision_hulls

import palletizer_core as pc


def main():
    if len(sys.argv) < 3:
        print("Uso: python build_and_test_arm.py <urdf> <cartella_collision> [force_override]")
        sys.exit(1)

    urdf_path, collision_dir = sys.argv[1], sys.argv[2]

    robot = parse_urdf(urdf_path)
    specs, tool_offset = build_joint_specs(robot)
    attach_collision_hulls(specs, collision_dir)

    n_with_hulls = sum(1 for s in specs if s.convex_hulls)
    print(f"{len(specs)} giunti, {n_with_hulls} con collision shape reale "
          f"({len(specs) - n_with_hulls} vuoti — normale se mesh_preprocess non e' ancora girato su tutto)")

    print("\nValori max_motor_force letti dall'URDF (<limit effort=...>):")
    for s in specs:
        print(f"  {s.name}: {s.max_motor_force}")

    force_override = float(sys.argv[3]) if len(sys.argv) > 3 else None
    if force_override is not None:
        print(f"\n[TEST] Sovrascrivo max_motor_force a {force_override} per tutti i giunti.")

    world = pc.PhysicsWorld()
    world.add_ground_plane(0.0)

    arm = pc.ArticulatedArm(world, "ur5_test", (0.0, 0.0, 0.0))

    for s in specs:
        js = pc.JointSpec()
        js.name = s.name
        js.axis = s.axis
        js.rot_parent_to_this = s.rot_parent_to_this
        js.pivot_in_parent = s.pivot_in_parent
        js.pivot_in_child = s.pivot_in_child
        js.lower_limit = s.lower_limit
        js.upper_limit = s.upper_limit
        js.max_motor_force = force_override if force_override is not None else s.max_motor_force
        js.link_mass = s.link_mass
        js.link_inertia = s.link_inertia
        js.convex_hulls = s.convex_hulls or []
        arm.add_link(js)

    arm.finalize_build()
    print(f"\nBraccio costruito: {arm.num_links()} link nel motore.")

    pos0, rot0 = arm.get_end_effector_pose()
    print(f"Posa end-effector a riposo (giunti=0): pos={tuple(round(v, 4) for v in pos0)} "
          f"quat={tuple(round(v, 4) for v in rot0)}")

    print("\nAttivo i motori a target-velocita'=0 su tutti i giunti (mantenimento posa)...")
    for i in range(len(specs)):
        arm.set_joint_target_velocity(i, 0.0)

    joints_before = arm.get_joint_positions()
    print(f"Angoli giunti PRIMA (rad): {[round(v, 4) for v in joints_before]}")

    for _ in range(60):
        world.step_simulation(1.0 / 60.0, 10)

    joints_after = arm.get_joint_positions()
    print(f"Angoli giunti DOPO (rad):  {[round(v, 4) for v in joints_after]}")
    for i, (name, before, after) in enumerate(zip([s.name for s in specs], joints_before, joints_after)):
        moved = abs(after - before)
        flag = " <-- si muove parecchio" if moved > 0.1 else ""
        print(f"  [{i}] {name}: {before:.4f} -> {after:.4f} (delta {moved:.4f}){flag}")

    pos1, rot1 = arm.get_end_effector_pose()
    print(f"\nPosa end-effector dopo 1s: pos={tuple(round(v, 4) for v in pos1)} "
          f"quat={tuple(round(v, 4) for v in rot1)}")

    import math
    drift = sum((a - b) ** 2 for a, b in zip(pos0, pos1)) ** 0.5
    values_ok = all(math.isfinite(v) for v in (*pos1, *rot1))

    if not values_ok:
        print("\n[ERRORE] Valori NaN/infiniti — bug reale (pivot/asse/inerzia).")
    elif drift > 0.05:
        print(f"\n[ATTENZIONE] Drift di {drift:.4f}m nonostante i motori attivi.")
    else:
        print(f"\n[OK] I motori reggono la posa: drift {drift:.5f}m, trascurabile.")


if __name__ == "__main__":
    main()
