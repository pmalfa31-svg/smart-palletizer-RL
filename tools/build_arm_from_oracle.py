r"""
Costruisce l'ArticulatedArm usando DIRETTAMENTE i valori esportati da
pybullet_oracle.py — bypassa completamente urdf_to_joint_specs.py, che
per ora resta "sospetto" (probabilmente manca la trasformazione di frame
che l'importer nativo applica). Se questo test regge la posa su TUTTI i
giunti, conferma che il problema era li' e non nel resto del motore.

Uso:
    cd tools
    python pybullet_oracle.py ..\assets\ur5\ur5.urdf --json ..\assets\ur5\oracle.json
    python build_arm_from_oracle.py ..\assets\ur5\oracle.json ..\assets\ur5\collision
"""
import sys
import json
sys.path.insert(0, "../build/lib/Release")

import palletizer_core as pc


def main():
    if len(sys.argv) < 3:
        print("Uso: python build_arm_from_oracle.py <oracle.json> <cartella_collision>")
        sys.exit(1)

    oracle_path, collision_dir = sys.argv[1], sys.argv[2]
    with open(oracle_path) as f:
        data = json.load(f)

    world = pc.PhysicsWorld()
    world.add_ground_plane(0.0)
    arm = pc.ArticulatedArm(world, "ur5_oracle", (0.0, 0.0, 0.0))

    import os

    for j in data["joints"]:
        js = pc.JointSpec()
        js.name = j["name"]
        js.joint_type = j["type"] if j["type"] in ("revolute", "fixed") else "fixed"
        js.axis = tuple(j["axis"])
        js.rot_parent_to_this = tuple(j["rot_parent_to_this"])
        js.pivot_in_parent = tuple(j["pivot_in_parent"])
        js.pivot_in_child = tuple(j["pivot_in_child"])
        js.link_mass = j["mass"] if j["mass"] > 0 else 1.0
        js.link_inertia = tuple(j["link_inertia"]) if any(j["link_inertia"]) else (0.01, 0.01, 0.01)

        if js.joint_type == "revolute":
            js.lower_limit = 1.0
            js.upper_limit = -1.0
            real_effort = {
                "shoulder_pan_joint": 150.0, "shoulder_lift_joint": 150.0, "elbow_joint": 150.0,
                "wrist_1_joint": 28.0, "wrist_2_joint": 28.0, "wrist_3_joint": 28.0,
            }
            js.max_motor_force = real_effort.get(j["name"], 150.0)

        link_guess = j["name"].replace("_joint", "_link")
        hull_path = os.path.join(collision_dir, f"{link_guess}.json")
        if os.path.exists(hull_path):
            with open(hull_path) as f:
                js.convex_hulls = json.load(f)["hulls"]
        else:
            js.convex_hulls = []

        arm.add_link(js)

    arm.finalize_build()
    print(f"Braccio costruito: {arm.num_links()} link (inclusi quelli fissi).")

    n = arm.num_links()
    for i in range(n):
        arm.set_joint_target_position(i, 0.0, 0.3)  # no-op sui fissi, ok sui revolute

    joints_before = arm.get_joint_positions()
    print(f"Angoli PRIMA: {[round(v, 4) for v in joints_before]}")

    for _ in range(60):
        world.step_simulation(1.0 / 60.0, 10)

    joints_after = arm.get_joint_positions()
    print(f"Angoli DOPO:  {[round(v, 4) for v in joints_after]}")
    for i, (name, before, after) in enumerate(zip([j["name"] for j in data["joints"]], joints_before, joints_after)):
        moved = abs(after - before)
        flag = " <-- si muove parecchio" if moved > 0.1 else ""
        print(f"  [{i}] {name}: {before:.4f} -> {after:.4f} (delta {moved:.4f}){flag}")

    pos, rot = arm.get_end_effector_pose()
    print(f"\nPosa end-effector dopo 1s: pos={tuple(round(v,4) for v in pos)}")


if __name__ == "__main__":
    main()
