"""
Oracolo di verifica: carica l'URDF con l'importer NATIVO di bullet3 (via
PyBullet) e ESPORTA in JSON i valori che l'importer calcola davvero per
pivot/inerzia di ogni giunto — inclusi quelli FISSI, che PyBullet non
fonde nel giunto successivo come facevamo noi, ma tiene come link separati.

Uso:
    python tools/pybullet_oracle.py assets/ur5/ur5.urdf --json assets/ur5/oracle.json
"""
import sys
import os
import json
import pybullet as p


def main():
    if len(sys.argv) < 2:
        print("Uso: python pybullet_oracle.py <path_urdf> [--json out.json]")
        sys.exit(1)

    urdf_path = sys.argv[1]
    json_out = None
    if "--json" in sys.argv:
        json_out = sys.argv[sys.argv.index("--json") + 1]

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(urdf_path)))
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)

    num_joints = p.getNumJoints(robot_id)
    print(f"{num_joints} giunti (PyBullet NON fonde i fissi come facevamo noi)\n")

    joints_data = []
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode()
        joint_type = info[2]
        joint_axis = info[13]
        parent_frame_pos = info[14]
        parent_frame_orn = info[15]
        parent_index = info[16]

        dyn = p.getDynamicsInfo(robot_id, i)
        mass = dyn[0]
        local_inertia_diag = dyn[2]
        local_inertial_pos = dyn[3]
        local_inertial_orn = dyn[4]

        type_str = {0: "revolute", 1: "prismatic", 4: "fixed"}.get(joint_type, str(joint_type))
        print(f"[{i}] {joint_name} ({type_str})")

        joints_data.append({
            "index": i,
            "name": joint_name,
            "type": type_str,
            "parent_index": parent_index,
            "axis": list(joint_axis),
            "pivot_in_parent": list(parent_frame_pos),
            "rot_parent_to_this": list(parent_frame_orn),
            "mass": mass,
            "link_inertia": list(local_inertia_diag),
            "pivot_in_child": list(local_inertial_pos),
            "child_inertial_orn": list(local_inertial_orn),
        })

    print(f"\njson_out = {json_out}")
    if json_out:
        with open(json_out, "w") as f:
            json.dump({"joints": joints_data}, f, indent=2)
        print(f"Salvato: {json_out}")
    else:
        print("[ATTENZIONE] --json non passato o non riconosciuto, nessun file scritto.")

    p.disconnect()


if __name__ == "__main__":
    main()
