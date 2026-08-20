"""
Converte una UrdfRobot (da urdf_parser.py) in una lista di specifiche di
giunto pronte per essere passate a JointSpec lato C++.

Il punto delicato: un URDF reale intercala giunti "revolute" (i veri assi
motorizzati) con giunti "fixed" che portano offset/rotazioni non banali
(es. in ur5.urdf, 'base_link-base_link_inertia' e' fixed ma introduce una
rotazione di 180 gradi). Non possiamo scartare i giunti fissi: li FONDIAMO
nel giunto revolute successivo, accumulando le trasformazioni. Quelli fissi
DOPO l'ultimo giunto revolute (flange, tool0) diventano un "tool_offset"
separato — e' il punto in cui monteremo la ventosa/pinza.

ATTENZIONE - approssimazione ancora presente: il tensore d'inerzia
(ixx/iyy/izz) viene usato cosi' com'e' dall'URDF, ASSUMENDO che sia gia'
espresso nel frame del link. In realta' l'URDF puo' specificare una
rotazione anche per il frame inerziale (<inertial><origin rpy=...>,
es. upper_arm_link ha rpy="0 1.5707963267948966 0") — quel caso NON e'
gestito: ruotare correttamente un tensore diagonale produce in generale
una matrice piena, che l'interfaccia di Bullet usata qui (btVector3
diagonale) non rappresenta. La posizione del baricentro (com_offset) e'
invece gestita correttamente qui sotto — era il problema piu' grosso e
ora e' risolto.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

from urdf_parser import UrdfRobot, UrdfJoint


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quat_multiply(q1: tuple, q2: tuple) -> tuple:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate_vec(q: tuple, v: tuple) -> tuple:
    x, y, z, w = q
    vx, vy, vz = v
    uvx = y * vz - z * vy
    uvy = z * vx - x * vz
    uvz = x * vy - y * vx
    uuvx = y * uvz - z * uvy
    uuvy = z * uvx - x * uvz
    uuvz = x * uvy - y * uvx
    return (
        vx + 2.0 * (w * uvx + uuvx),
        vy + 2.0 * (w * uvy + uuvy),
        vz + 2.0 * (w * uvz + uuvz),
    )


@dataclass
class PendingTransform:
    translation: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = (0.0, 0.0, 0.0, 1.0)

    def compose(self, xyz: tuple, rpy: tuple) -> "PendingTransform":
        q_step = quat_from_rpy(*rpy)
        new_rotation = quat_multiply(self.rotation, q_step)
        rotated_xyz = quat_rotate_vec(self.rotation, xyz)
        new_translation = tuple(a + b for a, b in zip(self.translation, rotated_xyz))
        return PendingTransform(new_translation, new_rotation)


@dataclass
class JointSpecData:
    name: str
    link_name: str
    axis: tuple
    rot_parent_to_this: tuple
    pivot_in_parent: tuple
    lower_limit: float
    upper_limit: float
    max_motor_force: float
    pivot_in_child: tuple = (0.0, 0.0, 0.0)
    link_mass: float = 1.0
    link_inertia: tuple = (0.05, 0.05, 0.05)
    convex_hulls: list = None


def build_joint_specs(robot: UrdfRobot) -> tuple[list[JointSpecData], dict]:
    joints = robot.joints_in_chain_order()
    specs: list[JointSpecData] = []
    pending = PendingTransform()

    for j in joints:
        if j.type == "fixed":
            pending = pending.compose(j.xyz, j.rpy)
            continue

        if j.type not in ("revolute", "continuous"):
            raise ValueError(f"Tipo di giunto non supportato: {j.type} ({j.name})")

        full = pending.compose(j.xyz, j.rpy)

        link = robot.links.get(j.child)
        link_mass = link.mass if (link and link.mass > 0.0) else 1.0
        link_inertia = link.inertia_diag if (link and any(link.inertia_diag)) else (0.05, 0.05, 0.05)

        # FIX: pivotInParent/pivotInChild devono essere misurati dal vero
        # BARICENTRO dei link, non dalla loro origine geometrica.
        parent_link = robot.links.get(j.parent)
        parent_com = parent_link.com_offset if parent_link else (0.0, 0.0, 0.0)
        child_com = link.com_offset if link else (0.0, 0.0, 0.0)

        pivot_in_parent = tuple(p - c for p, c in zip(full.translation, parent_com))
        pivot_in_child = child_com

        specs.append(JointSpecData(
            name=j.name,
            link_name=j.child,
            axis=j.axis,
            rot_parent_to_this=full.rotation,
            pivot_in_parent=pivot_in_parent,
            pivot_in_child=pivot_in_child,
            lower_limit=j.lower,
            upper_limit=j.upper,
            max_motor_force=j.effort,
            link_mass=link_mass,
            link_inertia=link_inertia,
        ))
        pending = PendingTransform()

    tool_offset = {
        "translation": pending.translation,
        "rotation": pending.rotation,
    }
    return specs, tool_offset


def attach_collision_hulls(specs: list[JointSpecData], collision_dir: str) -> None:
    import json
    import os

    for spec in specs:
        path = os.path.join(collision_dir, f"{spec.link_name}.json")
        if not os.path.exists(path):
            print(f"[WARN] nessun hull trovato per '{spec.link_name}' ({path}); collision shape vuota")
            spec.convex_hulls = []
            continue
        with open(path) as f:
            data = json.load(f)
        spec.convex_hulls = data["hulls"]


if __name__ == "__main__":
    import sys
    from urdf_parser import parse_urdf

    robot = parse_urdf(sys.argv[1])
    specs, tool_offset = build_joint_specs(robot)

    print(f"{len(specs)} giunti motorizzati trovati:")
    for s in specs:
        print(f"  {s.name}: pivot_parent={tuple(round(v, 4) for v in s.pivot_in_parent)} "
              f"pivot_child={tuple(round(v, 4) for v in s.pivot_in_child)} "
              f"limiti=[{s.lower_limit:.3f}, {s.upper_limit:.3f}]")
    print(f"\nOffset fino al tool (end-effector reale): {tool_offset}")
