"""
Converte una UrdfRobot (da urdf_parser.py) in una lista di specifiche di
giunto pronte per essere passate a JointSpec lato C++.

Il punto delicato: un URDF reale intercala giunti "revolute" (i veri assi
motorizzati) con giunti "fixed" che portano offset/rotazioni non banali
(es. in ur5.urdf, 'base_link-base_link_inertia' e' fixed ma introduce una
rotazione di 180 gradi — vedi commento nel file URDF stesso). Non possiamo
scartare i giunti fissi: li FONDIAMO nel giunto revolute successivo,
accumulando le trasformazioni. Quelli fissi DOPO l'ultimo giunto revolute
(flange, tool0) diventano un "tool_offset" separato — utile perche' e'
letteralmente il punto in cui monteremo la ventosa/pinza.

ATTENZIONE - approssimazione dichiarata: trattiamo l'origine geometrica di
ogni link (quella usata per orientare mesh/giunti nell'URDF) come
coincidente con il suo centro di massa. E' un'approssimazione comune per
una prima versione funzionante, ma non e' esatta (l'URDF puo' specificare
un <inertial><origin> diverso da quello del link). Se in futuro serve
precisione dinamica reale (non solo visiva), va tolta.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

from urdf_parser import UrdfRobot, UrdfJoint


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Converte roll-pitch-yaw (convenzione URDF: R = Rz(yaw)*Ry(pitch)*Rx(roll),
    la stessa di ROS tf 'sxyz') in un quaternione (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quat_multiply(q1: tuple, q2: tuple) -> tuple:
    """q1 * q2, entrambi (x, y, z, w). Applica prima q2, poi q1."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate_vec(q: tuple, v: tuple) -> tuple:
    """Ruota il vettore v col quaternione q (x, y, z, w)."""
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
    """Trasformazione accumulata da giunti fissi non ancora "consumati"
    da un giunto revolute."""
    translation: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = (0.0, 0.0, 0.0, 1.0)  # identita'

    def compose(self, xyz: tuple, rpy: tuple) -> "PendingTransform":
        """Compone questa trasformazione con la successiva (xyz, rpy) di
        un giunto figlio, nell'ordine parent -> figlio."""
        q_step = quat_from_rpy(*rpy)
        new_rotation = quat_multiply(self.rotation, q_step)
        rotated_xyz = quat_rotate_vec(self.rotation, xyz)
        new_translation = tuple(a + b for a, b in zip(self.translation, rotated_xyz))
        return PendingTransform(new_translation, new_rotation)


@dataclass
class JointSpecData:
    """Rappecchia JointSpec del C++, in una forma serializzabile facile
    da passare attraverso i binding pybind11."""
    name: str
    axis: tuple
    rot_parent_to_this: tuple  # quaternione (x, y, z, w)
    pivot_in_parent: tuple
    lower_limit: float
    upper_limit: float
    max_motor_force: float


def build_joint_specs(robot: UrdfRobot) -> tuple[list[JointSpecData], dict]:
    """Ritorna (lista di JointSpecData per i giunti revolute reali,
    tool_offset = trasformazione fissa residua dall'ultimo link mobile
    al vero end-effector/tool0)."""
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

        specs.append(JointSpecData(
            name=j.name,
            axis=j.axis,
            rot_parent_to_this=full.rotation,
            pivot_in_parent=full.translation,
            lower_limit=j.lower,
            upper_limit=j.upper,
            max_motor_force=j.effort,
        ))
        pending = PendingTransform()

    tool_offset = {
        "translation": pending.translation,
        "rotation": pending.rotation,
    }
    return specs, tool_offset


if __name__ == "__main__":
    import sys
    from urdf_parser import parse_urdf

    robot = parse_urdf(sys.argv[1])
    specs, tool_offset = build_joint_specs(robot)

    print(f"{len(specs)} giunti motorizzati trovati:")
    for s in specs:
        print(f"  {s.name}: asse={s.axis} pivot_parent={tuple(round(v, 4) for v in s.pivot_in_parent)} "
              f"rot={tuple(round(v, 4) for v in s.rot_parent_to_this)} "
              f"limiti=[{s.lower_limit:.3f}, {s.upper_limit:.3f}]")
    print(f"\nOffset fino al tool (end-effector reale): {tool_offset}")
