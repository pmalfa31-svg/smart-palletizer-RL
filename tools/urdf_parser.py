"""
Parser URDF minimale.

Non usiamo librerie come urdfpy/yourdfpy: dipendono da pacchi (pycollada,
networkx pinnati a versioni vecchie) che sono una fonte di grattacapi in un
progetto che deve restare buildabile per mesi. Ci serve un sottoinsieme
piccolo e stabile di URDF (link, giunti revolute, assi, limiti, path mesh):
lo leggiamo a mano con xml.etree, che e' nella standard library e non si
rompe mai.

Uso tipico:
    robot = parse_urdf("assets/ur5/ur5.urdf")
    for joint in robot.joints_in_chain_order():
        ...  # passa i dati a JointSpec lato C++ via i binding
"""
from __future__ import annotations
from dataclasses import dataclass, field
import xml.etree.ElementTree as ET


@dataclass
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 100.0


@dataclass
class UrdfLink:
    name: str
    mass: float = 0.0
    inertia_diag: tuple[float, float, float] = (0.0, 0.0, 0.0)
    com_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    visual_mesh: str | None = None
    collision_mesh: str | None = None


@dataclass
class UrdfRobot:
    name: str
    links: dict[str, UrdfLink] = field(default_factory=dict)
    joints: list[UrdfJoint] = field(default_factory=list)

    def joints_in_chain_order(self) -> list[UrdfJoint]:
        from collections import defaultdict
        children_joints: dict[str, list[UrdfJoint]] = defaultdict(list)
        for j in self.joints:
            children_joints[j.parent].append(j)

        children = {j.child for j in self.joints}
        roots = [l for l in self.links if l not in children]
        if not roots:
            return self.joints

        ordered: list[UrdfJoint] = []
        queue = list(roots)
        while queue:
            link = queue.pop(0)
            for j in children_joints.get(link, []):
                ordered.append(j)
                queue.append(j.child)
        return ordered


def _parse_floats(s: str | None, default: tuple) -> tuple:
    if not s:
        return default
    return tuple(float(x) for x in s.strip().split())


def parse_urdf(path: str) -> UrdfRobot:
    tree = ET.parse(path)
    root = tree.getroot()
    robot = UrdfRobot(name=root.get("name", "robot"))

    for link_el in root.findall("link"):
        name = link_el.get("name")
        link = UrdfLink(name=name)

        inertial = link_el.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                link.mass = float(mass_el.get("value", "0"))
            inertia_el = inertial.find("inertia")
            if inertia_el is not None:
                link.inertia_diag = (
                    float(inertia_el.get("ixx", "0")),
                    float(inertia_el.get("iyy", "0")),
                    float(inertia_el.get("izz", "0")),
                )
            origin_el = inertial.find("origin")
            if origin_el is not None:
                link.com_offset = _parse_floats(origin_el.get("xyz"), (0.0, 0.0, 0.0))

        visual = link_el.find("visual/geometry/mesh")
        if visual is not None:
            link.visual_mesh = visual.get("filename")
        collision = link_el.find("collision/geometry/mesh")
        if collision is not None:
            link.collision_mesh = collision.get("filename")

        robot.links[name] = link

    for joint_el in root.findall("joint"):
        j = UrdfJoint(
            name=joint_el.get("name"),
            type=joint_el.get("type"),
            parent=joint_el.find("parent").get("link"),
            child=joint_el.find("child").get("link"),
        )
        origin = joint_el.find("origin")
        if origin is not None:
            j.xyz = _parse_floats(origin.get("xyz"), (0.0, 0.0, 0.0))
            j.rpy = _parse_floats(origin.get("rpy"), (0.0, 0.0, 0.0))
        axis = joint_el.find("axis")
        if axis is not None:
            j.axis = _parse_floats(axis.get("xyz"), (0.0, 0.0, 1.0))
        limit = joint_el.find("limit")
        if limit is not None:
            j.lower = float(limit.get("lower", "0"))
            j.upper = float(limit.get("upper", "0"))
            j.effort = float(limit.get("effort", "100"))
        robot.joints.append(j)

    return robot


if __name__ == "__main__":
    import sys
    r = parse_urdf(sys.argv[1])
    print(f"Robot: {r.name} | {len(r.links)} link, {len(r.joints)} giunti")
    for j in r.joints_in_chain_order():
        print(f"  {j.name}: {j.parent} -> {j.child} (asse {j.axis}, tipo {j.type})")
