"""Forward kinematics straight from the URDF (no MuJoCo involved).

Used in three places:
  * tools/urdf_chain_to_csv.py  - writes the chain table that octave/fk_check.m reads
  * tests/test_fk_consistency.py - CI check that URDF and MJCF agree
  * later milestones (IK baseline) as an independent kinematic model

URDF conventions: joint origin = translation xyz then fixed rotation
R = Rz(yaw) @ Ry(pitch) @ Rx(roll); a revolute joint then rotates by q about `axis`
expressed in the joint frame.
"""
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from sim.paths import EE_URDF_LINK, ROBOT_URDF, URDF_BASE_LINK


@dataclass
class ChainJoint:
    name: str
    type: str            # "revolute" or "fixed"
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray


def _vec(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, float)


def load_chain(urdf_path=ROBOT_URDF, base=URDF_BASE_LINK, tip=EE_URDF_LINK):
    """Ordered list of joints from `base` link to `tip` link."""
    root = ET.parse(urdf_path).getroot()
    by_child = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        axis = j.find("axis")
        by_child[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            ChainJoint(
                name=j.get("name"),
                type=j.get("type"),
                xyz=_vec(origin.get("xyz") if origin is not None else None, [0, 0, 0]),
                rpy=_vec(origin.get("rpy") if origin is not None else None, [0, 0, 0]),
                axis=_vec(axis.get("xyz") if axis is not None else None, [1, 0, 0]),
            ),
        )
    chain, link = [], tip
    while link != base:
        if link not in by_child:
            raise ValueError(f"link '{link}' is not connected to '{base}'")
        link, joint = by_child[link]
        chain.append(joint)
    chain.reverse()
    for jt in chain:
        if jt.type not in ("revolute", "fixed"):
            raise ValueError(f"unsupported joint type {jt.type} ({jt.name})")
    return chain


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_to_matrix(axis, angle):
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def fk(chain, q):
    """4x4 pose of the tip link in the base frame. `q` lists revolute angles in chain order."""
    T = np.eye(4)
    qi = iter(q)
    for jt in chain:
        step = np.eye(4)
        step[:3, :3] = rpy_to_matrix(jt.rpy)
        step[:3, 3] = jt.xyz
        T = T @ step
        if jt.type == "revolute":
            rot = np.eye(4)
            rot[:3, :3] = axis_angle_to_matrix(jt.axis, next(qi))
            T = T @ rot
    return T
