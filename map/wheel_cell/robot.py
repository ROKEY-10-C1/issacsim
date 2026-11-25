"""Doosan URDF FK and damped least-squares tool-pose IK for the USD preview."""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


class Arm:
    def __init__(self, urdf):
        tree = ET.parse(urdf)
        self.model = tree.getroot().get("name").lower()
        self.joints = [tree.find(f"./joint[@name='joint_{i}']") for i in range(1, 7)]
        self.xyz = [np.fromstring(j.find("origin").get("xyz"), sep=" ") for j in self.joints]
        self.rpy = [Rotation.from_euler("xyz", np.fromstring(j.find("origin").get("rpy"), sep=" ")).as_matrix()
                    for j in self.joints]
        self.axes = [np.fromstring(j.find("axis").get("xyz"), sep=" ") for j in self.joints]
        self.lower = np.array([float(j.find("limit").get("lower")) for j in self.joints])
        self.upper = np.array([float(j.find("limit").get("upper")) for j in self.joints])
        self.speed = np.array([float(j.find("limit").get("velocity")) for j in self.joints])
        if self.model == "h2515":
            self.speed = np.minimum(self.speed, np.deg2rad([100,80,100,180,180,180]))

    def fk(self, base, q):
        p, rot = np.array(base), np.eye(3)
        frames = []
        for xyz, rpy, axis, angle in zip(self.xyz, self.rpy, self.axes, q):
            p = p + rot @ xyz
            rot = rot @ rpy
            axis_world = rot @ axis
            rot = rot @ Rotation.from_rotvec(axis * angle).as_matrix()
            frames.append((p, rot, axis_world))
        return frames

    def solve(self, base, target, side, previous=None):
        # Flange +Z points from the flange toward the wheel.
        rotation = np.array([[1, 0, 0], [0, 0, -side], [0, side, 0]])
        seeds = [previous] if previous is not None else []
        rng = np.random.default_rng(41)
        seeds += [np.zeros(6), *rng.uniform(-2.4, 2.4, (24, 6))]
        for seed in seeds:
            q = seed.copy()
            for _ in range(100):
                frames = self.fk(base, q)
                p, r, _ = frames[-1]
                position = target - p
                orientation = Rotation.from_matrix(rotation @ r.T).as_rotvec()
                if np.linalg.norm(position) < .0002 and np.linalg.norm(orientation) < .002:
                    return q
                jacobian = np.array([np.r_[np.cross(axis, p - origin), .3 * axis]
                                     for origin, _, axis in frames]).T
                error = np.r_[position, .3 * orientation]
                step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + .0004 * np.eye(6), error)
                q = np.clip(q + np.clip(step, -.25, .25), self.lower, self.upper)
        raise ValueError(f"{self.model.upper()} target unreachable at {target} from {base}")
