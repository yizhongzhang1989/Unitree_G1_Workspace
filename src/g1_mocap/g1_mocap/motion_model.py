"""Model-based floor validation including collision origins and foot rotation."""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .kinematics import G1Kinematics
from .motion_capture import JOINT_NAMES


class MotionModel:
    def __init__(self, urdf_path):
        robot = ET.parse(urdf_path).getroot()
        children = {joint.find('child').get('link') for joint in robot.findall('joint')}
        roots = {link.get('name') for link in robot.findall('link')} - children
        moving = {joint.get('name') for joint in robot.findall('joint')
                  if joint.get('type') != 'fixed'}
        if roots != {'pelvis'} or moving != set(JOINT_NAMES):
            raise ValueError('Capture model must be pelvis-rooted with exactly the specified 29 joints')
        self.kin = G1Kinematics(urdf_path, JOINT_NAMES)
        self.feet = ('left_ankle_roll_link', 'right_ankle_roll_link')
        self.contacts = []
        for name in self.feet:
            link = robot.find(f"link[@name='{name}']")
            contacts = []
            if link is None:
                raise ValueError(f'Model has no {name}')
            for collision in link.findall('collision'):
                origin = collision.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None
                                    else '0 0 0', sep=' ')
                rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None
                                    else '0 0 0', sep=' ')
                rotation = Rotation.from_euler('xyz', rpy).as_matrix()
                sphere = collision.find('geometry/sphere')
                box = collision.find('geometry/box')
                if sphere is not None:
                    contacts.append((xyz, float(sphere.get('radius'))))
                elif box is not None:
                    size = np.fromstring(box.get('size'), sep=' ')
                    for signs in itertools.product((-0.5, 0.5), repeat=3):
                        contacts.append((xyz + rotation @ (size * signs), 0.0))
                else:
                    raise ValueError(f'{name}: floor check supports sphere/box collisions only')
            if not contacts:
                raise ValueError(f'{name}: foot collision geometry is required')
            self.contacts.append(contacts)

    def foot_heights(self, rows):
        heights = np.empty((len(rows), 2))
        for index, row in enumerate(rows):
            root_rotation = Rotation.from_quat(row[3:7]).as_matrix()
            self.kin.key_body_pos(row[7:], self.feet)
            for side, name in enumerate(self.feet):
                position = row[:3] + root_rotation @ self.kin.frame_pos(name)
                rotation = root_rotation @ self.kin.frame_rot(name)
                heights[index, side] = min(
                    (position + rotation @ center)[2] - radius
                    for center, radius in self.contacts[side])
        return heights
