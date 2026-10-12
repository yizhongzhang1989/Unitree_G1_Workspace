"""Controlled 29-axis model inputs for offline motion tests."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from g1_mocap.motion_capture import JOINT_NAMES


@pytest.fixture(scope='session')
def controlled_urdf(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Keep real link geometry and fix non-control joints at their zero pose."""
    source = (Path(__file__).resolve().parents[2] / 'unitree_g1_description' /
              'model/g1_description/g1_29dof_mode_15.urdf')
    tree = ET.parse(source)
    for joint in tree.getroot().findall('joint'):
        if joint.attrib['name'] not in JOINT_NAMES:
            joint.set('type', 'fixed')
    path = tmp_path_factory.mktemp('controlled-model') / source.name
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return path
