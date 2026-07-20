from pathlib import Path
from types import SimpleNamespace

import pytest

from unitree_g1_description.joint_state_mapping import G1_JOINT_NAMES
from unitree_g1_description.mit_command import (
    LOW_CMD_MOTOR_COUNT,
    crc32_core,
    load_g1_mit_gains,
    load_position_limits,
    low_cmd_crc,
)


PACKAGE_DIR = Path(__file__).parents[1]


def _motor_command(index):
    return SimpleNamespace(
        mode=index % 2,
        q=index * 0.01,
        dq=-index * 0.02,
        tau=index * 0.03,
        kp=index * 0.04,
        kd=index * 0.05,
        reserve=index,
    )


def test_loads_gains_in_physical_motor_order():
    stiffness, damping = load_g1_mit_gains(
        PACKAGE_DIR / "config" / "default_29dof_param.yaml")

    assert len(stiffness) == len(damping) == len(G1_JOINT_NAMES)
    assert stiffness[:3] == pytest.approx((40.2, 99.1, 40.2))
    assert damping[-3:] == pytest.approx((0.9, 1.1, 1.1))


def test_loads_limits_for_body_and_grippers():
    names = (*G1_JOINT_NAMES, "left_eccentric_joint",
             "right_eccentric_joint")
    limits = load_position_limits(PACKAGE_DIR / "model" / "final.urdf", names)

    assert tuple(limits) == names
    assert limits["left_eccentric_joint"] == pytest.approx(
        (0.0, 2.76377472169236))
    assert limits["right_eccentric_joint"] == pytest.approx(
        (0.0, 2.76377472169236))


def test_crc_matches_unitree_reference_vector():
    message = SimpleNamespace(
        mode_pr=0,
        mode_machine=5,
        motor_cmd=[
            _motor_command(index) for index in range(LOW_CMD_MOTOR_COUNT)
        ],
        reserve=[1, 2, 3, 4],
    )

    assert low_cmd_crc(message) == 0xE494347F # type: ignore


def test_crc32_core_masks_each_operation_to_uint32():
    assert crc32_core((0x00000000,)) == 0xC704DD7B