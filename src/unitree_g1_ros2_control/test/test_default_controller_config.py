from pathlib import Path
import importlib.util
import math
from xml.etree import ElementTree

import yaml
from launch import LaunchContext
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).parents[1]


def _load_control_launch():
    path = PACKAGE_ROOT / "launch" / "control.launch.py"
    spec = importlib.util.spec_from_file_location("unitree_control_launch", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load launch module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides):
    """把 generate_launch_description 声明的那些参数备齐。

    use_camera_calibration 默认关掉：开着的话用例结果会取决于这台机器
    装没装 camera_calibration、标没标过。
    """
    context = LaunchContext()
    context.launch_configurations.update({
        "topology": "dual",
        "controller_manager": "/controller_manager",
        "joint_states_topic": "/joint_states",
        "robot_description_topic": "/robot_description",
        "use_sim_time": "false",
        "use_camera_calibration": "false",
        **module._HARDWARE_ARGUMENTS,
        **overrides,
    })
    return context


def test_default_controller_claims_g1_body_and_both_grippers():
    forward_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "forward_position_controller.yaml").read_text(
            encoding="utf-8"))
    trajectory_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "joint_trajectory_controller.yaml").read_text(
            encoding="utf-8"))
    gain_document = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "default_31dof_param.yaml").read_text(
            encoding="utf-8"))
    joints = gain_document["/**"]["ros__parameters"]["joints"]
    gain_config = gain_document["/g1_gain_table"]["ros__parameters"]

    forward_parameters = forward_config[
        "/forward_position_controller"]["ros__parameters"]
    trajectory_parameters = trajectory_config[
        "/joint_trajectory_controller"]["ros__parameters"]
    assert "joints" not in forward_parameters
    assert "joints" not in trajectory_parameters
    assert joints[29:] == [
        "left_eccentric_joint",
        "right_eccentric_joint",
    ]
    assert len(joints) == 31
    assert len(gain_config["stiffness"]) == len(joints)
    assert len(gain_config["damping"]) == len(joints)
    assert all(math.isfinite(value) and value >= 0.0
               for value in gain_config["stiffness"])
    assert all(math.isfinite(value) and value >= 0.0
               for value in gain_config["damping"])
    # The gravity feed-forward is a parameter of this controller now, not a
    # second controller relaying into it.
    assert set(forward_parameters) == {
        "gravity_table",
        "gravity_filter_cutoff_hz",
        "offset_ramp_s",
        "compensation_scale",
        "friction_scale",
        "friction_table",
        "friction_error_epsilon",
        "friction_velocity_epsilon",
        "target_velocity_cutoff_hz",
        "adaptive_stiffness_scale",
        "adaptive_stiffness_b",
        "adaptive_stiffness_power",
    }
    assert forward_parameters["gravity_table"] == \
        "package://arm_gravity_compensation/config/gravity_table.yaml"
    assert forward_parameters["compensation_scale"] == 1.0
    # 摩擦补偿的总开关，兼作保守度旋钮。
    assert forward_parameters["friction_scale"] >= 0.0
    # 系数从标定导出的独立文件读，正常留空 = 用那一份。分成两个文件是因为重力表
    # 还被力传感器补偿和负载估计读，它们用不到摩擦。
    assert forward_parameters["friction_table"] == \
        "package://arm_gravity_compensation/config/friction_table.yaml"
    # 空的 YAML 列表没有类型，rclcpp 声明时会 abort 整个 ros2_control_node，
    # 所以这两个覆盖项只能从命令行给，不能写进文件。
    assert "friction_load_ratio" not in forward_parameters
    assert "friction_offset_nm" not in forward_parameters
    # 误差项是主导：只靠目标速度恰恰在精细动作时补不够。两个 eps 都允许为 0
    # （= 关掉该项），但不能同时为 0，否则整个补偿恒为 0。
    assert forward_parameters["friction_error_epsilon"] > 0.0
    assert (forward_parameters["friction_error_epsilon"] > 0.0
            or forward_parameters["friction_velocity_epsilon"] > 0.0)
    assert forward_parameters["target_velocity_cutoff_hz"] > 0.0
    # b 是除数、power 是指数，两者不得为 0；scale 是总开关兼保守度旋钮。
    assert forward_parameters["adaptive_stiffness_scale"] >= 0.0
    assert forward_parameters["adaptive_stiffness_b"] > 0.0
    assert forward_parameters["adaptive_stiffness_power"] > 0.0
    assert trajectory_parameters["command_interfaces"] == ["position"]
    assert trajectory_parameters["state_interfaces"] == ["position", "velocity"]
    # Humble 的 JTC 没有“不发布状态”这个概念（校验下限 0.1 Hz），也没有理由
    # 偏离默认值，所以干脆不覆盖。
    assert "state_publish_rate" not in trajectory_parameters
    assert trajectory_parameters["allow_nonzero_velocity_at_trajectory_end"] is False
    assert trajectory_parameters["allow_partial_joints_goal"] is True
    # 轨迹点也是绝对位置，所以它拿到与 FPC 完全相同的一套重力前馈参数。
    assert trajectory_parameters["gravity_table"] == \
        forward_parameters["gravity_table"]
    assert trajectory_parameters["compensation_scale"] == 1.0
    assert trajectory_parameters["open_loop_control"] is False
    constraints = trajectory_parameters["constraints"]
    assert constraints["goal_time"] == 2.0
    assert constraints["stopped_velocity_tolerance"] == 0.05
    assert set(constraints) == {"goal_time", "stopped_velocity_tolerance", *joints}
    assert all(constraints[joint]["goal"] == 0.05 for joint in joints)


def test_controller_manager_registers_mutually_exclusive_fpc_and_jtc():
    manager_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "controllers.yaml").read_text(
            encoding="utf-8"))["controller_manager"]["ros__parameters"]

    assert manager_config["forward_position_controller"]["type"] == \
        "unitree_g1_forward_command_controller/ForwardCommandController"
    assert manager_config["joint_trajectory_controller"]["type"] == \
        "unitree_g1_joint_trajectory_controller/JointTrajectoryController"
    # 重力补偿已并入两个运动控制器，不再是独立 controller。
    assert "arm_gravity_compensation" not in manager_config


def test_plugin_names_stay_recognisable_to_the_test_dashboard():
    # dashboard_node.classify_controller() 靠这两个后缀/子串归类；改名会让页面
    # 把它们当成 "other"，Engage 按钮和关节面板都会消失。
    plugins = ElementTree.parse(PACKAGE_ROOT / "controller_plugins.xml").getroot()
    names = {element.get("name").lower() for element in plugins.findall("class")}

    assert any(name.endswith("jointtrajectorycontroller") for name in names)
    assert any("forward_command_controller" in name for name in names)


def test_arm_stiffness_default_is_owned_by_hardware_plugin():
    module = _load_control_launch()

    assert module._HARDWARE_ARGUMENTS["arm_stiffness_scale"] == ""
    description = module._robot_description(_context(module), PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert "arm_stiffness_scale" not in parameters
    header = (PACKAGE_ROOT / "include" / "unitree_g1_ros2_control" /
              "g1_topic_system.hpp").read_text(encoding="utf-8")
    assert "double arm_stiffness_scale_{1.0};" in header


def test_arm_stiffness_explicit_override_reaches_hardware_plugin():
    module = _load_control_launch()
    context = _context(module, arm_stiffness_scale="2.5")

    description = module._robot_description(context, PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert parameters["arm_stiffness_scale"] == "2.5"


def test_gripper_gains_only_come_from_gain_file():
    module = _load_control_launch()
    assert "gripper_kp" not in module._HARDWARE_ARGUMENTS
    assert "gripper_kd" not in module._HARDWARE_ARGUMENTS

    description = module._robot_description(_context(module), PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert "gripper_kp" not in parameters
    assert "gripper_kd" not in parameters


def test_control_launch_loads_both_motion_controllers_inactive():
    module = _load_control_launch()
    nodes = module._control_nodes(_context(module))
    spawners = {
        str(node._Node__arguments[0]): node._Node__arguments
        for node in nodes
        if isinstance(node, Node) and
        node._Node__node_executable == "spawner"
    }

    def parameter_files(arguments):
        return [
            Path(arguments[index + 1]).name
            for index, argument in enumerate(arguments)
            if argument == "--param-file"
        ]

    assert "--inactive" in spawners["forward_position_controller"]
    assert "--inactive" in spawners["joint_trajectory_controller"]
    assert parameter_files(spawners["forward_position_controller"]) == [
        "default_31dof_param.yaml", "forward_position_controller.yaml"]
    assert parameter_files(spawners["joint_trajectory_controller"]) == [
        "default_31dof_param.yaml", "joint_trajectory_controller.yaml"]
    assert "arm_gravity_compensation" not in spawners


_HEAD_JOINT = """<robot name="t">
  <joint name="d435_joint" type="fixed">
    <origin rpy="0 0.8307767 0" xyz="0.0576235 0.01753 0.42987"/>
    <parent link="torso_link"/>
    <child link="d435_link"/>
  </joint>
</robot>"""


def _origin_of(root, joint):
    node = [j for j in root.iter("joint") if j.get("name") == joint][0]
    return node.find("origin")


def test_calibrated_joint_origin_replaces_the_nominal_one():
    module = _load_control_launch()
    root = ElementTree.fromstring(_HEAD_JOINT)
    applied = module._apply_joint_origins(root, {"d435_joint": {
        "parent": "torso_link", "child": "d435_link",
        "xyz": [0.058, 0.0175, 0.4299], "rpy": [-0.0167, 0.8676, -0.0036]}})
    assert applied == ["d435_joint"]
    origin = _origin_of(root, "d435_joint")
    assert origin.get("xyz") == "0.058 0.0175 0.4299"
    assert origin.get("rpy") == "-0.0167 0.8676 -0.0036"


def test_calibrated_origin_is_skipped_when_the_urdf_moved_the_joint():
    """存的是 T_parent<-child。URDF 改了挂点就不是同一条边，叠上去是错的。"""
    module = _load_control_launch()
    root = ElementTree.fromstring(_HEAD_JOINT.replace("torso_link", "pelvis"))
    applied = module._apply_joint_origins(root, {"d435_joint": {
        "parent": "torso_link", "child": "d435_link",
        "xyz": [1.0, 2.0, 3.0], "rpy": [0.0, 0.0, 0.0]}})
    assert applied == []
    assert _origin_of(root, "d435_joint").get("xyz") == "0.0576235 0.01753 0.42987"


def test_unknown_joints_in_the_override_file_are_ignored():
    module = _load_control_launch()
    root = ElementTree.fromstring(_HEAD_JOINT)
    assert module._apply_joint_origins(root, {"some_other_joint": {
        "parent": "a", "child": "b", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}}) == []
    assert _origin_of(root, "d435_joint").get("xyz") == "0.0576235 0.01753 0.42987"


_WRIST_MOUNT = """<robot name="t">
  <link name="left_camera_mount_link"/>
</robot>"""


def test_wrist_camera_link_is_injected_because_the_urdf_has_no_optical_frame():
    module = _load_control_launch()
    root = ElementTree.fromstring(_WRIST_MOUNT)
    applied = module._apply_joint_origins(root, {"left_camera_optical_joint": {
        "parent": "left_camera_mount_link", "child": "camera_left", "create": True,
        "xyz": [-0.0004, 0.0401, 0.0724], "rpy": [-0.24, 0.036, -3.111]}})
    assert applied == ["left_camera_optical_joint"]
    assert "camera_left" in {link.get("name") for link in root.iter("link")}
    joint = [j for j in root.iter("joint")][0]
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "left_camera_mount_link"
    assert joint.find("child").get("link") == "camera_left"
    assert joint.find("origin").get("xyz") == "-0.0004 0.0401 0.0724"


def test_injected_joint_is_skipped_when_its_parent_link_is_missing():
    module = _load_control_launch()
    root = ElementTree.fromstring(_WRIST_MOUNT)
    assert module._apply_joint_origins(root, {"right_camera_optical_joint": {
        "parent": "right_camera_mount_link", "child": "camera_right", "create": True,
        "xyz": [0, 0, 0], "rpy": [0, 0, 0]}}) == []
    assert list(root.iter("joint")) == []


def test_existing_joint_is_edited_not_duplicated_even_when_create_is_set():
    """create 只是“没有就建”，有了就该改 —— 建重了 URDF 直接不合法"""
    module = _load_control_launch()
    root = ElementTree.fromstring(_HEAD_JOINT)
    applied = module._apply_joint_origins(root, {"d435_joint": {
        "parent": "torso_link", "child": "d435_link", "create": True,
        "xyz": [1.0, 2.0, 3.0], "rpy": [0.0, 0.0, 0.0]}})
    assert applied == ["d435_joint"]
    assert len(list(root.iter("joint"))) == 1
    assert _origin_of(root, "d435_joint").get("xyz") == "1.0 2.0 3.0"


def test_missing_camera_calibration_package_is_not_fatal():
    """标定包是可选的，没装也得能启控制栈"""
    module = _load_control_launch()
    assert isinstance(module._calibrated_joint_origins(), dict)


def test_urdf_still_has_the_links_the_camera_overrides_hang_off():
    """URDF 是 submodule，link 一改名这些覆盖就静静失效，只留一行 warning"""
    module = _load_control_launch()
    description = module._robot_description(_context(module), PACKAGE_ROOT, "dual")
    root = ElementTree.fromstring(description)
    links = {link.get("name") for link in root.iter("link")}
    assert {"torso_link", "d435_link",
            "left_camera_mount_link", "right_camera_mount_link"} <= links
    assert "d435_joint" in {j.get("name") for j in root.iter("joint")}
    # 腕相机光心不在 URDF 里，标定完才插进去
    assert not {"camera_left", "camera_right"} & links
