"""Guard the robot description against gazebo_ros2_control's YAML parse.

gazebo_ros2_control 0.4.x builds ``--param robot_description:=<urdf>`` and
parses the value as YAML before creating the controller manager.  A
colon-space or hash-space sequence anywhere in the expanded URDF, including
inside a comment, makes that parse fail.  The plugin then returns before it
creates the controller manager, so every joint is left as a passive body
under gravity.

Keep the description YAML-safe and this test will catch regressions.
"""

import os

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

ROBOT_CONFIGS = ("drivebase", "arm_drivebase", "life_drivebase")


@pytest.mark.parametrize("robot_config", ROBOT_CONFIGS)
def test_expanded_description_is_yaml_safe(robot_config):
    """The plugin passes the expanded URDF through a YAML override."""
    description = os.path.join(
        get_package_share_directory("rover_simulation"),
        "description",
        "robot.urdf.xacro",
    )
    xml = xacro.process_file(
        description, mappings={"robot_config": robot_config}).toxml()

    try:
        parsed = yaml.safe_load(xml)
    except yaml.YAMLError as exc:
        pytest.fail(
            "Expanded robot description is not a valid YAML scalar, so "
            "gazebo_ros2_control will abort with a parser error. Remove any "
            f"colon-space or hash-space sequences. Parser error: {exc}")

    assert isinstance(parsed, str)
    assert "<robot" in parsed
