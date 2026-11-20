"""Action registry for the arm action driver.

``ACTION_CONFIG_CLASSES`` is the one place where actions are wired into
the node.  Adding an action: subclass
``arm_action_driver.actions.base.ActionConfiguration``, implement its
hooks, and append the class here.
"""

from arm_action_driver.actions.base import ActionConfiguration
from arm_action_driver.actions.flip import FlipConfiguration
from arm_action_driver.actions.homing import HomingConfiguration
from arm_action_driver.actions.resting import (
    RestingConfiguration,
    RestingController,
)
from arm_action_driver.actions.zenith import ZenithConfiguration

# Every action the driver can run, in registry order.  The driver
# instantiates one of each; command routing, active-ownership dispatch,
# and status collection all read from these classes only.
ACTION_CONFIG_CLASSES = (
    HomingConfiguration,
    FlipConfiguration,
    ZenithConfiguration,
    RestingConfiguration,
)

__all__ = [
    "ActionConfiguration",
    "ACTION_CONFIG_CLASSES",
    "FlipConfiguration",
    "HomingConfiguration",
    "RestingConfiguration",
    "RestingController",
    "ZenithConfiguration",
]
