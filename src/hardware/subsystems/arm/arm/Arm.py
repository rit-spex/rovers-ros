import rclpy
from rclpy import Node, subscription
from custom_interfaces.msg import ArmMotor

class Arm(Node):
    # Base
    base_spd: subscription
    base_dir: subscription
    # Gripper
    grip_spd: subscription
    grip_dir: subscription
    # Wrist
    wrst_spd: subscription
    wrst_dir: subscription

    def __init__(self) -> None:
        super.__init__('Arm')

        self.base_spd = self.create_subscription(ArmMotor, "/ARM/BASE/SPEED", lambda x:
            1
        )
        self.base_dir = self.create_subscription(ArmMotor, "/ARM/BASE/DIRECTION")
        self.grip_spd = self.create_subscription(ArmMotor, "/ARM/GRIPPER/SPEED")
        self.grip_dir = self.create_subscription(ArmMotor, "/ARM/GRIPPER/DIRECTION")
        self.wrst_spd = self.create_subscription(ArmMotor, "/ARM/WRIST/SPEED")
        self.wrst_dir = self.create_subscription(ArmMotor, "/ARM/WRIST/DIRECTION")
