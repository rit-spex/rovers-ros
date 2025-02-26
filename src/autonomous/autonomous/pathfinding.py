import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import math
import time


class GeneratePath(Node):
    def __init__(self):
        super().__init__("path_generator")
        self.idSub = self.create_subscription(
            Bool, "/tracking/id_out", self.CheckData, 10
        )
        self.pointSub = self.create_subscription(
            Point, "/tracking/point_out", self.PathGenCallback, 10
        )
        self.velPubRight = self.create_publisher(Float32, "/auto/Axis/RY", 10)
        self.velPubLeft = self.create_publisher(Float32, "/auto/Axis/LY", 10)
        self.pointPub = self.create_publisher(Point, "/auto/measured_point", 10)
        self.outputVelLeft = Float32()
        self.outputVelRight = Float32()
        self.idCheck = Bool()
        self.measPoint = Point()
        self.MAX_LINEAR_VELOCITY = 2.0  # Maximum wheel linear velocity in m/s
        self.MAX_ANGULAR_VELOCITY = 5  # Maximum robot turn velocity in rad/s
        self.WHEEL_RADIUS = 0.08255  # Wheel radius in m (3.25 in)
        self.WHEEL_SEPERATION = 0.5715  # Wheel seperation distance (22.5 in)
        self.MAX_WHEEL_ANGULAR_VELOCITY = 34.557  # maximum wheel velocty on the teensy with modifier: 1100 RPM * 70% converted to rad/s

    def CheckData(self, msg):
        self.idCheck.data = msg.data

    def PathGenCallback(self, msg):
        # Incoming point is in reference to the camera, add distance from center of robot to point data to get the distance from the center
        x = (
            msg.z - 0.223
        )  # Math derived uses x as distance infront of robot but camera publishes it as z
        y = (
            msg.x + 0.183
        )  # The math derived uses y as distance left of robot but camera publisher publishes it a x and direction needs to flip

        self.measPoint.x = x
        self.measPoint.y = y
        speedMultLeft = 0.75
        speedMultRight = 0.75

        dP = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        dPhi = math.atan(y / x) * 20
        self.measPoint.z = dPhi
        print(dPhi)
        if self.idCheck.data == True:
            if dP > 1.4:
                # Try to get to point as fast as possible.
                dt = dP / self.MAX_LINEAR_VELOCITY
                # Ensure max angular velocity isn't violated, if it is, slow down the robot
                if abs(dPhi / dt) > self.MAX_ANGULAR_VELOCITY:
                    dt = dPhi / self.MAX_ANGULAR_VELOCITY

                angularVel = dPhi / dt
                linearVel = dP / dt
                rightVel = (
                    linearVel + (self.WHEEL_SEPERATION / 2) * angularVel
                ) / self.WHEEL_RADIUS
                leftVel = (
                    linearVel - (self.WHEEL_SEPERATION / 2) * angularVel
                ) / self.WHEEL_RADIUS

                rightVel = rightVel / self.MAX_WHEEL_ANGULAR_VELOCITY
                leftVel = leftVel / self.MAX_WHEEL_ANGULAR_VELOCITY
            else:
                rightVel = 0.0
                leftVel = 0.0
                dt = 0.0
        else:
            rightVel = 0.0
            leftVel = 0.0
            dt = 0.0

        # rightVel = rightVel * speedMultRight
        # leftVel = leftVel * speedMultLeft

        if rightVel > 1.0:
            rightVel = 1.0
        elif rightVel < -1.0:
            rightVel = -1.0

        if leftVel > 1.0:
            leftVel = 1.0
        elif leftVel < -1.0:
            leftVel = -1.0

        self.outputVelRight.data = 0.1  # -rightVel
        self.outputVelLeft.data = -0.1  # -leftVel

        self.pointPub.publish(self.measPoint)
        self.velPubRight.publish(self.outputVelRight)
        self.velPubLeft.publish(self.outputVelLeft)
        #     self.velPubLeft.publish(self.outputVelLeft)
        # for i in range(10):
        #     self.velPubRight.publish(self.outputVelRight)
        #     self.velPubLeft.publish(self.outputVelLeft)
        time.sleep(5)


def main(args=None):
    rclpy.init(args=args)

    pathGenerator = GeneratePath()

    rclpy.spin(pathGenerator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pathGenerator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
