import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node, Timer
from std_msgs.msg import Bool

try:
    import Jetson.GPIO as GPIO
    _GPIO_AVAILABLE = True
    _GPIO_IMPORT_ERROR = ""
except Exception as exc:
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False
    _GPIO_IMPORT_ERROR = str(exc)

LED_flicker_sec = 1.0
LED_PIN = 7


class StatusLED(Node):

    LED_flicker_timer: Timer

    # gpio status
    __led_status: bool

    def __init__(self):
        super().__init__("status_led_node")

        self._gpio_enabled = _GPIO_AVAILABLE
        if self._gpio_enabled:
            # GPIO setup to have led indicate heartbeat status
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
        else:
            self.get_logger().warning(
                "Jetson GPIO unavailable; status LED disabled (%s)"
                % _GPIO_IMPORT_ERROR
            )

        # requires some default value
        self.__led_status = False

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.LED_flicker_timer = self.create_timer(
            timer_period_sec=LED_flicker_sec,
            callback=self.__flicker_led,
        )

    def __flicker_led(self):
        if not self._gpio_enabled:
            return
        # self.get_logger().info("flicker_led")
        if self.__led_status:
            self.__led_status = False
            GPIO.output(LED_PIN, GPIO.LOW)  # Turn off LED to indicate live jetson
        else:
            self.__led_status = True
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn on LED to indicate live jetson

    def __on_estop_received(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("E-STOP received, stopping status LED...")
        self.__led_status = True
        if self._gpio_enabled:
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn the LED solid to indicate e-stop
        rclpy.shutdown()

    def destroy_node(self):
        if self._gpio_enabled:
            try:
                GPIO.cleanup(LED_PIN)
            except Exception:
                self.get_logger().exception("Failed to cleanup GPIO pin %s", LED_PIN)
        return super().destroy_node()

    def run(self):
        self.get_logger().info("starting status LED...")
        rclpy.spin(self)
        self.get_logger().info("stopping status LED")


def main():
    rclpy.init()
    status_led = StatusLED()
    try:
        status_led.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        status_led.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
