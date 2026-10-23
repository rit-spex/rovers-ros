import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node, Timer
from std_msgs.msg import Bool, UInt8, UInt16

try:
    import Jetson.GPIO as GPIO
    _GPIO_AVAILABLE = True
    _GPIO_IMPORT_ERROR = ""
    HIGH = GPIO.HIGH
    LOW = GPIO.LOW
except Exception as exc:
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False
    _GPIO_IMPORT_ERROR = str(exc)
    HIGH = 1
    LOW = 0

LED_flicker_sec = 1.0
LED_PIN_Y = 29 # Jetson Header Pin 1
LED_PIN_R = 32 # Jetson Header Pin 7
LED_PIN_G = 31 # Jetson Header Pin 11
LED_PIN_B = 33 # Jetson Header Pin 13

class StatusLED(Node):

    LED_flicker_timer: Timer

    # gpio status
    __led_flicker_state: bool
    __yellow_led_status: bool
    __green_led_status: bool

    def __init__(self):
        super().__init__("status_led_node")

        self._gpio_enabled = _GPIO_AVAILABLE
        if self._gpio_enabled:
            # GPIO setup to have led indicate heartbeat status
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(LED_PIN_Y, GPIO.OUT, initial=LOW)
            GPIO.setup(LED_PIN_R, GPIO.OUT, initial=LOW)
            GPIO.setup(LED_PIN_G, GPIO.OUT, initial=LOW)
            GPIO.setup(LED_PIN_B, GPIO.OUT, initial=LOW)
        else:
            self.get_logger().warning(
                "Jetson GPIO unavailable; status LED disabled (%s)"
                % _GPIO_IMPORT_ERROR
            )

        # requires some default value
        self.__led_flicker_state = False
        self.__yellow_led_status = True
        self.__green_led_status = False

        self.create_subscription(
            msg_type=Bool,
            topic="/ESTOP",
            callback=self.__on_estop_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/ARM/ENABLED",
            callback=self.__on_arm_enabled_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/SCIENCE/ENABLED",
            callback=self.__on_science_enabled_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=Bool,
            topic="/CHASSIS/ENABLED",
            callback=self.__on_chassis_enabled_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=UInt8,
            topic="/ROVER/AUTO_STATE",
            callback=self.__on_auto_state_received,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=UInt8,
            topic="/BASESTATION/auto_state/auto_state",
            callback=self.__on_auto_state_received,
            qos_profile=10,
        )

        self.LED_flicker_timer = self.create_timer(
            timer_period_sec=LED_flicker_sec,
            callback=self.__flicker_led,
        )

    def set_gpio(self, pin, val):
        if self._gpio_enabled:
            GPIO.output(pin, val)

    def __flicker_led(self):
        if self.__led_flicker_state:
            self.__led_flicker_state = False
            if self.__yellow_led_status:
                self.set_gpio(LED_PIN_Y, LOW)  # Turn off LED to indicate live jetson
                self.get_logger().debug("Blinking yellow off")
            if self.__green_led_status:
                self.get_logger().debug("Blinking green off")
                self.set_gpio(LED_PIN_G, LOW)  # Turn off LED to indicate arm enabled
        else:
            self.__led_flicker_state = True
            if self.__yellow_led_status:
                self.set_gpio(LED_PIN_Y, HIGH)  # Turn on LED to indicate live jetson
                self.get_logger().debug("Blinking yellow on")
            if self.__green_led_status:
                self.get_logger().debug("Blinking green on")
                self.set_gpio(LED_PIN_G, HIGH)  # Turn on LED to indicate arm enabled

    def __on_estop_received(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("E-STOP received, stop blinking yellow status LED...")
        self.__yellow_led_status = False
        if self._gpio_enabled:
            self.set_gpio(LED_PIN_Y, HIGH)  # Turn the LED solid to indicate e-stop
        rclpy.shutdown()

    def __on_auto_state_received(self, msg: UInt8):
        if msg.data > 0:
            self.get_logger().info("Auto state received, stopping blue LED...")
            self.set_gpio(LED_PIN_B, LOW)  # Turn off the LED to indicate auto state enabled
            return
        self.get_logger().info("Auto state not received, starting blue LED...")
        if self._gpio_enabled:
            self.set_gpio(LED_PIN_B, HIGH)  # Turn the LED solid to indicate auto state disabled

    def __on_arm_enabled_received(self, msg: Bool):
        if not msg.data:
            self.get_logger().info("Arm disabled, not blinking green LED...")
            self.__green_led_status = False  
            if self._gpio_enabled:
                self.set_gpio(LED_PIN_G, LOW)  # Turn off the LED to indicate arm disabled
            return
        self.get_logger().info("Arm enabled, blinking green LED...")
        self.__green_led_status = True  # Blink the LED to indicate arm enabled

    def __on_science_enabled_received(self, msg: Bool):
        if not msg.data:
            self.get_logger().info("Science dissabled, stopping green LED...")
            self.set_gpio(LED_PIN_G, LOW)  # Turn off the LED to indicate science disabled
            return
        self.get_logger().info("Science enabled, starting green LED...")
        self.__green_led_status = False
        if self._gpio_enabled:
            self.set_gpio(LED_PIN_G, HIGH)  # Turn the LED solid to indicate science enabled

    def __on_chassis_enabled_received(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Chassis ensabled, stopping red LED...")
            self.set_gpio(LED_PIN_R, LOW)  # Turn off the LED to indicate chassis enabled
            return
        self.get_logger().info("Chassis disabled, starting red LED...")
        if self._gpio_enabled:
            self.set_gpio(LED_PIN_R, HIGH)  # Turn the LED solid to indicate chassis disabled

    def destroy_node(self):
        if self._gpio_enabled:
            try:
                GPIO.cleanup(LED_PIN_Y)
                GPIO.cleanup(LED_PIN_R)
                GPIO.cleanup(LED_PIN_G)
                GPIO.cleanup(LED_PIN_B)
            except Exception:
                self.get_logger().exception("Failed to cleanup GPIO pin %d", LED_PIN_Y)
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
            status_led.get_logger().info("stopping status LED")
            rclpy.shutdown()


if __name__ == "__main__":
    main()
