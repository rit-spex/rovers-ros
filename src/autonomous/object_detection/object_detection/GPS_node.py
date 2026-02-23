#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        # 1. Declare Parameters
        # Adjust these! '/dev/ttyACM0' is common for USB, '/dev/ttyTHS1' for UART pins
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter(
            "baudrate", 9600
        )  # MAX-M10s often defaults to 38400 or 9600

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        # 2. specific Topic Name requested
        self.publisher_ = self.create_publisher(NavSatFix, "/GPS/ROVER", 10)

        # 3. Setup Serial Connection
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to GPS on {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            # We don't exit here so the node doesn't crash, but it won't publish.
            self.serial_conn = None

        # 4. Timer to read data (10Hz check)
        self.timer = self.create_timer(0.1, self.read_gps_data)

    def read_gps_data(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        try:
            # Read all available lines
            while self.serial_conn.in_waiting > 0:
                line = (
                    self.serial_conn.readline().decode("utf-8", errors="ignore").strip()
                )
                # self.get_logger().info(line)

                # Check for the GNGGA (Global Navigation) or GPGGA (GPS only) sentence
                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    # self.get_logger().info(line)
                    self.parse_and_publish(line)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def parse_and_publish(self, nmea_sentence):
        """
        Parses standard NMEA GNGGA string:
        $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        """
        parts = nmea_sentence.split(",")

        # specific check: verify we have a 'Fix Quality' > 0 (index 6)
        # If it's 0, the GPS doesn't have a lock yet.
        if len(parts) < 10 or parts[6] == "0":
            self.get_logger().info("No GPS fix yet...")
            return

        try:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"

            # Parse Latitude (DDMM.MMMM -> DD.DDDD)
            lat_raw = float(parts[2])
            lat_dir = parts[3]
            msg.latitude = self.nmea_to_decimal(lat_raw, lat_dir)

            # Parse Longitude (DDDMM.MMMM -> DD.DDDD)
            lon_raw = float(parts[4])
            lon_dir = parts[5]
            msg.longitude = self.nmea_to_decimal(lon_raw, lon_dir)

            # Parse Altitude
            msg.altitude = float(parts[9])

            # Publish!
            self.publisher_.publish(msg)
            # self.get_logger().info(f"Published GPS: {msg.latitude}, {msg.longitude}")

        except ValueError:
            pass  # Malformed string

    def nmea_to_decimal(self, coordinate, direction):
        """
        Converts NMEA format (DDMM.MMMM) to Decimal Degrees (DD.DDDD)
        """
        # Math: The first digits are degrees, the rest are minutes.
        # Longitude can be 3 digits for degrees (DDDMM), Latitude is 2 (DDMM).

        # Simple logic: Divide by 100 to separate DD and MM.MM
        degrees = int(coordinate / 100)
        minutes = coordinate - (degrees * 100)

        decimal = degrees + (minutes / 60.0)

        if direction == "S" or direction == "W":
            decimal *= -1

        return decimal


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
