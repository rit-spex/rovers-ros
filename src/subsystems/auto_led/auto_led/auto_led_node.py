
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from enum import Enum

#from custom_interfaces.msg import Can
from constants.CAN_constants import CAN_MESSAGE_IDS, TEENSY_CAN_MESSAGES, Subsystems_Names
from constants.can_encoding import TeensyCommunication
from custom_interfaces.msg import Can



#SET EVERYTHING TO LOWER CAMEL CASE
class driving_mode(Enum):
    teleop = 0 #blue
    pathfinding = 1 #red
    astronaut_command = 2 #red

class led_color(Enum): #move off to 0, and shift everything down one.
    off = 0
    blue = 1
    red = 2
    green = 3



class auto_led_node(Node): 
    mode: driving_mode | None#Rover's driving mode (Teleop, Pathfinding, Astronaut_Command, Arrival)
    arrived: bool
    green_on: bool
    FLICKER_TIME =.5 # Constant for LED flicker time in seconds


    def __init__(self):
        super().__init__('auto_led_node') # change node name

        #Subscribe to the rover's driving mode
        self.mode_subscriber = self.create_subscription(
            UInt8, # UINT8
            '/BASESTATION/auto_state/auto_state', 
            self.driving_mode_callback, 
            10
        )
        #Subscribe to the rover's arrival status
        self.arrival_subscriber = self.create_subscription(
            Bool,
            'AUTO_REACH', #TODO:Figure out what topic to subscribe to
            self.arrival_callback,
            10
        )
        self.led_can_message = TEENSY_CAN_MESSAGES[CAN_MESSAGE_IDS.AUTO_COLOR]  # Get the CAN message for LED control
        self.led_publisher = self.create_publisher(
            msg_type=Can,
            topic=self.led_can_message.topic_name,
            qos_profile=10
        )
        #Rover status
        self.mode = None # Temp until we recieve driving mode
        self.arrived = False 
        #LED state
        self.led_color = led_color.off  # Initial LED state is off
        self.green_on = False 
        
        
        self.flash_timer = self.create_timer(self.FLICKER_TIME, self.flashing)  # Timer to check LED state every second
        self.flash_timer.cancel()  # Cancel the timer initially. it will be started when needed
        self.get_logger().info('auto_led_node has been started.')



    def set_led(self, color: Enum):
        if color == self.led_color:  # If the desired color is the same as the current LED state, do nothing:
            return  
        self.led_color = color
        uint8_color = color.value  # Convert the Enum to its corresponding integer value
        #TODO: Implement the actual communication with the Teensy to set the LED color
        self.led_can_message.signals["state"].set_value(uint8_color)  # Set the color value in the CAN message
        can_packet = TeensyCommunication.encode_can_message(self.led_can_message)  # Encode the CAN message
        self.led_publisher.publish(can_packet)  # Publish the CAN message
        
        self.get_logger().info(f'Sending LED command: {uint8_color} (Color: {color.name})')  # Log the LED command being sent

  
    def driving_mode_callback(self, msg: UInt8): #come back and check out logic
        # Update the rover's driving mode based on the received message
        new_mode = driving_mode(msg.data)  # Convert the received UInt8 to the corresponding Driving_Mode enum
        if new_mode != self.mode:
            self.mode = new_mode  # Update the mode based on the received message
            self.get_logger().info(f'Received driving mode: {self.mode}')
            self.update_led()  # Update LED state based on the new mode

 

    def arrival_callback(self, msg: Bool):
        # Update the rover's arrival status based on the received message
        if msg.data == self.arrived:
            return  # No change in arrival status, do nothing
        self.arrived = msg.data  
        self.update_led()  # Update LED state based on the new arrival status
        self.get_logger().info(f'Received arrival status: {self.arrived}') 

    
    def update_led(self):
        """Update the LED state based on the rover's status."""
        autonomous = self.mode in (
            driving_mode.pathfinding,
            driving_mode.astronaut_command
       )

        if self.arrived and autonomous:
            self.green_on = True
            self.set_led(led_color.green)
            self.flash_timer.reset()
            return

        self.flash_timer.cancel()
        self.green_on = False

        if self.mode == driving_mode.teleop:
            self.set_led(led_color.blue)
        elif autonomous:
            self.set_led(led_color.red)




    def flashing(self):
        #when arrived does the blinking of the green LED
        if self.arrived:
            self.green_on = not self.green_on  # Toggle green LED state
            if self.green_on:
                self.set_led(led_color.green)  # Turn on green LED
            else:
                self.set_led(led_color.off)  # Turn off green LED



def main():
    rclpy.init()
    node = auto_led_node()  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    










if __name__ == '__main__':
    main()
