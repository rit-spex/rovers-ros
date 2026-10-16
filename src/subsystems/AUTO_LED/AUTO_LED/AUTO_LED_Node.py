import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from enum import Enum

#subscribe to publishers
#you can subscribe to topics that do not exist
#needs publisher LEDS connected to the chassis
#TRaveling over can
#DOES NOT NEED TO KNOW TEENSY PINS
#USE ENUM Instesd of string
#autonoums and astronaut autonomous are both red (1 & 2)

class status(Enum):
    Teleop = 0
    Pathfinding = 1
    Astronaut_Command = 2
    Arrival = 3 #Might change ts cuz we might not recieve.


class AUTO_LED_Node(Node): 
    mode: Enum #Rover's driving mode (Teleop, Pathfinding, Astronaut_Command, Arrival)
    arrived: bool
    led_state: str #Enum for LED state (Off, Red, Blue, Green)
    green_on: bool
    FLICKER_TIME =.5 # Constant for LED flicker time in seconds
    def __init__(self):
        super().__init__('AUTO_LED_Node') # change node name

        #Subscribe to the rover's driving mode
        self.mode_subscriber = self.create_subscription(
            String, # UINT8
            'TOPIC_NAME1', #TODO: Figure out what topic to subscribe to
            self.led_callback,
            10
        )
        #Subscribe to the rover's arrival status
        self.arrival_subscriber = self.create_subscription(
            Bool,
            'TOPIC_NAME2', #TODO:Figure out what topic to subscribe to
            self.arrival_callback,
            10
        )
        #Rover status
        self.mode = status.Teleop # DEF CHANGE YO
        self.arrived = False 
        self.led_state = "off" #TODO: Make (INT) Enum, import class
        self.green_on = False 
        
        
        self.flash_timer = self.create_timer(self.FLICKER_TIME, self.flashing)  # Timer to check LED state every second
        self.flash_timer.cancel()  # Cancel the timer initially; it will be started when needed
        self.get_logger().info('AUTO_LED_Node node has been started.') # Change Name

    def set_led(self, color: int): # make color an enum
        if color == self.led_state:
            return  # No change in LED state, do nothing
        
        self.led_state = color
        self.send_led_command(color)

  
    def led_callback(self, msg): 

        # Update the rover's driving mode based on the received message
        #msg is a string
        new_mode = msg.data.lower()  
        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f'Received driving mode: {self.mode}')
            if not self.arrived:
                self.update_led()  # Update LED state based on the new mode

 

    def arrival_callback(self, msg: Bool):
        # Update the rover's arrival status based on the received message
        #msg is a boolean
        if msg.data == self.arrived:
            return  # No change in arrival status, do nothing

        self.arrived = msg.data  

        if self.arrived: # check if auto is enabled
            self.set_led('green')  # Set LED to green when arrived
            self.green_on = True  # Start with green LED on when arrived
            self.flash_timer.reset()  # Start the timer to handle LED flickering

        else:
            self.flash_timer.cancel()  # Stop the timer when not arrived
            self.green_on = False  # Reset green LED state when not arrived
            self.update_led()  # Update LED state based on the current mode

        self.get_logger().info(f'Received arrival status: {self.arrived}') 

    
    def update_led(self):
        # Update the LED state based on the rover's status
        if self.mode == 'autonomous':
            self.set_led('red')  # Set LED to red for autonomous mode
        elif self.mode == 'manual':
            self.set_led('blue')  # Set LED to blue for manual mode


    def flashing(self):
        #when arrived does the blinking of the green LED
        if self.arrived:
            self.green_on = not self.green_on  # Toggle green LED state
            if self.green_on:
                self.set_led('green')  # Turn on green LED
            else:
                self.set_led('off')  # Turn off green LED


    def send_led_command(self, color):
        # This is a placeholder for the actual implementation
        self.get_logger().info(f'Sending LED command: {color}')
        # Implement the actual communication with the Teensy here

def main():
    rclpy.init()
    node = AUTO_LED_Node()  # change name
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    # will never get here :(
    print('Hi from AUTO_LED.')










if __name__ == '__main__':
    main()
