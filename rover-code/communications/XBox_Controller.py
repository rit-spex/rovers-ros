#!/usr/bin/env python3
"""
File: XBox_Controller.py

Description: This script defines the XboxController class, which interfaces with
an Xbox controller connected to the system using the Pygame and ROS 2 (rclpy)
libraries. The class reads the button presses, axis movements, and D-pad (hat)
states of the Xbox controller and publishes them as Joy messages on ROS 2 topics.
It publishes the controller state only when there is a change in any of these states,
reducing unnecessary message publishing.

Author: Ryan Barry
Date Created: July 18, 2023

Dependencies:

    rclpy
    sensor_msgs.msg.Joy
    pygame

"""

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxController(Node):
    def __init__(self):
        super().__init__("xbox_controller")
        pygame.init()
        self.joystick = None
        self.button_states = []
        self.axis_states = []
        self.hat_states = []

        # Initialize the Xbox controller
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            # Check if the connected joystick is an Xbox controller
            if "Xbox" in joystick.get_name():
                self.joystick = joystick
                self.button_states = [False] * joystick.get_numbuttons()
                self.axis_states = [0.0] * joystick.get_numaxes()
                self.hat_states = [(0, 0)] * joystick.get_numhats()
                break

        if not self.joystick:
            raise Exception("Xbox controller not found!")

        # Create publishers for Joy message
        self.button_publisher = self.create_publisher(Joy, "xbox_controller/buttons", 10)
        self.axis_publisher = self.create_publisher(Joy, "xbox_controller/axes", 10)
        self.hat_publisher = self.create_publisher(Joy, "xbox_controller/hats", 10)
        self.timer_ = self.create_timer(0.1, self.publish_controller_state)

        # Variables to track previous state
        self.prev_button_states = [False] * self.joystick.get_numbuttons()
        self.prev_axis_states = [0.0] * self.joystick.get_numaxes()
        self.prev_hat_states = [(0, 0)] * self.joystick.get_numhats()

    def publish_controller_state(self):
        # Update button states
        for i in range(self.joystick.get_numbuttons()):
            self.button_states[i] = self.joystick.get_button(i)

        # Update axis states
        for i in range(self.joystick.get_numaxes()):
            self.axis_states[i] = self.joystick.get_axis(i)

        # Update hat states
        for i in range(self.joystick.get_numhats()):
            self.hat_states[i] = self.joystick.get_hat(i)

        # Publish Joy messages only if there is a change in the state
        if self.button_states != self.prev_button_states:
            button_msg = Joy()
            button_msg.header.stamp = self.get_clock().now().to_msg()
            button_msg.buttons = self.button_states
            self.button_publisher.publish(button_msg)

        if self.axis_states != self.prev_axis_states:
            axis_msg = Joy()
            axis_msg.header.stamp = self.get_clock().now().to_msg()
            axis_msg.axes = self.axis_states
            self.axis_publisher.publish(axis_msg)

        if self.hat_states != self.prev_hat_states:
            hat_msg = Joy()
            hat_msg.header.stamp = self.get_clock().now().to_msg()
            hat_msg.axes = [axis for hat in self.hat_states for axis in hat]
            self.hat_publisher.publish(hat_msg)

        # Update previous state
        self.prev_button_states = self.button_states[:]
        self.prev_axis_states = self.axis_states[:]
        self.prev_hat_states = self.hat_states[:]
