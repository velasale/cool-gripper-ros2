#!/usr/bin/env python3

# This node communicates with the gripper via PySerial
# It controls the vacumm on/off and fingers open/close

# Standard imports
import numpy as np
import serial
from serial import SerialException
import re

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperFingers
from std_msgs.msg import String


class SuctionGripper(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("cool_gripper")

        # publishers


        # services
        self.vacuum_service = self.create_service(GripperVacuum, 'set_vacuum_status', self.vacuum_service_callback())


        # set up Pyserial
        arduino_port = "dev/ttyACM0"
        baud = 57600
        try:
            self.my_serial = serial.Serial(arduino_port, baud)
            self.get_logger().info(f"Connected to serial port {arduino_port}")
            self.timer = self.create(0.1, self.timer_callback)  # to continously read the sensors
        except SerialException as e:
            self.get_logger().info(f"Could not connect to serial port {arduino_port}.")
            print(e)

        # variables
        self.start_character = "<"
        self.end_character = ">"
        self.vacuum_on_command = "1"
        self.vacuum_off_command = "2"
        self.fingers_engaged_command = "3"
        self.fingers_disengaged_command = "4"

    
    def timer_callback(self):
        "Read all messages in serial"
        todo = 1

    def vacuum_service_callback(self, request, response):
        """
        Callback function for the vacuum service. Uses the bool stored in set_vacuum to turn vacuum on/off
        """
        if request.


        todo = 1
    
    def fingers_service_callback(self, request, response):
        """
        Callback function for the fingers service. Uses the bool stored in set fingers to engage or disengage.
        Note: This might be better than an action
        """

        # If request is True, engage fingers
        if request.set_fingers:
            self.get_logger().info("Sending request: engage fingers")
            msg_str = self.start_character + self.fingers_engaged_command + self.end_character
            self.my_serial.write(str(msg_str).encode())
        
        response.results = True
        return response

    




def main(args=None):
    # Initialize node
    rclpy.init(args=args)
    # Instantiate class
    my_gripper = SuctionGripper()
    # Hand control over to ROS2
    rclpy.spin(my_gripper)
    # Shutdwon cleanly
    rclpy.shutdown

if __name__ == "__main__":
    main()