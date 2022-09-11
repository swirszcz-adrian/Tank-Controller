#!/usr/bin/env python


from math import pi
from typing import Callable
import rospy
import time
import pygame
from tank_controller.msg import motorsManualControl
from tank_controller.msg import vector3D
from tank_controller.srv import globalStop
from std_msgs.msg import Header


class GamepadBridge():
    """Class responsible for creating and running node, that captures PS4 controller's inputs and scales it accordingly"""

    def __init__(self, scale_linear: float = 1.5, scale_angular: float = 1.0 * pi, deadzone: float = 0.1):
        """Start pygame for capturing controller's data, initialize ROS nodes"""
        rospy.init_node("gamepad_bridge", anonymous=False)
        self.rate_int = 20                      # [Hz]
        self.rate = rospy.Rate(self.rate_int)   # [Hz]

        rospy.loginfo("Starting up gamepad_bridge node:")

        self.global_stop = rospy.ServiceProxy("/global_stop", globalStop)
        rospy.loginfo(" - client nodes created")

        self.control_pub = rospy.Publisher("/motors_manual_control", motorsManualControl, queue_size=1)
        rospy.loginfo(" - publishers created")
        
        pygame.init()
        rospy.loginfo(" - pygame started")

        # Button mapings
        self.button_keys = {
            "x": 0,
            "circle" : 1,
            "triangle" : 2,
            "square": 3,
            "L1" : 4,
            "R1" : 5,
            "share" : 8,
            "options" : 9,
            "PS" : 10,
            "L3" : 11,
            "R3" : 12,
        }
        # self.analog_keys = {    # This one won't be used in code, it's just for me to remember bindings
        #     "left_hor" : 0,
        #     "left_ver" : 1,
        #     "L2" : 2,
        #     "right_ver" : 3,
        #     "right_ver" : 4,
        #     "R2" : 5
        # }
        self.analog_values = {
            0 : 0.0,
            1 : 0.0,
            2 : 0.0,
            3 : 0.0,
            4 : 0.0,
            5 : 0.0
        }

        # Internal variables
        self.is_connected = False
        self.is_taking_control = False
        self.l2_pressed = False
        self.r2_pressed = False
        self.scale_linear = scale_linear
        self.scale_angular = scale_angular
        self.deadzone = deadzone
        rospy.loginfo("Initialization successfull, attempting to establish comunication with gamepad")


        # Check if controller is connected
        self.checkGamepad()
        if self.is_connected:
            rospy.loginfo("Gamepad controller successfully connected!")
        else:
            rospy.logwarn("Gamepad controller is not connected!")


    def __del__(self) -> None:
        """Quit pygame"""
        pygame.quit()
        print("|=== Node has been sucessfully destroyed ===|")


    def globalStopClient(self, stop: bool) -> None:
        """Small function that calls /global_stop service and logs info"""
        try:
            self.global_stop(Header(stamp=rospy.Time.now()), stop)
        except rospy.ServiceException as e:
            rospy.logwarn("/global_stop service call failed: %s"%e)
        else:
            if stop:
                rospy.loginfo("Sent request to ACTIVATE global stop")
            else:
                rospy.loginfo("Sent request to DEACTIVATE global stop")


    def takeControl(self) -> None:
        """Small function for taking over manual control and logging info"""
        if not self.is_taking_control:
            self.is_taking_control = True
            self.globalStopClient(True)     # Just for safety
            rospy.loginfo("Switching to manual control")


    def returnControl(self) -> None:
        """Small function for returning control to MCU and logging info"""
        if self.is_taking_control:
            self.is_taking_control = False
            self.globalStopClient(True)     # Just for safety
            rospy.loginfo("Switching to automatic control")


    def checkGamepad(self) -> bool:
        """Function called whenever no new input is received from the controller to check whether controller is still connected"""
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count():
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            self.is_connected = True
        else:
            self.is_connected = False
            self.returnControl()


    def run(self) -> None:
        """Main program loop. The "try: finally:" clauzule was used to prevent program from throwing errors after Ctrl+C was passed via terminal"""
        rospy.loginfo("Waiting for /global_stop service")
        rospy.wait_for_service("/global_stop")
        rospy.loginfo("Connection with /global_stop service established")
        loops_without_contact = 0

        try:
            while not rospy.is_shutdown():
                # No new inputs, increase the counter
                if not pygame.event.peek():
                    loops_without_contact += 1
                    # If counter reached rate value (aka. 1 second passed without new controls) check if controller is connected
                    if loops_without_contact == self.rate_int:
                        loops_without_contact = 0
                        self.checkGamepad()
                        if not self.is_connected:
                            rospy.logwarn("Controller has been disconected")

                else:
                    # After reconnecting controller there would be some leftover trash in events, that would cause get() method to throw an error
                    try:
                        events = pygame.event.get()
                    except BaseException:
                        pass
                    
                    else:
                        for event in events:
                            # Pygame has been stopped (this realistically shouldn't happen)
                            if event.type == pygame.QUIT:
                                rospy.logwarn("Pygame has been stopped, destroying node...")
                                return None
                            
                            # Gamepad button has been pressed
                            if event.type == pygame.JOYBUTTONDOWN:
                                # Turn global stop ON/OFF
                                if event.button == self.button_keys["x"]:
                                    if self.l2_pressed and self.r2_pressed:
                                        self.globalStopClient(False)
                                    else:
                                        self.globalStopClient(True)
                                
                                # Take/Return control
                                if event.button == self.button_keys["triangle"]:
                                    if self.l2_pressed and self.r2_pressed:
                                        self.returnControl()
                                    else:
                                        self.takeControl()
                                
                            # Joystick has been moved, look into GamepadBridge.__init__() method for analog key bindings
                            if event.type == pygame.JOYAXISMOTION:
                                self.analog_values[event.axis] = event.value
                
                # Check whether L2 and R2 are pressed
                self.l2_pressed = True if self.analog_values[2] > 0.5 else False
                self.r2_pressed = True if self.analog_values[5] > 0.5 else False

                # Fill basic data
                msg = motorsManualControl()
                msg.header.stamp = rospy.Time.now()
                msg.isConnected = self.is_connected
                msg.takeControl = self.is_taking_control

                # Scale and fill x and z values (if they are smaller than deadzone value, assume they are equal to 0)
                msg.linear.x = -self.scale_linear * self.analog_values[1] if abs(self.analog_values[1]) > self.deadzone else 0.0
                msg.angular.z = -self.scale_angular * self.analog_values[0] if abs(self.analog_values[0]) > self.deadzone else 0.0

                # Publish message
                self.control_pub.publish(msg)

                # Wait
                self.rate.sleep()
        finally:
            msg = motorsManualControl()
            msg.header.stamp = rospy.Time.now()
            msg.isConnected = False
            msg.takeControl = False
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.control_pub.publish(msg)
            rospy.loginfo("Destroying node...")



    def __call__(self) -> None:
        """Calls run() method"""
        self.run()


if __name__ == "__main__":
    gamepad_bridge = GamepadBridge()
    gamepad_bridge()

