#!/usr/bin/env python


from math import pi
from typing import Callable
import rospy
import time
from multiprocessing import Process, Value
from pyPS4Controller.controller import Controller
from tank_controller.msg import motorsManualControl
from tank_controller.msg import vector3D
from tank_controller.srv import globalStop
from std_msgs.msg import Header


class GamepadReadData(Controller, Process):
    """Class responsible for capturing PS4 controller's inputs"""
    
    def __init__(self, x, z, is_connected, is_controlling, globalstop_service: Callable, interface: str = "/dev/input/js0"):
        """Initialize all parent classes, save variables for multiprocessing"""
        Controller.__init__(self, interface=interface, connecting_using_ds4drv=False)
        Process.__init__(self)
        self.x = x
        self.z = z
        self.connected = is_connected
        self.is_controlling = is_controlling
        self.globalstop = globalstop_service
        self.__l1_pressed = False
        self.__r1_pressed = False
    

    def controller_connected(self) -> None:
        """Function called automatically upon connecting controller"""
        self.connected.value = 1
        rospy.loginfo("Controller has been connected")
        

    def controller_disconnected(self) -> None:
        """Function called automatically upon disconnecting controller"""
        self.globalstop(True)
        self.connected.value = 0
        rospy.logwarn("Controller has been disconnected")


    # Overriding key and joystick interrupts
    # This is the reason why I had to betray camelCase


    def on_x_press(self):
        """Stop or move robot"""
        if self.__l1_pressed and self.__r1_pressed:
            self.globalstop(False)
        else:
            self.globalstop(True)

    
    def on_triangle_press(self):
        """Take or return control"""  
        if self.__l1_pressed and self.__r1_pressed:
            self.is_controlling = False
        else:
            self.is_controlling = True


    def on_x_release(self):
        pass


    def on_triangle_release(self):
        pass


    def on_L1_press(self):
        """L1 and R1 act as switch button"""
        self.__l1_pressed = True

    
    def on_R1_press(self):
        """L1 and R1 act as switch button"""
        self.__r1_pressed = True

    
    def on_L1_release(self):
        """L1 and R1 act as switch button"""
        self.__l1_pressed = False


    def on_R1_release(self):
        """L1 and R1 act as switch button"""
        self.__r1_pressed = False


    def on_L3_up(self, value):
        self.x.value = value


    def on_L3_down(self, value):
        self.x.value = value


    def on_L3_y_at_rest(self):
        self.x.value = 0.0


    def on_L3_left(self, value):
        self.z.value = value


    def on_L3_right(self, value):
        self.z.value = value


    def on_L3_x_at_rest(self):
        self.z.value = 0.0


    def run(self):
        """Main program loop, that is also an override of Process'es method"""
        while True:
            self.listen(on_connect=self.controller_connected, on_disconnect=self.controller_disconnected, timeout=3600)


class GamepadBridge(Process):
    """Class responsible for creating and running node, that captures PS4 controller's inputs and scales it accordingly"""
    
    def __init__(self, interface: str = "/dev/input/js0", scale_linear: float = 3.0, scale_angular: float = 1.0 * pi, deadzone: float = 0.1):
        """Create ros nodes, start proccess capturing data from connected PS4 controller"""
        rospy.init_node("gamepad_bridge", anonymous=False)
        self.rate = rospy.Rate(20)  # [Hz]
        rospy.loginfo("Starting up gamepad_bridge node:")

        self.global_stop = rospy.ServiceProxy("/global_stop", globalStop)
        rospy.loginfo(" - client nodes created")

        self.control_pub = rospy.Publisher("/motors_manual_control", motorsManualControl, queue_size=1)
        rospy.loginfo(" - publishers created")

        self.x = Value("d", 0.0)
        self.z = Value("d", 0.0)
        self.is_connected = Value("i", 0)
        self.is_controlling = Value("i", 0)
        self.listener_process = GamepadReadData(self.x, self.z, self.is_connected, self.is_controlling, self.global_stop_client, interface)
        rospy.loginfo(" - listener created")

        self.scale_linear = scale_linear
        self.scale_angular = scale_angular
        self.deadzone = deadzone
        rospy.loginfo("Initialization successfull, starting listener process")
        self.listener_process.start()


    def __del__(self):
        """Send feedback"""
        print("|=== Node has been sucessfully destroyed ===|")


    def global_stop_client(self, stop: bool) -> None:
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

    
    def publish_motors_control(self) -> None:
        """Function responsible for scaling controller's input data and publishing it"""
        # Fill basic data
        msg = motorsManualControl()
        msg.header.stamp = rospy.Time.now()
        msg.isConnected = self.is_connected.value
        msg.takeControl = self.is_controlling.value

        # Scale control value, if it's smaller than joystick's deadzone assume it's zero
        linear = self.x.value * (-0.0000305185)
        linear = self.scale_linear * linear if abs(linear) >= self.deadzone else 0.0

        angular = self.z.value * (-0.0000305185)
        angular = self.scale_angular * angular if abs(angular) >= self.deadzone else 0.0

        # Fill remaining data and publish message
        msg.linear.x = linear
        msg.angular.z = angular   
        self.control_pub.publish(msg)     


    def run(self) -> None:
        """Main program loop. The "try: finally:" clauzule was used to prevent program from throwing errors after Ctrl+C was passed via terminal"""
        rospy.loginfo("Waiting for /global_stop service")
        rospy.wait_for_service("/global_stop")
        try:
            while not rospy.is_shutdown():
                self.publish_motors_control()
                self.rate.sleep()
        finally:
            rospy.loginfo("Destroying node...")
            self.listener_process.terminate()


    def __call__(self) -> None:
        """Calls run() method"""
        self.run()

if __name__ == "__main__":
    gamepad_bridge_node = GamepadBridge()
    gamepad_bridge_node()