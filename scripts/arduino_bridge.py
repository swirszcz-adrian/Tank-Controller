#!/usr/bin/env python


import rospy
import serial
from tank_controller.msg import motorsManualControl, motorsAutoControl, motorsData
from tank_controller.msg import vector3D
from tank_controller.srv import globalStop, globalStopRequest, globalStopResponse
from time import sleep


class ArduinoBridge():
    """Class responsible for creating and running node, that takes care of comunicationg with Arduino board"""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 9600, timeout:float = 0.05) -> None:
        """Initialize serial communiucation, create ros nodes"""
        rospy.init_node("arduino_bridge", anonymous=False)
        self.rate = rospy.Rate(10)  # [Hz]
        rospy.loginfo("Starting up arduino_bridge node:")

        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.flushInput()
        self.ser.flushOutput()
        rospy.loginfo(" - serial communication established")

        self.stop_service = rospy.Service("/global_stop", globalStop, self.globalStopCallback)
        rospy.loginfo(" - services created")

        self.manual_ctrl_sub = rospy.Subscriber("/motors_manual_control", motorsManualControl, self.motorsManualControlCallback)
        self.auto_ctrl_sub = rospy.Subscriber("/motors_auto_control", motorsAutoControl, self.motorsAutoControlCallback)
        rospy.loginfo(" - subscribers created")

        self.data_pub = rospy.Publisher("/motors_data", motorsData, queue_size=1)
        rospy.loginfo(" - publishers created")

        self.controller_connected = False
        self.manual_control = False
        rospy.loginfo("Initialization successfull")


    def __del__(self) -> None:
        """Close serial port"""
        self.ser.close()
        print("|=== Node has been sucessfully destroyed ===|")

    
    def globalStopCallback(self, request: globalStopRequest) -> globalStopResponse:
        """Function called whenever a global stop is requested"""
        if request.stop:
            self.ser.write(b"ST ON\n")
            rospy.loginfo("Received request to ACTIVATE global stop")
        else:
            self.ser.write(b"ST OFF\n")
            rospy.loginfo("Received request to DEACTIVATE global stop")
        
        # Send back empty response
        return globalStopResponse()


    def motorsSetControl(self, linear: vector3D, angular: vector3D) -> None:
        """Function called to actually pass control data to Arduino board (after it has been decided whether to use automatic or manual control)
        - wheel radius, r = 0.073 [m]
        - wheels spacing, d = 0.18 [m]
        - ticks per rotation, tr = 624
        """  
        # Convert linear velocity from [m/s] to [ticks/ms]
        #   linear velocity = v_lin / (2 * π * r) * tr / 1000
        #   where: 1 / (2 * π * r) * tr / 1000 ≈ 1.3604477
        v_lin: float = linear.x * 1.360477
        
        # Convert angular velocity from [rad/s] to [ticks/ms]
        #   angular velocity = ±v_ang / (2 * π) * (π * d) / (2 * π * r) * tr / 1000
        #   where: 1 / (2 * π) * (π * d) / (2 * π * r) * tr / 1000 ≈ 0.12244
        v_ang: float = angular.z * 0.12244

        # Send calculated values to Arduino board
        motor_left: str = "{:+.2f}".format(v_lin - v_ang)
        motor_right: str = "{:+.2f}".format(v_lin + v_ang)
        self.ser.write('MV {} {}\n'.format(motor_left[:5], motor_right[:5]).encode())


    def motorsManualControlCallback(self, message: motorsManualControl) -> None:
        """Function called whenever manual controller sends new data"""
        # Update controller status
        self.controller_connected = message.isConnected
        self.manual_control = message.takeControl

        # Send data only if manual controller is connected and is taking over control
        if self.controller_connected and self.manual_control:
            self.motorsSetControl(message.linear, message.angular)


    def motorsAutoControlCallback(self, message: motorsAutoControl) -> None:
        """Function called whenever automatic controller sends new data"""
        # Send data only if manual controller is disconnected or is not taking over control
        if not self.controller_connected or not self.manual_control:
            self.motorsSetControl(message.linear, message.angular)


    def motorsDataRead(self) -> None:
        """Function used to read"""
        # Read data from serial port
        try:
            data = self.ser.readline().decode(encoding="ascii").strip().split(sep=" ")
        except BaseException:
            # check if there is actual data
            rospy.logwarn("Unrecognized message received from Arduino board")
        else:
            # Check if data is not corrupted
            if (data[0] == "MV" or data[0] == "ST") and (len(data) == 3):
                # Try to read motors' speed and convert it to float
                try:
                    motor_left = float(data[1])
                    motor_right = float(data[2])
                except BaseException:
                    rospy.logwarn("Received corrupted values from Arduino board")
                else:
                    # Fill basic data
                    msg = motorsData()
                    msg.header.stamp = rospy.Time.now()
                    msg.isStopped = True if data[0] == "ST" else False

                    # Separate data info linear and angular velocity
                    v_ang = 0.5 * (motor_right - motor_left)
                    v_lin = motor_left + v_ang

                    # Do the reverse of motorsSetControl() function
                    v_ang = v_ang / 0.12244
                    v_lin = v_lin / 1.360477

                    # Fill rest of the data and publish message
                    msg.linear.x = v_lin
                    msg.angular.z = v_ang
                    self.data_pub.publish(msg)
            else:
                rospy.logwarn("Received corrupted data from Arduino board")


    def run(self) -> None:
        """Main program loop. The "try: finally:" clauzule was used to prevent program from throwing errors after Ctrl+C was passed via terminal"""
        # Wait a while for serial communication
        rospy.loginfo("Starting up communication. Please wait...")
        sleep(3)
        rospy.loginfo("Communication has been established!")
        
        try:
            while not rospy.is_shutdown():
                self.motorsDataRead()
                self.rate.sleep()
        finally:
            rospy.loginfo("Destroying node...")

    
    def __call__(self) -> None:
        """Calls run() method"""
        self.run()


if __name__ == "__main__":
    arduino_bridge_node = ArduinoBridge()
    arduino_bridge_node()