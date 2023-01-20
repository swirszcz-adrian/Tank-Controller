# Tank-Controller
Backup of ROS-based controller for a tank-like robot

Robot was based on Traxster II skeleton made by Robot Connections. Arduino Uno and Monster Moto Shield were used for controlling robot's DC motors (using simple PI controllers). 
Most algorithms were implemented on Raspberry Pi 4B running on Raspbian Legacy OS with ROS Noetic.
Robot was also equipped with Adafruit BNO055 IMU and Raspberry Pi NoIR Camera as well as an alphanumeric display.

# Hardware connections

![Hardware connections](https://github.com/swirszcz-adrian/Tank-Controller/blob/master/images/hardware-connections.png)

> **Image sources:**
> - [Monster Moto Shield](https://kamami.pl/sterowniki-silnikow-dc/582462-monster-moto-shield-modul-z-podwojnym-sterownikiem-silnikow-dc-vnh2sp30-kompatybilny-z-arduino.html )
> - [Raspberry Pi 4B](https://botland.com.pl/moduly-i-zestawy-raspberry-pi-4b/16579-raspberry-pi-4-model-b-wifi-dualband-bluetooth-8gb-ram-15ghz-765756931199.html)
> - [Arduino Uno](https://botland.com.pl/arduino-seria-podstawowa-oryginalne-plytki/1060-arduino-uno-rev3-a000066-7630049200050.html)
> - [Adafruit BNO055 IMU](https://ca.robotshop.com/products/bno055-9-dof-absolute-orientation-imu-fusion-breakout-board)
> - [LCD dislay](https://botland.com.pl/wyswietlacze-alfanumeryczne-i-graficzne/19732-wyswietlacz-lcd-2x16-znakow-niebieski-ze-zlaczami-justpi-5903351243131.html)
> - [Raspberry Pi NoIR Camera](https://botland.com.pl/kamery-do-raspberry-pi/6128-raspberry-pi-noir-camera-hd-v2-8mpx-oryginalna-kamera-nocna-dla-raspberry-pi-640522710898.html)
> - [DC Motors with encoders](https://www.hennkwell.com.tw/en/product/Dia.-32mm-Gearbox-Motor/PK32.html)

IMU was connected using basic I2C bus.

Instruction on how to connect display can be found [here](https://www.circuitbasics.com/raspberry-pi-lcd-set-up-and-programming-in-c-with-wiringpi/).

# Folder structure

    .
    ├── arduino
    │ └── arduino_motor_controller            # Current version of controller, implemented on Arduino
    │ └── arduino_motor_controller_obsolete   # Old, obsolete Arduino controller
    │ └── arduino_test_connection             # Used for testing connection quality between Raspberry Pi and Arduino Uno
    │ └── arduino_test_pid                    # Used for testing PI controller's response
    ├── images                                # Contains images used in this documentation
    ├── include
    │ └── tank_controller                     # Contains header files for library used for communication with IMU
    ├── launch                                # Contains launch file, for starting all project nodes and master node
    ├── msg                                   # Contains all custom messages
    ├── scripts                               # Contains obsolete Python scripts
    ├── src                                   # Current cpp files responsible for running all the nodes as well as running tests
    │ └── tank_controller                     # Contains source files for library used for communication with IMU
    └── srv                                   # Contains all custom services

# Dependencies and instalation
This project was originally created on Raspbian Legacy OS and ROS Noetic. Other operating systems should work, as long as you install **[WiringPi](http://wiringpi.com/)** 
library, which is used for communication with display and IMU over GPIO pins and I2C bus. 

Other dependencies include basic ROS packages (roscpp, rospy, std_msgs, message_generation) and:
- [Serial package](http://wjwwood.io/serial/)
- [SFML library](https://www.sfml-dev.org/)

If you want to use outdated python scripts, please install Python 3 (preferably 3.7.3) as well as [pyame](https://www.pygame.org/news) and [pyserial](https://pypi.org/project/pyserial/).

**WARNING!**
Raspberry Pi may have issues while communicating with Adafruit BNO055 IMU over I2C, caused by sensor's **clock-stretching** (not supported by Raspberry's basic I2C bus). 
To prevent data loss, please lover Rasperry's I2C clock frequency from 100kHz to 20kHz.
This can be done by adding ```i2c-bcm2708 baudrate=20000``` line to the ```/etc/modules``` file:

Once all dependecies have been installed, and I2C clock frequency has been lowered, you can safely install package by cloning files into your catkin workspace 
and running ```catkin_make``` command.
