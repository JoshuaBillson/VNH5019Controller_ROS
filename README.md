# Package Summary
This package aims to provide a means for interfacing the 
[Open Source VNH5019 Motor Controller](https://github.com/JoshuaBillson/VNH5019Controller) within 
the ROS development environment. The motors are controlled by publishing to the 
[vnh5019_motor_controller](#Topics) topic. For convenience, a keyboard teleoperation node 
[keyboard_teleop](#Nodes) has been provided.

## <a name="Nodes"></a>Nodes
**serial_transmitter:** Writes to the motor controller over USB. Expects the port to be given as an argument.  
**keyboard_teleop** Provides keyboard teleoperation for the motor controller via the arrow keys.

## <a name="Topics"></a>Topics
**vnh5019_motor_controller ([vnh5019_serial_controller/MixedCommand](#MixedCommand)):** Publish to this 
topic to write to the motor controller.

## Launch Files
**serial.launch:** Launches the write-over-usb protocol for the motor controller. Takes the USB port associated
with the motor controller as an argument.  
```
roslaunch vnh5019_motor_controller serial.launch port:=/dev/ttyACM0
```
**teleop.launch:** Launches the keyboard teleoperation protocol. Takes the USB port associated with the 
motor controller as an argument.  
```
roslaunch vnh5019_motor_controller teleop.launch motor_port:=/dev/ttyACM0
```

## <a name="MixedCommand"></a>vnh5019_serial_controller/MixedCommand
**int8 speed:** A value from -100 (full reverse) to 100 (full forward).  
**int8 turn:** A value from -100 (max left) to 100 (max right).

## Example
```python
import rospy
from vnh5019_serial_controller.msg import MixedCommand

motors = rospy.Publisher('vnh5019_motor_controller', MixedCommand, queue_size=10)

def stop():
    global motors
    motors.publish(MixedCommand(speed=0, turn=0))


def turn_left():
    global motors
    motors.publish(MixedCommand(speed=0, turn=-50))


def turn_right():
    global motors
    motors.publish(MixedCommand(speed=0, turn=50))


def forward():
    global motors
    motors.publish(MixedCommand(speed=100, turn=0))


def reverse():
    global motors
    motors.publish(MixedCommand(speed=-100, turn=0))
```
