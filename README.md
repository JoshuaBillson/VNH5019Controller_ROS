# Package Summary
This package aims to provide a means for integrating the 
[Open Source VNH5019 Motor Controller](https://github.com/JoshuaBillson/VNH5019Controller) with 
the ROS development environment. The motors are controlled by publishing to the 
[vnh5019_motor_controller](#Topics) topic.

# <a name="Nodes"></a>Nodes
### vnh5019_write_serial
Writes to the motor controller over USB.  
#### Parameters
##### port
Type: `string`  
The serial port to which the motor controller is connected.  
##### duty_cycle
Type: `float`  
Default: `1.0`  
The max duty cycle of the motors from 0 to 1. Useful for driving motors above their rated voltage.  
#### Example
```
rosrun vnh5019_motor_controller vnh5019_write_serial _port:="/dev/ttyACM0" _duty_cycle:=0.9
```

# Topics
### vnh5019_motor_controller
Type: `vnh5019_serial_controller/MixedCommand`  
Publish to this topic to write a speed and turn value to the motor controller.

# Messages
### MixedCommand
##### speed
Type: `Int8`  
Values: `-100 (full reverse) to 100 (full forward)`  
##### turn
Type: `Int8`  
Values: `-100 (max left) to 100 (max right)` 
##### Example
```python
msg = MixedCommand(speed=100, turn=0)
```

# Example
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
