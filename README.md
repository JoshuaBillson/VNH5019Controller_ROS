# Package Summary
This package aims to provide a means for interfacing the 
[Open Source VNH5019 Motor Controller](https://github.com/JoshuaBillson/VNH5019Controller) within 
the ROS development environment. The motors are controlled by publishing to the 
**vnh5019_motor_controller** topic. For convenience, a keyboard teleoperation node **keyboard_teleop** 
has been provided.

## Nodes
**serial_transmitter:** Writes to the motor controller over USB.  
**keyboard_teleop** Provides keyboard teleoperation for the motor controller via the arrow keys.

## Topics
**vnh5019_motor_controller (vnh5019_serial_controller/MixedCommand):** Publish to this topic to write to the
motor controller.

## Launch Files
**serial.launch:** Launches the write-over-usb protocol for the motor controller.  
**teleop.launch:** Launches the keyboard teleoperation protocol.
