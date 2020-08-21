# ROS Client
import rospy

# Messages
from vnh5019_motor_controller.msg import MixedCommand
from geometry_msgs.msg import Twist

# Standard Library
from time import sleep

# Publisher For Writing To The Motor Controller
motors = rospy.Publisher('vnh5019_motor_controller', MixedCommand, queue_size=10)


def keyboard_callback(data):
    """
    Receives messages published to the key_vel topic and writes to the motors.
    data (geometry_msgs/Twist): The message received from the key_vel topic.
    """
    global motors
    msg = MixedCommand()

    # Decode Linear Velocity
    if data.linear.x > 0:
        msg.speed = 100
    elif data.linear.x < 0:
        msg.speed = -100
    else:
        msg.speed = 0

    # Decode Angular Velocity
    if data.angular.z > 0 and msg.speed == 0: # Turn Left
        msg.turn = -50
    elif data.angular.z < 0 and msg.speed == 0: # Turn Right
        msg.turn = 50
    else:
        msg.turn = 0

    motors.publish(msg)


def setup():
    """ Setup Routine """
    # Initialize Node
    rospy.init_node("motor_keyboard_teleop")

    # Initialize Subscribers
    rospy.Subscriber('key_vel', Twist, keyboard_callback)


def keyboard_teleop():
    """ Executable """
    setup()
    rospy.spin()
