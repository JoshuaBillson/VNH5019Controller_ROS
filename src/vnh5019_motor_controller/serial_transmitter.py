import rospy
import VNH5019Controller
import sys
from vnh5019_serial_controller.msg import MixedCommand
from time import sleep

# Globals
controller = VNH5019Controller.Controller()
speed_value = 0
turn_value = 0

def receive_message(data):
    global speed_value, turn_value
    speed_value = data.speed
    turn_value = data.turn


def shutdown():
    global controller
    controller.set_standby()


def setup():
    global controller
    VNH5019Controller.init_serial(rospy.myargv(argv=sys.argv)[1])
    sleep(5)
    controller.set_active()
    sleep(2)

    rospy.init_node('serial_transmitter')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('vnh5019_motor_controller', MixedCommand, receive_message)


def loop():
    global controller, speed_value, turn_value
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        controller.write_mixed_command(speed_value, turn_value)
        rate.sleep()


def serial_transmitter():
    setup()
    loop()

