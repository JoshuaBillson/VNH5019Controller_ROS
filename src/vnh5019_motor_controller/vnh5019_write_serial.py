import rospy
import VNH5019Controller
from vnh5019_serial_controller.msg import MixedCommand
from time import sleep

# Globals
controller = VNH5019Controller.Controller()
speed_value = 0
turn_value = 0

# Parameters
DUTY_CYCLE = None


def receive_message(data):
    """Called When A New Message Is Received"""
    global speed_value, turn_value
    speed_value = data.speed
    turn_value = data.turn


def shutdown():
    """Node Shutdown Callback"""
    global controller
    controller.set_standby()


def setup():
    """Setup Procedure"""
    global controller, DUTY_CYCLE

    # Initialize Node
    rospy.init_node('vnh5019_write_serial')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('vnh5019_motor_controller', MixedCommand, receive_message)

    # Initialize Parameters
    DUTY_CYCLE = rospy.get_param("~duty_cycle", 1.0)
    assert 0 <= DUTY_CYCLE <= 1, "Error: Duty Cycle Must Be Between 0 and 1!"

    # Setup Motor Controller
    VNH5019Controller.init_serial(rospy.get_param("~port"))
    sleep(4)
    controller.set_active()
    sleep(2)


def loop():
    """Main Program Loop"""
    global controller, speed_value, turn_value, DUTY_CYCLE
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        controller.write_mixed_command(int(DUTY_CYCLE * speed_value), int(DUTY_CYCLE * turn_value))
        rate.sleep()


def vnh5019_write_serial():
    """Executable"""
    setup()
    loop()

