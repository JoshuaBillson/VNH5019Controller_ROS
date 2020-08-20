cmake_minimum_required(VERSION 2.8.3)
project(vnh5019_motor_controller)

## Package Dependencies
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
)


## Uncomment this if the package has a setup.py. This macro ensures
catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MixedCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration ##
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vnh5019_serial_controller
#  CATKIN_DEPENDS message_generation message_runtime rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  nodes/serial_transmitter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/serial.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
