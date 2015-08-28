cmake_minimum_required(VERSION 2.8.3)
project(gpio_messages)

## Find catkin macros and libraries
find_package(
	catkin REQUIRED COMPONENTS
		std_msgs
		message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    io_description.msg
    io_states.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#     FILES
#     GpioCommand.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpio_messages
    CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)