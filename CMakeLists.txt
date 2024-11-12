cmake_minimum_required(VERSION 3.0.2)
project(circular_motion)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs simulator_utils)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(circular_mover src/circular_pub.cpp)
target_link_libraries(circular_mover ${catkin_LIBRARIES})
