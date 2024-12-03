cmake_minimum_required(VERSION 3.0.2)
project(circular_motion)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs simulator_utils message_generation)

add_service_files(
    FILES
    synchronize.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sync_node src/synchronize.cpp)
target_link_libraries(sync_node ${catkin_LIBRARIES})
add_dependencies(sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(circular_mover src/circular_pub.cpp src/geometry_helpers.cpp src/pointList.cpp)
target_link_libraries(circular_mover ${catkin_LIBRARIES})
