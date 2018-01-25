cmake_minimum_required(VERSION 2.8.3)
project(face_detect_base)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


add_message_files(FILES ROIArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmp_name
   CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
