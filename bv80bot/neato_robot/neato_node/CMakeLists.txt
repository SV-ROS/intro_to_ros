cmake_minimum_required(VERSION 2.8.3)
project(neato_node)

find_package(catkin REQUIRED message_generation)

add_message_files(
   FILES
   Button.msg
   Sensor.msg
 )
 
generate_messages(
  DEPENDENCIES
)

catkin_package(DEPENDS message_runtime )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
