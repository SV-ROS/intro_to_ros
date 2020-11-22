cmake_minimum_required(VERSION 2.8.3)
project(neato)

find_package(catkin REQUIRED message_generation)

add_message_files(
   FILES
   BumperEvent.msg
   ButtonEvent.msg
   Sensors.msg
 )

generate_messages(
  DEPENDENCIES
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#catkin_package()
#catkin_python_setup()




