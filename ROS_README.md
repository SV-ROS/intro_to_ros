# Notes on using ROS with this package

rostopic echo --filter "m.name=='Wall_Distance'" /sensorAnalog

rostopic /move_base_simple/goal

roslaunch neato base_map.launch

roslaunch neato base_nav.launch