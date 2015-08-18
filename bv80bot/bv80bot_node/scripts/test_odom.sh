#!/bin/bash

rostopic pub --once /cmd_vel geometry_msgs/Twist "[0.0,0,0]" "[0,0,3]"
rostopic pub --once /cmd_vel geometry_msgs/Twist "[0.0,0,0]" "[0,0,0]"

