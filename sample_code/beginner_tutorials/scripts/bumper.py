#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
#
# 2017 Alan Federman SV-ROS
#
# A simple listener for the SV_ROS (from neato_node) the listens to see if 
# the bumper has been pressed on a Botvac.
#
# This code can be incuded in navigation scripts to halt or turn the robot if 
# the bumper has been pressed
#
# the values are Left_Bumper Right_Bumper Left_Side_Bumper Right_Side_Bumper
#
# to use install in catkin_ws in a node subdirectory on your laptop for example
# /catkin_ws/src/test/src  and rosrun test bumper.py -- of course set the master
# and launch the base on the Botvac.
#
import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos,pi

from neato_node.msg import Button, Sensor

class NeatoNode:
	
    def __init__(self):

        self.sensorPub = rospy.Publisher('sensor', Sensor, queue_size=10)
        sensor = Sensor()

def callback(sensor):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s',sensor.name)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('sensor', Sensor, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
