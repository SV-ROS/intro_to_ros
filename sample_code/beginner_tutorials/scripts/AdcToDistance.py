#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_AdcToDistance(req):
    
    print "Returning distance values for analog value %d " % (req.adc)
    inches = (254.0/1024.0) *2.0* (req.adc/5)
    cm = inches * 2.54
    
    return AdcToDistanceResponse(inches,cm)

def AdcToDistance_server():
    rospy.init_node('AdcToDistance_server')
    s = rospy.Service('adc_to_distance', AdcToDistance, handle_AdcToDistance)
    print "Ready to convert adc values to distance"
    rospy.spin()

if __name__ == "__main__":
    AdcToDistance_server()

