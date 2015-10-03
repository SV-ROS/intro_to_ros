#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
 
import sys
 
import rospy
from beginner_tutorials.srv import *
 
def adc_to_distance_client(adc):
    rospy.wait_for_service('adc_to_distance')
    try:
        adc_dist = rospy.ServiceProxy('adc_to_distance', AdcToDistance)
        resp1 = adc_dist(adc)
        print resp1.inches
        print resp1.cm
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
 
def usage():
    return "%s [ x where 0 < x < 1023 ]" % sys.argv[0]
 
if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting to convert %d to distance" % x
    results=adc_to_distance_client(x)
    print "%d  = %3.3f in inches and %3.3f in cm " % (x, results.inches,results.cm)
