import sys
import getopt

import rospy
import roslib; roslib.load_manifest('beginner_tutorials')
from std_msgs.msg import UInt16
from sensor_msgs.msg import Range

from beginner_tutorials.srv import *

class Sonar_Node(object):

   def __init__(self):

      rospy.init_node('sonar_node',anonymous=False)
      self.inchpub = rospy.Publisher('/ultrasound/inch',UInt16)
      self.cmpub =  rospy.Publisher('/ultrasound/cm',UInt16)
      rospy.on_shutdown(self.shutdown)
      
      robotrate=10
      r=rospy.Rate(robotrate)
       
      self.analog_range=0
  
   def shutdown(self):
      # Always stop the robot when shutting down the node.
      rospy.loginfo("Stopping the Node...")
      rospy.sleep(1)
     
 
   def sonarCb(self,Range):
      msg_str="SonarRange=%3.2f" % (Range.range)
      rospy.loginfo(rospy.get_name()+msg_str)
      self.analog_range = Range.range
   
   def listener(self):   
      rospy.Subscriber("ultrasound_fwd", Range, self.sonarCb)

   def calc_distance(self):

      rospy.wait_for_service('adc_to_distance')
      try:
         adc_dist = rospy.ServiceProxy('adc_to_distance', AdcToDistance)
         distance = adc_dist(self.analog_range)
         print distance.inches
         print distance.cm
         #self.inches=distance.inches
         #self.cm=distance.cm

         self.inchpub.publish(distance.inches)
         self.cmpub.publish(distance.cm) 

      except rospy.ServiceException, e:
         print "Service call failed: %s" % e

if __name__ == '__main__':

   sn=Sonar_Node() 
   sn.listener()

   r=rospy.Rate(10) 

   while not rospy.is_shutdown():
      sn.calc_distance()  
      r.sleep()
