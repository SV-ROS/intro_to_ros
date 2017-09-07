#!/usr/bin/env python
# license removed for brevity
import roslib; roslib.load_manifest("neato_node")
import rospy
from neato_node.msg import Button, Sensor
from sensor_msgs.msg import LaserScan

class NeatoNode:

   def __init__(self):
       self.default_frame_id = '/base_laser_link'
       self.sequence = 1
       self.pub = rospy.Publisher('scan', LaserScan, queue_size=1)
       self.sensorPub = rospy.Publisher('sensor', Sensor, queue_size=10)
       sensor = Sensor()

   def callback(self, sensor):
        #rospy.loginfo(rospy.get_caller_id() + ' I heard %s',sensor.name)

       laserscan = LaserScan()
       laserscan.header.stamp = rospy.Time.now()
       laserscan.header.seq = self.sequence
       laserscan.header.frame_id = self.default_frame_id
       self.sequence = self.sequence + 1
       if (sensor.name == "Left_Side_Bumper"):
           laserscan.angle_min = 0.6
           laserscan.angle_max = 1
           laserscan.range_min = 0.0
           laserscan.range_max = 0.3
           laserscan.ranges = [0.29, 0.29]
       elif (sensor.name == "Left_Bumper"):
           laserscan.angle_min = 0.6
           laserscan.angle_max = 1
           laserscan.range_min = 0.0
           laserscan.range_max = 0.3
           laserscan.ranges = [0.29, 0.29]
       elif (sensor.name == "Right_Bumper"):
           laserscan.angle_min = -0.6
           laserscan.angle_max = 0
           laserscan.range_min = 0.0
           laserscan.range_max = 0.3
           laserscan.ranges = [0.29, 0.29]
       elif (sensor.name == "Right_Side_Bumper"):
           laserscan.angle_min = -0.6
           laserscan.angle_max = 0
           laserscan.range_min = 0.0
           laserscan.range_max = 0.3
           laserscan.ranges = [0.29, 0.29]

       #rospy.loginfo(laserscan)
       self.pub.publish(laserscan)

   def listener(self):
       rospy.init_node('bumperToLaser', anonymous=True)
       rospy.loginfo(rospy.get_caller_id() + 'listener start')
       # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
       rospy.loginfo(rospy.get_caller_id() + 'listener init_node')

       rospy.Subscriber('sensor', Sensor, self.callback)
       rospy.loginfo(rospy.get_caller_id() + 'listener set subscriber')

       # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()

if __name__ == '__main__':
   print("main start")
   node = NeatoNode()
   node.listener()
####


