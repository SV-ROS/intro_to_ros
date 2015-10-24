#!/usr/bin/env python

# ROS node for the Neato Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for Neato robot vacuums.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos,pi

from sensor_msgs.msg import LaserScan
from neato_node.msg import Button, Sensor
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from neato_driver.neato_driver import Botvac

class NeatoNode:
	
    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        self.CMD_RATE =2 

        self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        rospy.loginfo("Using port: %s" % self.port)

        self.robot = Botvac(self.port)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=10)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.buttonPub = rospy.Publisher('button', Button, queue_size=10)
        self.sensorPub = rospy.Publisher('sensor', Sensor, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        self.cmd_vel = [0, 0]
        self.old_vel = self.cmd_vel

    def spin(self):
        encoders = [0, 0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id', 'base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link))

        scan.angle_min =0.0 
        scan.angle_max =359.0*pi/180.0 
        scan.angle_increment =pi/180.0 
        scan.range_min = 0.020
        scan.range_max = 5.0

        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')

        button = Button()
        sensor = Sensor()
        self.robot.setBacklight(1)
        self.robot.setLED("Green")
        # main loop of driver
        r = rospy.Rate(20)
        cmd_rate= self.CMD_RATE

        while not rospy.is_shutdown():
            # notify if low batt
            #if self.robot.getCharger() < 10:
            #    print "battery low " + str(self.robot.getCharger()) + "%"
            # get motor encoder values
            left, right = self.robot.getMotors()

            cmd_rate = cmd_rate-1
            if cmd_rate ==0:
		    # send updated movement commands
		    #if self.cmd_vel != self.old_vel or self.cmd_vel == [0,0]:
                    # max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
		    self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], (abs(self.cmd_vel[0])+abs(self.cmd_vel[1]))/2)
		    cmd_rate = self.CMD_RATE

            self.old_vel = self.cmd_vel

            # prepare laser scan
            scan.header.stamp = rospy.Time.now()
           
            self.robot.requestScan()
            scan.ranges = self.robot.getScanRanges()

            # now update position information
            dt = (scan.header.stamp - then).to_sec()
            then = scan.header.stamp

            d_left =  (left - encoders[0])/1000.0
            d_right =  (right - encoders[1])/1000.0
            encoders = [left, right]

	    #print d_left, d_right, encoders

            dx = (d_left+d_right)/2
            dth = (d_right-d_left)/(self.robot.base_width/1000.0)

            x = cos(dth)*dx
            y = -sin(dth)*dx
            self.x += cos(self.th)*x - sin(self.th)*y
            self.y += sin(self.th)*x + cos(self.th)*y
            self.th += dth
            #print self.x,self.y,self.th

            # prepare tf from base_link to odom
            quaternion = Quaternion()
            quaternion.z = sin(self.th/2.0)
            quaternion.w = cos(self.th/2.0)

            # prepare odometry
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = dx/dt
            odom.twist.twist.angular.z = dth/dt


            # sensors
            lsb, rsb, lfb, rfb = self.robot.getDigitalSensors()

            # buttons
            btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down = self.robot.getButtons()


            # publish everything
            self.odomBroadcaster.sendTransform((self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z,
                                                                     quaternion.w), then, "base_footprint", "odom")
            self.scanPub.publish(scan)
            self.odomPub.publish(odom)
            button_enum = ("Soft_Button", "Up_Button", "Start_Button", "Back_Button", "Down_Button")
            sensor_enum = ("Left_Side_Bumper", "Right_Side_Bumper", "Left_Bumper", "Right_Bumper")
            for idx, b in enumerate((btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down)):
                if b == 1:
                    button.value = b
                    button.name = button_enum[idx]
                    self.buttonPub.publish(button)

            for idx, b in enumerate((lsb, rsb, lfb, rfb)):
                if b == 1:
                    sensor.value = b
                    sensor.name = sensor_enum[idx]
                    self.sensorPub.publish(sensor)
            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.setBacklight(0)
        self.robot.setLED("Off")
        self.robot.setLDS("off")
        self.robot.setTestMode("off")

    def sign(self,a):
        if a>=0:
		return 1
	else:
		return-1

    def cmdVelCb(self,req):
        x = req.linear.x * 1000
        th = req.angular.z * (self.robot.base_width/2)
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail

        if k > self.robot.max_speed:
            x = x*self.robot.max_speed/k; th = th*self.robot.max_speed/k

        self.cmd_vel = [int(x-th), int(x+th)]

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

