#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
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
driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import serial
import rospy
import roslib
import time
import threading

from math import sin, cos, pi
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from neato.msg import ButtonEvent, BumperEvent, Sensors
from sensor_msgs.msg import LaserScan, BatteryState

BASE_WIDTH = 248  # millimeters
MAX_SPEED = 300  # millimeters/second


class Neato:
    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        self.CMD_RATE = 2

        port = rospy.get_param('~port', "/dev/ttyACM0")
        rospy.loginfo("Using port: %s" % port)

        self.port = serial.Serial(port, 115200, timeout=0.1)

        if not self.port.isOpen():
            rospy.logerror("Failed To Open Serial Port")
            return

        rospy.loginfo("Open Serial Port %s ok" % port)

        # Storage for state tracking
        self.state = {"LSIDEBIT": False, "RSIDEBIT": False,
                      "LFRONTBIT": False, "RFRONTBIT": False,
                      "BTN_SOFT_KEY": False, "BTN_SCROLL_UP": False,
                      "BTN_START": False, "BTN_BACK": False,
                      "BTN_SCROLL_DOWN": False
                      }

        self.stop_state = True

        # turn things on
        self.comsData = []
        self.responseData = []
        self.currentResponse = []

        self.reading = False

        self.readLock = threading.RLock()
        self.readThread = threading.Thread(None, self.read)
        self.readThread.start()

        self.port.flushInput()
        self.sendCmd("\n\n\n")
        self.port.flushInput()

        self.setTestMode("on")
        self.setLDS("on")

        time.sleep(0.5)
        self.setLed("ledgreen")

        self.base_width = BASE_WIDTH
        self.max_speed = MAX_SPEED

        self.flush()

        # publishers and subscribers
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=10)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.batteryPub = rospy.Publisher(
            'sensor_msgs', BatteryState, queue_size=10)
        self.buttonEventPub = rospy.Publisher(
            'neato/button_event', ButtonEvent, queue_size=10)
        self.bumperEventPub = rospy.Publisher(
            'neato/bumper_event', BumperEvent, queue_size=10)
        self.sensorsPub = rospy.Publisher(
            'neato/sensors', Sensors, queue_size=10)

        self.odomBroadcaster = TransformBroadcaster()
        self.cmd_vel = [0, 0]
        self.old_vel = self.cmd_vel

    def spin(self):
        encoders = [0, 0]

        self.x = 0  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id', 'base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link))

        scan.angle_min = 0.0
        scan.angle_max = 359.0 * pi / 180.0
        scan.angle_increment = pi / 180.0
        scan.range_min = 0.020
        scan.range_max = 5.0

        odom = Odometry(header=rospy.Header(frame_id="odom"),
                        child_frame_id='base_footprint')

        self.setBacklight(1)
        self.setLED("Green")
        # main loop of driver
        r = rospy.Rate(20)
        cmd_rate = self.CMD_RATE

        while not rospy.is_shutdown():
            # notify if low batt
            # if self.getCharger() < 10:
            #    print "battery low " + str(self.getCharger()) + "%"
            # get motor encoder values
            left, right = self.getMotors()

            cmd_rate = cmd_rate - 1
            if cmd_rate == 0:
                # send updated movement commands
                # if self.cmd_vel != self.old_vel or self.cmd_vel == [0,0]:
                # max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
                # self.setMotors(self.cmd_vel[0], self.cmd_vel[1], (abs(self.cmd_vel[0])+abs(self.cmd_vel[1]))/2)
                self.setMotors(self.cmd_vel[0], self.cmd_vel[1],
                               max(abs(self.cmd_vel[0]), abs(self.cmd_vel[1])))
                cmd_rate = self.CMD_RATE

            self.old_vel = self.cmd_vel

            # prepare laser scan
            scan.header.stamp = rospy.Time.now()

            self.requestScan()
            scan.ranges, scan.intensities = self.getScanRanges()

            # now update position information
            dt = (scan.header.stamp - then).to_sec()
            then = scan.header.stamp

            d_left = (left - encoders[0]) / 1000.0
            d_right = (right - encoders[1]) / 1000.0
            encoders = [left, right]

            dx = (d_left + d_right) / 2
            dth = (d_right - d_left) / (self.base_width / 1000.0)

            x = cos(dth) * dx
            y = -sin(dth) * dx
            self.x += cos(self.th) * x - sin(self.th) * y
            self.y += sin(self.th) * x + cos(self.th) * y
            self.th += dth

            # prepare tf from base_link to odom
            quaternion = Quaternion()
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            # prepare odometry
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = dx / dt
            odom.twist.twist.angular.z = dth / dt

            # read sensors and data
            digitalSensors = self.getDigitalSensors()
            analogSensors = self.getAnalogSensors()
            chargerValues = self.getCharger()
            buttons = self.getButtons()

            # region Publish Bumper Events
            for i, b in enumerate(("LSIDEBIT", "RSIDEBIT",
                                   "LFRONTBIT", "RFRONTBIT",
                                   "LeftDropInMM", "RightDropInMM",
                                   "LeftMagSensor", "RightMagSensor")):
                if i < 4:
                    engaged = (digitalSensors[b] == '1')  # Bumper Switches
                elif i < 6:
                    # Optical Sensors (no drop: ~0-40)
                    engaged = (int(analogSensors[b]) > 50)
                else:
                    # Mag Sensors (no mag: 32768)
                    engaged = (int(analogSensors[b]) < 15000)
                if engaged != self.state[b]:
                    bumperEvent = BumperEvent()
                    bumperEvent.bumper = i
                    bumperEvent.engaged = engaged
                    self.bumperEventPub.publish(bumperEvent)

                self.state[b] = engaged
            # endregion Publish Bumper Info

            # region Publish Battery Info

            battery = BatteryState()
            # http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html

            power_supply_health = 1  # POWER_SUPPLY_HEALTH_GOOD
            if (chargerValues["BatteryOverTemp"] == "1"):
                power_supply_health = 2  # POWER_SUPPLY_HEALTH_OVERHEAT
            elif (chargerValues["EmptyFuel"] == '1'):
                power_supply_health = 3  # POWER_SUPPLY_HEALTH_DEAD
            elif (chargerValues["BatteryFailure"] == '1'):
                power_supply_health = 5  # POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

            power_supply_status = 3  # POWER_SUPPLY_STATUS_NOT_CHARGING
            if (chargerValues["ChargingActive"] == '1'):
                power_supply_status = 1  # POWER_SUPPLY_STATUS_CHARGING
            elif (chargerValues["FuelPercent"] == '100'):
                power_supply_status = 4  # POWER_SUPPLY_STATUS_FULL

            battery.voltage = int(analogSensors["BatteryVoltageInmV"]) // 1000
            # battery.temperature = int(analogSensors["BatteryTemp0InC"])
            battery.current = int(analogSensors["CurrentInmA"]) // 1000
            # battery.charge
            # battery.capacity
            # battery.design_capacity
            battery.percentage = int(chargerValues["FuelPercent"])
            battery.power_supply_status = power_supply_status
            battery.power_supply_health = power_supply_health
            battery.power_supply_technology = 1  # POWER_SUPPLY_TECHNOLOGY_NIMH
            battery.present = int(chargerValues['FuelPercent']) > 0
            # battery.cell_voltage
            # battery.cell_temperature
            # battery.location
            # battery.serial_number
            self.batteryPub.publish(battery)

            # endregion Publish Battery Info

            # region Publish Button Events

            for i, b in enumerate(("BTN_SOFT_KEY", "BTN_SCROLL_UP", "BTN_START",
                                   "BTN_BACK", "BTN_SCROLL_DOWN")):
                engaged = (buttons[b] == '1')
                if engaged != self.state[b]:
                    buttonEvent = ButtonEvent()
                    buttonEvent.button = i
                    buttonEvent.engaged = engaged
                    self.buttonEventPub.publish(buttonEvent)

                self.state[b] = engaged

            # endregion Publish Button Info

            # region Publish Sensors

            self.sensors = Sensors()

            # for i, s in enumerate(analogSensors):
            #    self.sensors[s] = analogSensors[s]

            # for i, s in enumerate(digitalSensors):
            #    self.sensors[s] = digitalSen

            # if you receive an error similar to "KeyError: 'XTemp0InC'" just comment out the value
            self.WallSensorInMM = analogSensors["WallSensorInMM"]
            self.sensors.BatteryVoltageInmV = analogSensors["BatteryVoltageInmV"]
            self.sensors.LeftDropInMM = analogSensors["LeftDropInMM"]
            self.sensors.RightDropInMM = analogSensors["RightDropInMM"]
            #self.sensors.XTemp0InC = analogSensors["XTemp0InC"]
            #self.sensors.XTemp1InC = analogSensors["XTemp1InC"]
            self.sensors.LeftMagSensor = analogSensors["LeftMagSensor"]
            self.sensors.RightMagSensor = analogSensors["RightMagSensor"]
            self.sensors.UIButtonInmV = analogSensors["UIButtonInmV"]
            self.sensors.VacuumCurrentInmA = analogSensors["VacuumCurrentInmA"]
            self.sensors.ChargeVoltInmV = analogSensors["ChargeVoltInmV"]
            self.sensors.BatteryTemp0InC = analogSensors["BatteryTemp0InC"]
            self.sensors.BatteryTemp1InC = analogSensors["BatteryTemp1InC"]
            self.sensors.CurrentInmA = analogSensors["CurrentInmA"]
            self.sensors.SideBrushCurrentInmA = analogSensors["SideBrushCurrentInmA"]
            self.sensors.VoltageReferenceInmV = analogSensors["VoltageReferenceInmV"]
            self.sensors.AccelXInmG = analogSensors["AccelXInmG"]
            self.sensors.AccelYInmG = analogSensors["AccelYInmG"]
            self.sensors.AccelZInmG = analogSensors["AccelZInmG"]

            self.sensors.SNSR_DC_JACK_CONNECT = digitalSensors["SNSR_DC_JACK_CONNECT"]
            self.sensors.SNSR_DUSTBIN_IS_IN = digitalSensors["SNSR_DUSTBIN_IS_IN"]
            self.sensors.SNSR_LEFT_WHEEL_EXTENDED = digitalSensors["SNSR_LEFT_WHEEL_EXTENDED"]
            self.sensors.SNSR_RIGHT_WHEEL_EXTENDED = digitalSensors["SNSR_RIGHT_WHEEL_EXTENDED"]
            self.sensors.LSIDEBIT = digitalSensors["LSIDEBIT"]
            self.sensors.LFRONTBIT = digitalSensors["LFRONTBIT"]
            self.sensors.RSIDEBIT = digitalSensors["RSIDEBIT"]
            self.sensors.RFRONTBIT = digitalSensors["RFRONTBIT"]
            self.sensorsPub.publish(self.sensors)

            # endregion Publish Sensors

            # region publish lidar and odom
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w), then,
                "base_footprint", "odom")
            self.scanPub.publish(scan)
            self.odomPub.publish(odom)

            # endregion publish lidar and odom

            # wait, then do it again
            r.sleep()

        # shut down
        self.setBacklight(0)
        self.setLED("Off")
        self.setLDS("off")
        self.setTestMode("off")

    def sign(self, a):
        if a >= 0:
            return 1
        else:
            return -1

    def cmdVelCb(self, req):
        x = req.linear.x * 1000
        th = req.angular.z * (self.base_width / 2)
        k = max(abs(x - th), abs(x + th))
        # sending commands higher than max speed will fail

        if k > self.max_speed:
            x = x * self.max_speed / k
            th = th * self.max_speed / k

        self.cmd_vel = [int(x - th), int(x + th)]

    def exit(self):
        self.setLDS("off")
        self.setLed("buttonoff")

        time.sleep(1)

        self.setTestMode("off")
        self.port.flush()

        self.reading = False
        self.readThread.join()

        self.port.close()

    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.sendCmd("testmode " + value)

    def setLDS(self, value):
        self.sendCmd("setldsrotation " + value)

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.sendCmd("getldsscan")

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        intensities = list()

        angle = 0

        if not self.readTo("AngleInDegrees"):
            self.flush()
            return ranges, intensities

        last = False
        while not last:  # angle < 360:
            try:
                vals, last = self.getResponse()
            except Exception as ex:
                rospy.logerr("Exception Reading Neato lidar: " + str(ex))
                last = True
                vals = []

            vals = vals.split(",")

            if ((not last) and ord(vals[0][0]) >= 48
                    and ord(vals[0][0]) <= 57):
                # print angle, vals
                try:
                    a = int(vals[0])
                    r = int(vals[1])
                    i = int(vals[2])
                    e = int(vals[3])

                    while (angle < a):
                        ranges.append(0)
                        intensities.append(0)
                        angle += 1

                    if (e == 0):
                        ranges.append(r / 1000.0)
                        intensities.append(i)
                    else:
                        ranges.append(0)
                        intensities.append(0)
                except:
                    ranges.append(0)
                    intensities.append(0)

                angle += 1

        if len(ranges) != 360:
            rospy.loginfo("Missing laser scans: got %d points" % len(ranges))

        return ranges, intensities

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        # This is a work-around for a bug in the Neato API. The bug is that the
        # robot won't stop instantly if a 0-velocity command is sent - the robot
        # could continue moving for up to a second. To work around this bug, the
        # first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        # the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False

        self.sendCmd("setmotor" + " lwheeldist " + str(int(l)) +
                     " rwheeldist " + str(int(r)) + " speed " + str(int(s)))

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """

        self.sendCmd("getmotors")

        if not self.readTo("Parameter"):
            self.flush()
            return [0, 0]

        last = False
        while not last:
            # for i in range(len(xv11_motor_info)):
            try:
                vals, last = self.getResponse()
                # print vals,last
                values = vals.split(",")
                self.state[values[0]] = float(values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato motors: " + str(ex))

        return [
            self.state["LeftWheel_PositionInMM"],
            self.state["RightWheel_PositionInMM"]
        ]

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """

        self.sendCmd("getanalogsensors")

        if not self.readTo("SensorName"):
            self.flush()
            return

        last = False
        analogSensors = {}
        while not last:  # for i in range(len(xv11_analog_sensors)):
            try:
                vals, last = self.getResponse()
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
                analogSensors[values[0]] = int(values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Analog sensors: " +
                             str(ex))

        return analogSensors

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """

        self.sendCmd("getdigitalsensors")

        if not self.readTo("Digital Sensor Name"):
            self.flush()
            return {}

        last = False
        digitalSensors = {}
        while not last:  # for i in range(len(xv11_digital_sensors)):
            try:
                vals, last = self.getResponse()
                # print vals
                values = vals.split(",")
                # print "Got Sensor: %s=%s" %(values[0],values[1])
                self.state[values[0]] = int(values[1])
                digitalSensors[values[0]] = int(values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Digital sensors: " +
                             str(ex))
        return digitalSensors

    def getButtons(self):
        """ Get button values from neato. """

        self.sendCmd("GetButtons")

        if not self.readTo("Button Name"):
            self.flush()
            return {}

        last = False
        buttons = {}
        while not last:
            vals, last = self.getResponse()
            values = vals.split(",")
            try:
                self.state[values[0]] = (values[1] == '1')
                buttons[values[0]] = (values[1] == '1')
            except Exception as ex:
                rospy.logerr("Exception Reading Neato button info: " +
                             str(ex))

        return buttons

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """

        self.sendCmd("getcharger")

        if not self.readTo("Label"):
            self.flush()
            return

        last = False
        chargerValues = {}
        while not last:  # for i in range(len(xv11_charger_info)):

            vals, last = self.getResponse()
            values = vals.split(",")
            try:
                if values[0] in ["VBattV", "VExtV"]:
                    # convert to millivolt to maintain as int and become mVBattV & mVExtV.
                    self.state['m' + values[0]] = int(float(values[1]) * 100)
                    chargerValues['m' + values[0]] = int(
                        float(values[1]) * 100)
                elif values[0] in ["BatteryOverTemp", "ChargingActive", "ChargingEnabled", "ConfidentOnFuel", "OnReservedFuel", "EmptyFuel", "BatteryFailure", "ExtPwrPresent"]:
                    # boolean values
                    self.state[values[0]] = (values[1] == '1')
                    chargerValues[values[0]] = (values[1] == '1')
                elif values[0] in ["FuelPercent", "MaxPWM", "PWM"]:
                    # int values
                    self.state[values[0]] = int(values[1])
                    chargerValues[values[0]] = int(values[1])
                # other values not supported.

            except Exception as ex:
                rospy.logerr("Exception Reading Neato charger info: " +
                             str(ex))
        return chargerValues

    def setBacklight(self, value):
        if value > 0:
            self.sendCmd("setled backlighton")
        else:
            self.sendCmd("setled backlightoff")

    def setLed(self, cmd):
        self.sendCmd("setled %s" % cmd)

    def setLED(self, cmd):
        self.setLed(cmd)

    def sendCmd(self, cmd):
        # rospy.loginfo("Sent command: %s"%cmd)
        self.port.write("%s\n" % cmd)

    def readTo(self, tag, timeout=1):
        try:
            line, last = self.getResponse(timeout)
        except:
            return False

        if line == "":
            return False

        while line.split(",")[0] != tag:
            try:
                line, last = self.getResponse(timeout)
                if line == "":
                    return False
            except:
                return False

        return True

    # thread to read data from the serial port
    # buffers each line in a list (self.comsData)
    # when an end of response (^Z) is read, adds the complete list of response lines to self.responseData and resets the comsData list for the next command response.
    def read(self):
        self.reading = True
        line = ""

        while (self.reading and not rospy.is_shutdown()):
            try:
                # read from serial 1 char at a time so we can parse each character
                val = self.port.read(1)
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Serial: " + str(ex))
                val = []

            if len(val) > 0:
                '''
                if ord(val[0]) < 32:
                    print("'%s'"% hex(ord(val[0])))
                else:
                    print ("'%s'"%str(val))
                '''

                if ord(val[0]) == 13:  # ignore the CRs
                    pass

                elif ord(val[0]) == 26:  # ^Z (end of response)
                    if len(line) > 0:
                        # add last line to response set if it is not empty
                        self.comsData.append(line)
                        # print("Got Last Line: %s" % line)
                        line = ""  # clear the line buffer for the next line

                    # print ("Got Last")
                    with self.readLock:  # got the end of the command response so add the full set of response data as a new item in self.responseData
                        self.responseData.append(list(self.comsData))

                    self.comsData = [
                    ]  # clear the bucket for the lines of the next command response

                # NL, terminate the current line and add it to the response data list (comsData) (if it is not a blank line)
                elif ord(val[0]) == 10:
                    if len(line) > 0:
                        self.comsData.append(line)
                        # print("Got Line: %s" % line)
                        line = ""  # clear the bufer for the next line
                else:
                    line = line + val  # add the character to the current line buffer

    # read response data for a command
    # returns tuple (line,last)
    # line is one complete line of text from the command response
    # last = true if the line was the last line of the response data (indicated by a ^Z from the neato)
    # returns the next line of data from the buffer.
    # if the line was the last line last = true
    # if no data is avaialable and we timeout returns line=""
    def getResponse(self, timeout=1):

        # if we don't have any data in currentResponse, wait for more data to come in (or timeout)
        while (len(self.currentResponse)
               == 0) and (not rospy.is_shutdown()) and timeout > 0:

            # pop a new response data list out of self.responseData (should contain all data lines returned for the last sent command)
            with self.readLock:
                if len(self.responseData) > 0:
                    self.currentResponse = self.responseData.pop(0)
                    # print "New Response Set"
                else:
                    self.currentResponse = []  # no data to get

            if len(self.currentResponse
                   ) == 0:  # nothing in the buffer so wait (or until timeout)
                time.sleep(0.010)
                timeout = timeout - 0.010

        # default to nothing to return
        line = ""
        last = False

        # if currentResponse has data pop the next line
        if not len(self.currentResponse) == 0:
            line = self.currentResponse.pop(0)
            # print line,len(self.currentResponse)
            if len(self.currentResponse) == 0:
                last = True  # if this was the last line in the response set the last flag
        else:
            print("Time Out")  # no data so must have timedout

        # rospy.loginfo("Got Response: %s, Last: %d" %(line,last))
        return (line, last)

    def flush(self):
        while (1):
            l, last = self.getResponse(1)
            if l == "":
                return


if __name__ == "__main__":
    robot = Neato()
    robot.spin()
