#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
import roscopter.msg

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State

class MCN():

    def __init__(self):
        rospy.init_node('main_control_node')
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 2000, 1500, 1500, 1500] #Setting to stabilized mode
        self.x = 1500.0 #Side Tilt
        self.y = 1500.0 #Front Tilt
        self.z = 1000.0 #Throttle
        self.yaw = 1500.0 #Spin
        self.alt = 0.0
        self.armed = False
        self.risen = False
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_state = rospy.Subscriber('/state', State, self.state_check)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_height = rospy.Subscriber('/vfr_hud', VFR_HUD, self.parse_action)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.hover()
            r.sleep()


    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons
        #self.x = 1500-self.axes[0]*300 #Scales 1200-1800
        #self.y = 1500-self.axes[1]*300 #Scales 1200-1800
        #self.z = 2000+(self.axes[3])*1000 #Scales 1000-3000
        #self.yaw = 1500-self.axes[2]*300 #Scales 1200-1800

    def state_check(self, data):
        self.armed = data.armed

    def parse_action(self, data):
        self.airspeed = data.airspeed
        self.groundspeed = data.groundspeed
        self.heading = data.heading
        self.throttle = data.throttle
        self.alt = data.alt 
        self.climb = data.climb

    def hover(self):

        if self.buttons:
            if self.buttons[3]:
                self.command_serv(4)
                self.risen = False
                print 'Disarm Quad'
            if self.buttons[2]:
                self.command_serv(3)
                print 'Arm Quad'

        if self.armed:
            if self.alt < 1.0 and not self.risen:
                print 'Rise'
                #Make it rise
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), 1550, int(self.yaw))
                self.pub_rc.publish(self.twist)
            elif self.alt > 1.0: 
                self.risen = True
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), 1450, int(self.yaw))
                self.pub_rc.publish(self.twist)
            elif self.alt > 0.2 and self.risen:
                print 'Lower'
                #Lower it
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), 1450, int(self.yaw))
                self.pub_rc.publish(self.twist)
            elif self.alt <= 0.2 and self.risen:
                print 'Land'
                #Land it
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), 1000, int(self.yaw))
                self.pub_rc.publish(self.twist)

                print 'Test Complete, Copter Disarming'
                self.command_serv(4)
                self.risen = False


if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #TODO When killing from command line, have the copter disarm
    #TODO Groundspeed/Airspeed control loop to have stable vertical flight
