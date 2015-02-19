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

class MCN():

    def __init__(self):
        rospy.init_node('main_control_node')
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
        self.x = 1500.0
        self.y = 1500.0
        self.z = 1000.0
        self.yaw = 1500
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons
        self.x = 1500-self.axes[0]*300
        self.y = 1500-self.axes[1]*300
        self.z = 1500+(self.axes[3])*500
	self.yaw = 1500-self.axes[2]*300

    def fly(self):

        if self.buttons:
            if self.buttons[2]:
                self.command_serv(3)
                print 'Arm Quad'
            if self.buttons[3]:
                self.command_serv(4)
                print 'Disarm Quad'

        (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
        self.pub_rc.publish(self.twist)

if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
