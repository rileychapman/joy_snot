#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
import roscopter.msg
import matplotlib.pyplot as plt

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State

# This code should launch a quad- or hexa- copter into the air, approach a desired altitude set by the user, hover there for approximately 1 minute, then descend.  GPS and Altitude drift should be minimal. All control loops are self-designed.  Failsafe is controlled by human and puts it into auto-stabilized RC mode.

class MCN():
    def __init__(self):
        rospy.init_node('main_control_node')
        #set up control joystick
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500] #Setting to stabilized mode
        self.x = 1500.0 #Side Tilt
        self.y = 1500.0 #Front Tilt
        self.z = 1000.0 #Throttle
        self.yaw = 1500.0 #Spin
        #read in data of interest
        self.alt = 0.0
        self.plt_alt = []
        self.armed = False
        self.risen = False
        self.failsafe = False
        #set up communications protocol
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_state = rospy.Subscriber('/state', State, self.state_check)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_height = rospy.Subscriber('/vfr_hud', VFR_HUD, self.parse_action)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)

        #controller
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.hover()
            r.sleep()


    def joy_callback(self, data):
        #Reads in joystick controller positions and information
        self.axes = data.axes
        self.buttons = data.buttons
        self.x = 1500-self.axes[0]*300 #Scales 1200-1800
        self.y = 1500-self.axes[1]*300 #Scales 1200-1800
        self.z = 2000+(self.axes[3])*1000 #Scales 1000-3000
        self.yaw = 1500-self.axes[2]*300 #Scales 1200-1800

    def state_check(self, data):
        #Determines whether vehicle is armed and therefore able to receive commands
        self.armed = data.armed

    def parse_action(self, data):
        #Reads in data of interest from the vehicle
        #TODO Add GPS Data to determine drift
        self.airspeed = data.airspeed
        self.groundspeed = data.groundspeed
        self.heading = data.heading
        self.throttle = data.throttle
        self.alt = data.alt 
        self.plt_alt.append(self.alt) #for now only plotting altitude
        self.climb = data.climb

    def hover(self):
        #Reads in information about vehicle in order to determine action
        if self.buttons: #If button pressed
            if self.buttons[3]: #Disarm and plot data from flight
                self.command_serv(4)
                self.risen = False
                print 'Disarm Quad'
                plt.plot(self.plt_alt)
                plt.show()
            if self.buttons[2]: #Arm
                self.command_serv(3)
                print 'Arm Quad'
            if self.buttons[0]: #Failsafe
                self.command_serv(8)
                self.failsafe = True
                print 'Failsafe'
                rospy.sleep(5)

        if self.armed and not self.failsafe:
            if self.alt < 1.0 and not self.risen:
                print 'Rise'
                #Make it rise, uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1550, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt > 1.0: 
                #TODO Tune for Hovering here
                self.risen = True
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1450, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt > 0.2 and self.risen:
                print 'Lower'
                #Lower it, uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1450, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt <= 0.2 and self.risen:
                print 'Land'
                #Land itm uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1000, 1500)
                self.pub_rc.publish(self.twist)
                print 'Test Complete, Copter Disarming'
                self.command_serv(4)
                self.risen = False
        elif self.failsafe:
            self.command_serv(7) #Put in stabilized mode
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
        
        self.pub_rc.publish(self.twist)


if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #TODO Groundspeed/Airspeed control loop to have stable vertical flight
