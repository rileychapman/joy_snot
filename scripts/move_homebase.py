#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
import roscopter.msg

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand

class MCN():

    def __init__(self):
        rospy.init_node('update_homebase_location_node')
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.gps_status = 0
        self.sub_gps = rospy.Subscriber('/gps_beacon', NavSatFix, self.gps_beacon_callback)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            r.sleep()

    def gps_beacon_callback(self, data):
        #print data
        self.gps_status = data.status.status
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude
        print str(self.gps_status) + " : " + str(self.lat) + " : " + str(self.lon) + " : " + str(self.alt)



if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #TODO When killing from command line, have the copter disarm
