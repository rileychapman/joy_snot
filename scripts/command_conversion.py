#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from roscopter.msg import RC
from roscopter.srv import APMCommand

def conv(joystick_value):
	return (((joystick_value + 1)/2)*1000)+1000

def callback(data):
	copter = rospy.ServiceProxy('command', APMCommand)
	if data.buttons[2] == 1:
		copter(3)
		print "Arming"
	if data.buttons[3] == 1:
		copter(4)
		print "Disarming"

	rc_data = [conv(data.axes[0]), conv(data.axes[1]), conv(data.axes[3]), conv(data.axes[2]), 1000, 1000, 1000, 1000]
	print rc_data
	pub.publish(RC(rc_data))

def listener():
	#setup publisher to roscopter send_rc topic
	global pub
	pub = rospy.Publisher('send_rc', RC)
	#setup subscriber to joystick joy topic
	rospy.Subscriber('joy', Joy, callback)
	rospy.init_node('snotbot_teleop', anonymous=True)
	rospy.spin()

if __name__ == "__main__":
	listener()