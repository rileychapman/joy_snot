#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from roscopter.msg import RC
from roscopter.srv import APMCommand
import time

def conv(joystick_value):
	#Linear control
	return (((joystick_value + 1)/2)*1000)+1000

def callback(data):
	#Be sure physical safety switch is engaged.  If no RC commands are sent after arming within 5-10 seconds, the copter will automatically disarm
	copter = rospy.ServiceProxy('command', APMCommand)
	if data.buttons[2] == 1:
		copter(3)
		print "Arming"
		time.sleep(1)
	if data.buttons[3] == 1:
		copter(4)
		print "Disarming"
		time.sleep(1)

	rc_data = [conv(data.axes[0]), conv(data.axes[1]), conv(data.axes[3]), conv(data.axes[2]), 65535, 65535, 65535, 65535]
	print rc_data
	pub.publish(RC(rc_data))

def listener():
	#setup publisher to roscopter send_rc topic
	global pub
	pub = rospy.Publisher('send_rc', RC, queue_size=10)
	#setup subscriber to joystick joy topic
	rospy.Subscriber('joy', Joy, callback)
	rospy.init_node('snotbot_teleop', anonymous=True)
	rospy.spin()

if __name__ == "__main__":
	try:
		listener()
	except rospy.ROSInterruptException:
		pass