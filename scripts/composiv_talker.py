#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('chatter',String,queue_size=10) #create publisher
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10) #10hz

	while( not rospy.is_shutdown()):
		killBill_str="That really was a hattori hanzo sword %s" % rospy.get_time()
		rospy.loginfo(killBill_str)
		pub.publish(killBill_str) #published the message
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterreptException:
		pass
