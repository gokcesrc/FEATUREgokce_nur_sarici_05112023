#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + ' I heard %s',data.data)

def listener():
	rospy.init_node('listener',anonymous=True) #listener node
	rospy.Subscriber('chatter',String,callback) #subject to listen
	rospy.spin()

if __name__ == '__main__':
	listener()
