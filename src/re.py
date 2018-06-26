#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String



def b(msg):
	print msg.data
     #print msg.linear.x
     #print msg.linear.y
     #print msg.linear.z

def a(msg):
	print msg.linear.x
	print msg.linear.y
	print msg.linear.z

#rospy.Subscriber("test", Twist, a)

rospy.Subscriber("a", Twist, a)
rospy.Subscriber("b", String, b)

#rospy.Subscriber("teleop/increment/motor", String, a)

rospy.init_node("test")
rospy.spin()