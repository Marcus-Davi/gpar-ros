#!/usr/bin/env python3
import roslib
roslib.load_manifest('gpar_learn')
import rospy
import sys
from gpar_learn.msg import FloatStamped

def talker():
    pub = rospy.Publisher('~point', FloatStamped , queue_size=10)
    rate = rospy.Rate(rospy.get_param('~freq')) # Freq
    value = rospy.get_param('~value',default=0)
    rospy.loginfo('init value : %d',value)
    while not rospy.is_shutdown():
        point  = FloatStamped()
        point.header.stamp =  rospy.Time.now()
        point.data = value
        value = value+1
        pub.publish(point)
        rate.sleep()

if __name__=='__main__':
    try:
        rospy.init_node("pose_publisher", anonymous=True)
        talker()
    except rospy.ROSInternalException as e:
        pass
