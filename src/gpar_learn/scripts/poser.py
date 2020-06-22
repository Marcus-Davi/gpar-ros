#!/usr/bin/env python3
# import roslib
# roslib.load_manifest('gpar_learn')
import rospy
from geometry_msgs.msg import PointStamped

def talker():
    rospy.init_node("pose_publisher", anonymous=True)
    pub = rospy.Publisher('point', PointStamped , queue_size=10)
    rate = rospy.Rate(rospy.get_param('~freq')) # Freq
    while not rospy.is_shutdown():
        point  = PointStamped()
        point.header.stamp =  rospy.Time.now()
        pub.publish(point)
        rate.sleep()

if __name__=='__main__':
    print('print')
    try:
        talker()
    except rospy.ROSInternalException:
        pass
