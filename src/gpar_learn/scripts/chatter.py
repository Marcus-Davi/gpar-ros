#!/usr/bin/env python3
# import roslib
# roslib.load_manifest('gpar_learn')
import rospy
from std_msgs.msg import String

def talker():
    rospy.loginfo('chatter init ...')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10) # Freq
    while not rospy.is_shutdown():
        hello_str = "hello " + str(rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__=='__main__':
    print('print')
    try:
        talker()
    except rospy.ROSInternalException:
        pass
