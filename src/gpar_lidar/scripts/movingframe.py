#!/bin/env python3
import roslib
roslib.load_manifest('gpar_lidar')
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2 as pc2
import geometry_msgs.msg
from std_msgs.msg import String
def cloud_callback(data):
    pass
#    rospy.loginfo("laser callback")

if __name__=='__main__':
    rospy.init_node('movingframe_node')
    speed = rospy.get_param("~frame_speed",'0.1')
    print(speed)
    rospy.Subscriber("/cloud",pc2,cloud_callback)
    pub_msg = rospy.Publisher("/simple_cloud_combinator/msgs",String,queue_size = 10)
    speed_per_sampling = float(speed)/100.0;
    rate = rospy.Rate(100); # 100 hz
  
    t = geometry_msgs.msg.TransformStamped() 
    br = tf2_ros.TransformBroadcaster()
    z = 0.0
    while not rospy.is_shutdown():
        if z > 2:
           z = 0
           pub_msg.publish('reset')
        z = z+speed_per_sampling;
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "moving_frame"
        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = z;

        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w = 1;
        br.sendTransform(t)
        rate.sleep()

