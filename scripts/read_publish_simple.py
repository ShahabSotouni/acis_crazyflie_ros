#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('crazyflie', PoseStamped, queue_size=100)
    rospy.init_node('cf_data_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.get_time()
        msg.header.frame_id = "cf1/lh"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        
        info_str = "cf1: %s" % rospy.get_time()
        rospy.loginfo(info_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
