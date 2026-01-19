#!/usr/bin/env python3
"""
Simple node to broadcast TF from odometry for SO(3) simulator visualization
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomToTF:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # Use remappable topic name instead of hardcoded path
        odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.loginfo("[OdomToTF] Started - broadcasting TF from %s", odom_topic)

    def odom_callback(self, msg):
        """Broadcast TF transform from odometry"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"  # Match URDF base link name

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    odom_to_tf = OdomToTF()
    rospy.spin()
