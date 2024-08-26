#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class WorldToBaseFootprintTF:
    def __init__(self):
        rospy.init_node('world_to_base_footprint_tf')

        # Create a transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Subscribe to the odometry topic
        self.odom_sub = rospy.Subscriber('/gem/base_footprint/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_footprint"

        # Set the translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0  # Assuming the robot is on a 2D plane

        # Set the rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Send the transform
        self.br.sendTransform(t)

if __name__ == '__main__':
    try:
        WorldToBaseFootprintTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
