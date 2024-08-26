#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def read_csv(file_path):
    waypoints = []
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        
        for i, row in enumerate(reader):
            if len(row) >= 2:  # Ensure there are at least x and y coordinates
                x = float(row[0])
                y = float(row[1])
                z = float(row[2]) if len(row) > 2 else 0.0
                waypoints.append((x, y, z))
                if (i>=4000):
                	break
            	
    return waypoints

def create_path_message(waypoints):
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "world"  # Change the frame ID according to your setup

    for wp in waypoints:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"  # Use the same frame ID as the path
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        #pose.pose.position.z = wp[2]
        pose.pose.orientation.w = 1.0  # No rotation, just a simple orientation

        path.poses.append(pose)

    return path

def main():
    rospy.init_node('path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)

    file_path = 'wps.csv'  # Path to your CSV file

    waypoints = read_csv(file_path)
    rospy.loginfo(f"Loaded {len(waypoints)} waypoints")

    rate = rospy.Rate(1)  # 1 Hz
    #while not rospy.is_shutdown():
    path_msg = create_path_message(waypoints)
    path_pub.publish(path_msg)
    rate.sleep()
    path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
