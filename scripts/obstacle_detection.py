#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # Check if an obstacle is detected based on your criteria
    threshold = 10  # Adjust the desired threshold
    middle_index = len(msg.ranges) // 2
    if any(distance < threshold for distance in msg.ranges[middle_index-20:middle_index+20]):
        print('Obstacle Ahead Detected')

rospy.init_node('ellipse_drawer')
sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, callback)
rospy.spin()