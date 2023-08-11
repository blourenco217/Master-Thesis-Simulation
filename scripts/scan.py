#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # print(len(msg.ranges))
    print(msg.ranges[int(len(msg.ranges)/2)])

rospy.init_node('scan_values')
sub = rospy.Subscriber('/mybot/laser_scan', LaserScan, callback)
rospy.spin()