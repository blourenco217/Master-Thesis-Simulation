#!/usr/bin/env python
import rospy
from avoidance.obstacle_extraction import ObstacleExtraction
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def scan_callback(msg):
    obstacle_extraction.scan_callback(msg)

def odometry_callback(msg):
        # self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        obstacle_extraction.ego_pose = (position.x, position.y, euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])

if __name__ == '__main__':
    try:
        rospy.init_node('debug_lidar')
        scan_sub = rospy.Subscriber('/ego_vehicle/laser_scan', LaserScan, scan_callback)
        odom_sub = rospy.Subscriber('/ego_vehicle/odom', Odometry, odometry_callback)
        obstacle_extraction = ObstacleExtraction()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


