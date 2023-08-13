#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys  # Import the sys module

class MoveAndPrintPosition:
    def __init__(self, namespace, velocity):  # Accept namespace as an argument
        rospy.init_node('move_and_print_position', anonymous=True)
        self.pub = rospy.Publisher('/' + namespace + '/cmd_vel', Twist, queue_size=10)  # Use namespace in topic
        self.sub_odom = rospy.Subscriber('/' + namespace + '/odom', Odometry, self.odom_callback)  # Use namespace in topic
        self.rate = rospy.Rate(10)  # 10 Hz

        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = velocity   # Set the desired linear velocity
        self.twist_cmd.angular.z = 0.0  # No angular velocity

        self.position = (0.0, 0.0)  # Initialize the position

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def move_straight(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist_cmd)
            # rospy.loginfo("Current position: x = {:.2f}, y = {:.2f}".format(*self.position))
            self.rate.sleep()

if __name__ == '__main__':
    # if len(sys.argv) != 2:
    #     rospy.loginfo("Usage: {} <namespace>".format(sys.argv[0]))
    #     sys.exit(1)

    namespace = sys.argv[1]  # Get namespace from command line argument
    velocity = float(sys.argv[2])   # Get velocity from command line argument

    try:
        move_print = MoveAndPrintPosition(namespace, velocity)
        move_print.move_straight()
    except rospy.ROSInterruptException:
        pass

