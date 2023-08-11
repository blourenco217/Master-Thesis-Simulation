#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveAndPrintPosition:
    def __init__(self):
        rospy.init_node('move_and_print_position', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 3.0   # Set the desired linear velocity
        self.twist_cmd.angular.z = 0.0  # No angular velocity

        self.position = (0.0, 0.0)  # Initialize the position

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def move_straight(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist_cmd)
            rospy.loginfo("Current position: x = {:.2f}, y = {:.2f}".format(*self.position))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        move_print = MoveAndPrintPosition()
        move_print.move_straight()
    except rospy.ROSInterruptException:
        pass
