#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VehicleController:
    def __init__(self):
        rospy.init_node('vehicle_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.position = None

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position

    def move_straight(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = 1.0  # Adjust the desired linear speed
        twist_cmd.angular.z = 0.0

        while not rospy.is_shutdown():
            if self.position is not None:
                if self.position.x < 5.0:  # Adjust the desired distance
                    self.cmd_vel_pub.publish(twist_cmd)
                else:
                    self.cmd_vel_pub.publish(Twist())  # Stop the vehicle
                    break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = VehicleController ()
        controller.move_straight()
    except rospy.ROSInterruptException:
        pass
