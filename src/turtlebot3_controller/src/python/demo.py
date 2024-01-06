#!/usr/bin/env python
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Stabilization:
    def __init__(self, x_d=1.0, y_d=1.0):
        self.x_d = x_d
        self.y_d = y_d

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    @staticmethod
    def controller(x, y, theta, x_d, y_d):
        k = 1.0
        l = 0.2
        e_x = x_d - x
        e_y = y_d - y
        u_x = k * e_x
        u_y = k * e_y
        A = np.array(
            [
                [np.cos(theta), -l * np.sin(theta)],
                [np.sin(theta), l * np.cos(theta)],
            ]
        )
        U = np.array([[u_x], [u_y]])
        v_w = np.linalg.solve(A, U)
        v = v_w[0]
        w = v_w[1]

        return v, w

    def timer_cb(self, data):
        v, w = self.controller(self.x, self.y, self.theta, self.x_d, self.y_d)

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w

        self.vel_pub.publish(vel)

    def odom_cb(self, data):
        posistion = data.pose.pose.position
        self.x = posistion.x
        self.y = posistion.y

        oriention = data.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([oriention.x, oriention.y, oriention.z, oriention.w])

        info = "(x, y, theta) = ({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.theta)
        rospy.loginfo(info)


def main(args):
    rospy.init_node("stabilization_demo")
    stabilization = Stabilization(1.0, 1.0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
