#!/usr/bin/env python3

import os

import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomReader:
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.k = 0.0

    def callback(self, data):
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if (euler[2] - self.theta) * 10 > 1.9:
            self.k = self.k + 1
        elif (euler[2] - self.theta) * 10 < -1.9:
            self.k = self.k - 1
        self.theta = 2 * self.k * np.pi + euler[2]
        self.vel_lin = data.twist.twist.linear.x
        self.vel_ang = data.twist.twist.angular.z

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                os.system("clear")
                print("X = %6.4f m, Y = %6.4f m" % (self.posx, self.posy))
                print("K = %6.4f" % (self.k))
                print("Theta = %6.4f rad" % (self.theta))
                print("ThetaR = %6.4f rad" % (self.thetaR))
                print("Vel_lin = %6.4f m/s" % (self.vel_lin))
                print("Vel_ang = %6.4f rad/s" % (self.vel_ang))
                print("\n")
                rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("odomreader", anonymous=True)
    odom_reader = OdomReader()
    odom_reader.run()
