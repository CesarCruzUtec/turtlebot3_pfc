#!/usr/bin/env python3

import os

import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import datetime


class OdomReader:
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.current_time = datetime.datetime.now().strftime("%H-%M-%S")
        self.file_name = "data/randomMov_" + self.current_time + ".txt"

    def callback(self, data):
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        self.theta = euler[2]
        self.vel_lin = data.twist.twist.linear.x
        self.vel_ang = data.twist.twist.angular.z

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                os.system("clear")
                print("X = %6.4f m, Y = %6.4f m" % (self.posx, self.posy))
                print("Vel_lin = %6.4f m/s" % (self.vel_lin))
                print("Vel_ang = %6.4f rad/s" % (self.vel_ang))
                with open(self.file_name, "a") as f:
                    f.write(
                        "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n"
                        % (
                            rospy.get_time(),
                            self.posx,
                            self.posy,
                            self.theta,
                            self.vel_lin,
                            self.vel_ang,
                        )
                    )
                rate.sleep()
        except KeyboardInterrupt:
            os.system("clear")
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("odomreader", anonymous=True)
    odom_reader = OdomReader()
    odom_reader.run()
