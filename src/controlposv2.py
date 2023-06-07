#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class ControlPosition:
    def __init__(
        self,
        xd,
        yd,
        thetad,
        kp,
        kd=np.array([0.0, 0.0, 0.0]),
        ki=np.array([0.0, 0.0, 0.0]),
    ):
        self.xd = xd
        self.yd = yd
        self.thetad = thetad
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        self.k = 0.0
        self.S = np.zeros((3, 2))
        self.kp = np.diag(kp)
        self.kd = np.diag(kd)
        self.ki = np.diag(ki)

    def callback(self, data):
        self.x = self.x.append(data.pose.pose.position.x)
        self.y = self.y.append(data.pose.pose.position.y)
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if self.theta.size == 0:
            self.theta = self.theta.append(euler[2])
        else:
            if (euler[2] - self.theta[-1]) * 10 > 1.9:
                self.k = self.k + 1
            elif (euler[2] - self.theta[-1]) * 10 < -1.9:
                self.k = self.k - 1
            self.theta = self.theta.append(2 * self.k * np.pi + euler[2])
        self.S = np.array(
            [[np.cos(self.theta[-1]), 0.0], [np.sin(self.theta[-1]), 0.0], [0.0, 1.0]]
        )

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        vmax = 0.26
        wmax = 1.82
        try:
            while not rospy.is_shutdown():
                os.system("clear")

                print("\n")
                rate.sleep()
        except KeyboardInterrupt:
            self.velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.velocity)
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("controlpos", disable_signals=True)
    cp = ControlPosition(1.0, 1.0, 45.0 * np.pi / 180.0)
    cp.run()
