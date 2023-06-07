#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class ControlPosition:
    def __init__(self, xd, yd, thetad, kp, kd, ki):
        self.posd_True = np.array([xd, yd, thetad])
        self.xd, self.yd, self.thetad = [], [], []
        self.x, self.y, self.theta = [], [], []
        self.xe, self.ye, self.thetae = [], [], []
        self.k = 0.0
        self.S = np.zeros((3, 2))
        self.kp = np.diag(kp)
        self.kd = np.diag(kd)
        self.ki = np.diag(ki)
        self.rate = 10
        self.newPath = True
        self.ierr_p = np.array([0.0, 0.0, 0.0])
        self.vmax = 0.26
        self.wmax = 1.82
        self.step = [True, True]

    def callback(self, data):
        self.x.append(data.pose.pose.position.x)
        self.y.append(data.pose.pose.position.y)
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if len(self.theta) == 0:
            self.theta.append(euler[2])
        else:
            if (euler[2] - self.theta[-1]) * 10 > 1.9:
                self.k = self.k + 1
            elif (euler[2] - self.theta[-1]) * 10 < -1.9:
                self.k = self.k - 1
            self.theta.append(2 * np.pi * self.k + euler[2])
        self.S = np.array(
            [[np.cos(self.theta[-1]), 0.0], [np.sin(self.theta[-1]), 0.0], [0.0, 1.0]]
        )

    def newDesiredPosition(self):
        err = self.posd_True - np.array([self.x[-1], self.y[-1], 0.0])
        theta_dp = np.arctan2(err[1], err[0]) + 2 * np.pi * self.k
        err[2] = theta_dp - self.theta[-1]
        if np.linalg.norm(err[0:2]) > 0.02 and self.step[0]:
            if abs(err[2]) > 0.05 and self.step[1]:
                self.pos_d = np.array([self.x[-1], self.y[-1], theta_dp])
                print("Rotating to next orientation")
            else:
                self.step[1] = False
                self.pos_d = np.array([self.posd_True[0], self.posd_True[1], theta_dp])
                print("Moving to next position")
        else:
            self.pos_d = self.posd_True
            self.step = [True, True]
            self.newPath = True

    def control(self):
        pos = np.array([self.x[-1], self.y[-1], self.theta[-1]])
        err = self.pos_d - pos
        if self.newPath:
            derr = np.array([0.0, 0.0, 0.0])
            ierr = np.array([0.0, 0.0, 0.0])
            self.ierr_p = np.array([0.0, 0.0, 0.0])
            self.newPath = False
        else:
            err_p = np.array([self.xe[-1], self.ye[-1], self.thetae[-1]])
            derr = (err - err_p) * self.rate
            ierr = self.ierr_p + err / self.rate
        self.xe.append(err[0]), self.ye.append(err[1]), self.thetae.append(err[2])
        dpos = np.dot(self.kp, err) + np.dot(self.kd, derr) + np.dot(self.ki, ierr)
        ik = np.dot(np.linalg.inv(self.S), dpos)
        self.vel.linear.x = np.clip(ik[0], -self.vmax, self.vmax)
        self.vel.angular.z = np.clip(ik[1], -self.wmax, self.wmax)

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(self.rate)
        self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        try:
            while not rospy.is_shutdown():
                pub.publish(self.vel)
                os.system("clear")

                self.newDesiredPosition()
                self.control()

                print("\n")
                rate.sleep()
        except KeyboardInterrupt:
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("controlpos", disable_signals=True)
    kp = [1, 1, 1]
    kd = [0.5, 0.5, 0.5]
    ki = [0.2, 0.2, 0.2]
    cp = ControlPosition(1.0, 1.0, 45.0 * np.pi / 180.0)
    cp.run()
