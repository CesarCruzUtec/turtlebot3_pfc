#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import tf.transformations
import os
import matplotlib.pyplot as plt


class ControlTrapls:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.thetaR = 0.0
        self.k = 0.0
        self.S = np.zeros((3, 2))
        # self.init = True
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.Vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    def initPlot(self):
        self.fig, self.axs = plt.subplots(nrows=4, ncols=1, figsize=(6, 9))
        self.axs[0].set_xlim(-1, 5)
        self.axs[0].set_ylim(-2, 2)
        self.axs[0].set_xlabel("x(m)")
        self.axs[0].set_ylabel("y(m)")
        self.axs[0].grid(True)
        self.axs[1].set_ylabel("x(m)")
        self.axs[1].grid(True)
        self.axs[2].set_ylabel("y(m)")
        self.axs[2].grid(True)
        self.axs[3].set_ylabel("theta(rad)")
        self.axs[3].grid(True)
        self.axs[3].set_xlabel("time(s)")
        self.plotPos = [[], [], [], []]
        # tp, xp, yp, thetap = [], [], [], []

    def updatePlot(self, t_now, x, y, theta):
        self.plotPos[0].append(t_now)
        self.plotPos[1].append(x)
        self.plotPos[2].append(y)
        self.plotPos[3].append(theta)

        self.axs[0].plot(self.plotPos[1], self.plotPos[2], "b")

        self.axs[1].plot(self.plotPos[0], self.plotPos[1], "b")
        self.axs[2].plot(self.plotPos[0], self.plotPos[2], "b")
        self.axs[3].plot(self.plotPos[0], self.plotPos[3], "b")

        self.axs[1].get_shared_x_axes().join(self.axs[1], self.axs[2], self.axs[3])
        self.axs[1].set_xticklabels([])
        self.axs[2].set_xticklabels([])

        plt.draw()
        plt.pause(0.1)

    def rcf(self, x):
        y = np.sin(x)
        dy = np.cos(x)
        ddy = -np.sin(x)
        if ddy == 0:
            ddy = 1e-10
        rc = (1 + dy**2) ** (3 / 2) / ddy
        print("X: = {0:>6.4f}".format(x))
        print("Y: = {0:>6.4f}".format(y))
        print("Dy: = {0:>6.4f}".format(dy))
        print("Ddy: = {0:>6.4f}".format(ddy))
        print("Curvature: = {0:>6.4f}".format(rc))
        return rc

    def callback(self, data):
        # if self.init:
        #   self.init = False

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if (
            self.thetaR < np.pi
            and self.thetaR > 0.9 * np.pi
            and euler[2] > -np.pi
            and euler[2] < -0.9 * np.pi
        ):
            self.k = self.k + 1
        elif (
            self.thetaR > -np.pi
            and self.thetaR < -0.9 * np.pi
            and euler[2] < np.pi
            and euler[2] > 0.9 * np.pi
        ):
            self.k = self.k - 1
        self.thetaR = euler[2]
        self.theta = 2 * self.k * np.pi + self.thetaR
        self.S = np.array(
            [[np.cos(self.theta), 0.0], [np.sin(self.theta), 0.0], [0.0, 1.0]]
        )

    def run(self):
        rospy.init_node("controltrapls", disable_signals=True)
        rospy.Subscriber("/odom", Odometry, ct.callback)
        rate = rospy.Rate(10)
        self.initPlot()
        # pos_d = np.array([0.0, 0.0, 0.0])
        x = np.linspace(0, 5, 100)
        y = np.sin(x)
        self.axs[0].plot(x, y, "r--")
        vel = 0.26
        ang = 1.82
        init_pose = True
        try:
            while not rospy.is_shutdown():
                os.system("clear")
                self.pub.publish(self.Vel)

                if abs(1 - self.theta) > 0.01 and init_pose:
                    self.Vel.angular.z = 1 - self.theta
                else:
                    init_pose = False
                    rc = self.rcf(self.x)
                    lnvel = 1 + np.cos(self.x) ** 2
                    lnvel = np.clip(lnvel, -vel, vel)
                    self.Vel.linear.x = lnvel
                    self.Vel.angular.z = lnvel / rc
                    # print("Curvature: = {0:>6.4f}".format(rc))

                print("Velocity: linear = {0:>6.4f}".format(self.Vel.linear.x))
                print("Velocity: angular = {0:>6.4f}".format(self.Vel.angular.z))

                t_now = rospy.get_time()
                self.updatePlot(t_now, self.x, self.y, self.theta)
            rate.sleep()
        except KeyboardInterrupt:
            print("Stopped by user")
            self.Vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            self.pub.publish(self.Vel)
            plt.savefig("controltra.png")
            plt.close()


if __name__ == "__main__":
    ct = ControlTrapls()
    ct.run()
