#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from matplotlib.gridspec import GridSpec
from nav_msgs.msg import Odometry


class ControlPosition:
    def __init__(self, xd, yd, thetad, kp, ki, kd):
        self.posd_True = np.array([xd, yd, thetad])
        self.xd, self.yd, self.thetad = [], [], []
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.xp, self.yp, self.thetap = [], [], []
        self.xe, self.ye, self.thetae = [], [], []
        self.time = []
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
        self.finished = False
        self.pos_d = np.array([0.0, 0.0, 0.0])

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if (euler[2] - self.theta) * 10 > 1.9:
            self.k = self.k + 1
        elif (euler[2] - self.theta) * 10 < -1.9:
            self.k = self.k - 1
        self.theta = 2 * np.pi * self.k + euler[2]
        self.S = np.array(
            [[np.cos(self.theta), 0.0], [np.sin(self.theta), 0.0], [0.0, 1.0]]
        )

    def plot_init(self):
        self.fig = plt.figure(figsize=(12, 6))
        gs1 = GridSpec(3, 2, wspace=0.15, hspace=0.1)
        self.ax1 = self.fig.add_subplot(gs1[:, 0])
        self.ax2 = self.fig.add_subplot(gs1[0, 1])
        self.ax3 = self.fig.add_subplot(gs1[1, 1], sharex=self.ax2)
        self.ax4 = self.fig.add_subplot(gs1[2, 1], sharex=self.ax2)
        self.ax1.set_xlim(-1.5, 1.5)
        self.ax1.set_ylim(-1.5, 1.5)
        self.ax1.grid()
        self.ax2.grid()
        self.ax3.grid()
        self.ax4.grid()
        self.ax1.set_xlabel("x(m)")
        self.ax1.set_ylabel("y(m)")
        self.ax2.set_ylabel("x(m)")
        self.ax3.set_ylabel("y(m)")
        self.ax4.set_ylabel("theta(rad)")
        self.ax4.set_xlabel("time(s)")
        self.ax2.tick_params(labelbottom=False)
        self.ax3.tick_params(labelbottom=False)

        self.ax1.plot(self.posd_True[0], self.posd_True[1], "ro", markersize=2)
        plt.draw()
        plt.pause(0.001)

    def update_plot(self):
        self.time.append(rospy.get_time() - self.initialTime)
        self.xp.append(self.x)
        self.yp.append(self.y)
        self.thetap.append(self.theta)

        self.ax1.plot(self.xp, self.yp, "b")
        self.ax2.plot(self.time, self.xp, "b")
        self.ax2.plot(self.time, self.xd, "r")
        self.ax2.plot(self.time, self.xe, "g")
        self.ax3.plot(self.time, self.yp, "b")
        self.ax3.plot(self.time, self.yd, "r")
        self.ax3.plot(self.time, self.ye, "g")
        self.ax4.plot(self.time, self.thetap, "b")
        self.ax4.plot(self.time, self.thetad, "r")
        self.ax4.plot(self.time, self.thetae, "g")

        plt.draw()
        plt.pause(0.001)

    def printStatus(self):
        print("Current position: ")
        print("    x = {:>6.4f}".format(self.xp[-1]))
        print("    y = {:>6.4f}".format(self.yp[-1]))
        print("theta = {:>6.4f}".format(self.thetap[-1] * 180 / np.pi))
        print("Desired position: ")
        print("    x = {:>6.4f}".format(self.xd[-1]))
        print("    y = {:>6.4f}".format(self.yd[-1]))
        print("theta = {:>6.4f}".format(self.thetad[-1] * 180 / np.pi))
        print("Error: ")
        print("    x = {:>6.4f}".format(self.xe[-1]))
        print("    y = {:>6.4f}".format(self.ye[-1]))
        print("theta = {:>6.4f}".format(self.thetae[-1] * 180 / np.pi))

    def newDesiredPosition(self):
        err = self.posd_True - np.array([self.x, self.y, 0.0])
        theta_dp = np.arctan2(err[1], err[0]) + 2 * np.pi * self.k
        if np.linalg.norm(err[0:2]) > 0.02 and self.step[0]:
            err[2] = theta_dp - self.theta
            if abs(err[2]) > 0.01 and self.step[1]:
                self.pos_d = np.array([self.x, self.y, theta_dp])
                print("Rotating to next orientation")
            else:
                self.step[1] = False
                self.pos_d = np.array([self.posd_True[0], self.posd_True[1], theta_dp])
                print("Moving to next position")
        else:
            self.step[0] = False
            err[2] = self.posd_True[2] - self.theta + 2 * np.pi * self.k
            if abs(err[2]) > 0.01 and not self.step[1]:
                self.pos_d = self.posd_True
                print("Rotating to final orientation")
            else:
                self.step[1] = True
                self.finished = True
                print("Final position reached")
                plt.savefig("controlPos.png")
                plt.close()

        self.xd.append(self.pos_d[0])
        self.yd.append(self.pos_d[1])
        self.thetad.append(self.pos_d[2])

    def control(self):
        pos = np.array([self.x, self.y, self.theta])
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
            self.ierr_p = ierr
        self.xe.append(err[0])
        self.ye.append(err[1])
        self.thetae.append(err[2])
        dpos = np.dot(self.kp, err) + np.dot(self.kd, derr) + np.dot(self.ki, ierr)
        ik = np.dot(np.linalg.pinv(self.S), dpos)
        self.vel.linear.x = np.clip(ik[0], -self.vmax, self.vmax)
        self.vel.angular.z = np.clip(ik[1], -self.wmax, self.wmax)

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(self.rate)
        self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        rospy.wait_for_message("/odom", Odometry)
        self.plot_init()
        self.initialTime = rospy.get_time()

        try:
            while not rospy.is_shutdown() and not self.finished:
                pub.publish(self.vel)
                os.system("clear")

                self.newDesiredPosition()
                self.control()
                self.update_plot()
                self.printStatus()

                print("\n")
                rate.sleep()
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
        except KeyboardInterrupt:
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
            plt.savefig("controlPos.png")
            plt.close()
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("controlpos", disable_signals=True)
    kp = [2, 2, 2]
    kd = [0.3, 0.3, 0.25]
    ki = [0.5, 0.5, 0.5]
    cp = ControlPosition(0.0, 0.0, 0.0, kp, ki, kd)
    cp.run()
