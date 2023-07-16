#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from matplotlib.gridspec import GridSpec
from nav_msgs.msg import Odometry


class ControlTrajectory:
    def __init__(self, xd, yd, thetad, kp, ki, kd):
        self.x_True = xd
        self.y_True = yd
        self.theta_True = thetad
        self.thetaP = 0.0
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.xd, self.yd, self.thetad = [], [], []
        self.xp, self.yp, self.thetap = [], [], []
        self.xe, self.ye, self.thetae = [], [], []
        self.xei, self.yei, self.thetaei = [], [], []
        self.xed, self.yed, self.thetaed = [], [], []
        self.linear_vel, self.angular_vel = [], []
        self.vel_lin, self.vel_ang = 0.0, 0.0
        self.lin_real, self.ang_real = [], []
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
        self.iterator = 0
        self.past_iterator = -1
        self.finished = False
        self.status = "Not started"
        self.pos_d = np.array([0.0, 0.0, 0.0])

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        if (euler[2] - self.thetaP) > 1.9*np.pi:
            self.k = self.k + 1
        elif (euler[2] - self.thetaP) < -1.9*np.pi:
            self.k = self.k - 1
        self.theta = euler[2]
        # self.theta = 2 * np.pi * self.k + self.thetaP
        self.S = np.array(
            [[np.cos(self.theta), 0.0], [np.sin(self.theta), 0.0], [0.0, 1.0]]
        )
        self.vel_lin = data.twist.twist.linear.x
        self.vel_ang = data.twist.twist.angular.z
        

    def plot_init(self):
        self.fig = plt.figure(figsize=(12, 6))
        gs1 = GridSpec(3, 2, wspace=0.15, hspace=0.1)
        self.ax1 = self.fig.add_subplot(gs1[:, 0])
        self.ax2 = self.fig.add_subplot(gs1[0, 1])
        self.ax3 = self.fig.add_subplot(gs1[1, 1], sharex=self.ax2)
        self.ax4 = self.fig.add_subplot(gs1[2, 1], sharex=self.ax2)
        self.ax1.set_xlim(-5, 1)
        self.ax1.set_ylim(-2, 5)
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

        self.ax1.plot(self.x_True, self.y_True, "r--")
        plt.draw()
        plt.pause(0.01)

    def update_plot(self):
        self.time.append(rospy.get_time() - self.initialTime)
        self.xp.append(self.x)
        self.yp.append(self.y)
        self.thetap.append(self.theta)
        self.lin_real.append(self.vel_lin)
        self.ang_real.append(self.vel_ang)

        self.ax1.plot(self.xp, self.yp, "b", linewidth=0.5)
        self.ax2.plot(self.time, self.xp, "b", linewidth=0.5)
        self.ax2.plot(self.time, self.xd, "r", linewidth=0.5)
        self.ax2.plot(self.time, self.xe, "g", linewidth=0.5)
        self.ax3.plot(self.time, self.yp, "b", linewidth=0.5)
        self.ax3.plot(self.time, self.yd, "r", linewidth=0.5)
        self.ax3.plot(self.time, self.ye, "g", linewidth=0.5)
        self.ax4.plot(self.time, self.thetap, "b", linewidth=0.5)
        self.ax4.plot(self.time, self.thetad, "r", linewidth=0.5)
        self.ax4.plot(self.time, self.thetae, "g", linewidth=0.5)

        plt.draw()
        plt.pause(0.01)

    def printStatus(self):
        # print(self.status)
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
        if self.iterator != len(self.x_True):
            posd = np.array([self.x_True[self.iterator], 
                             self.y_True[self.iterator], 0.0])
            err = posd - np.array([self.x, self.y, 0.0])
            theta_dp = np.arctan2(err[1], err[0])
            if np.linalg.norm(err[0:2]) > 0.02 and self.step[0]:
                err[2] = theta_dp - self.theta
                if abs(err[2]) > 0.01 and self.step[1]:
                    self.pos_d = np.array([self.x, self.y, theta_dp])
                    self.status = "Rotating to next orientation"
                else:
                    self.step[1] = False
                    self.pos_d = np.array([posd[0], posd[1], theta_dp])
                    self.status = "Moving to next position"
            else:
                self.past_iterator = self.iterator
                self.iterator = self.iterator + 1
                self.ierr_p = np.array([0.0, 0.0, 0.0])
                self.newPath = True
                self.step = [True, True]
        else:
            self.pos_d = np.array([self.x, self.y, self.theta_True])
            err = self.pos_d - np.array([self.x, self.y, self.theta])
            if abs(err[2]) < 0.01:
                self.finished = True
                self.status = "Final position reached"
                plt.savefig("controlTraReal.png")
                plt.close()

        self.xd.append(self.pos_d[0])
        self.yd.append(self.pos_d[1])
        self.thetad.append(self.pos_d[2])

    def control(self):
        pos = np.array([self.x, self.y, self.theta])
        err = self.pos_d - pos
        if len(self.xe) == 0:
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
        self.xei.append(ierr[0])
        self.yei.append(ierr[1])
        self.thetaei.append(ierr[2])
        self.xed.append(derr[0])
        self.yed.append(derr[1])
        self.thetaed.append(derr[2])
        
        dpos = np.dot(self.kp, err) + np.dot(self.kd, derr) + np.dot(self.ki, ierr)
        print(dpos)
        ik = np.dot(np.linalg.pinv(self.S), dpos)
        self.vel.linear.x = np.clip(ik[0], -self.vmax, self.vmax)
        self.vel.angular.z = np.clip(ik[1], -self.wmax, self.wmax)
        self.linear_vel.append(self.vel.linear.x)
        self.angular_vel.append(self.vel.angular.z)
        print(self.vel.linear.x, self.vel.angular.z)

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
                # self.printStatus()

                print("\n")
                rate.sleep()
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
            np.savetxt("controlTraReal.txt",np.column_stack(
                (self.time,self.xp, self.yp, self.thetap,
                # self.xd, self.yd, self.thetad,
                # self.xe, self.ye, self.thetae,
                # self.xei, self.yei, self.thetaei,
                # self.xed, self.yed, self.thetaed,
                self.linear_vel, self.angular_vel,
                self.lin_real, self.ang_real)
            ))
        except KeyboardInterrupt:
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
            plt.savefig("controlTraReal.png")
            plt.close()
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("controltra", disable_signals=True)
    kp = [0.8, 0.8, 0.8]
    ki = [5e-3, 5e-3, 7e-3]
    kd = [0.0, 0.0, 0.0]

    traj_x = np.array([-2.0, -4.0, -4.0])
    traj_y = np.array([-1.0, 1.0, 4.0])
    cp = ControlTrajectory(traj_x, traj_y, 0.0, kp, ki, kd)
    cp.run()
