#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

global x, y, theta, S, init
x = 0.0
y = 0.0
theta = 0.0
S = np.zeros((3, 2))
init = True


def fix_angle(angle):
    if angle > np.pi:
        angle = angle - 2 * np.pi
    elif angle < -np.pi:
        angle = angle + 2 * np.pi
    return angle


def callback(data):
    global x, y, theta, S, init
    if init:
        init = False

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    quaternion = data.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )
    theta = euler[2]
    S = np.array([[np.cos(theta), 0.0], [np.sin(theta), 0.0], [0.0, 1.0]])


if __name__ == "__main__":
    rospy.init_node("controlpos", disable_signals=True)
    rospy.Subscriber("/odom", Odometry, callback)
    print(
        "Current position: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
            x, y, theta
        )
    )
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    Vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    # Initiliaze plot
    fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(6, 9))
    axs[0].set_xlim(-1, 3)
    axs[0].set_ylim(-1, 3)
    axs[0].set_xlabel("x(m)")
    axs[0].set_ylabel("y(m)")
    axs[0].grid(True)
    axs[1].set_ylabel("x(m)")
    axs[1].grid(True)
    axs[2].set_ylabel("y(m)")
    axs[2].grid(True)
    axs[3].set_ylabel("theta(rad)")
    axs[3].grid(True)
    axs[3].set_xlabel("time(s)")
    tp, xp, yp, thetap = [], [], [], []
    xdp, ydp, thetadp = [], [], []
    xe, ye, thetae = [], [], []

    v_max = 0.26  # 0.26 m/s
    w_max = 1.82  # 1.82 rad/s

    x_d, y_d, theta_d = 0.0, 0.0, 0.0
    theta_d = theta_d * np.pi / 180

    axs[0].plot(x_d, y_d, "ro", markersize=2)
    plt.draw()
    plt.pause(0.001)

    Kp = np.diag([2.0, 2.0, 1.0])

    try:
        while not rospy.is_shutdown():
            if not init:
                os.system("clear")
                pub.publish(Vel)
                theta_dp = np.arctan2(y_d - y, x_d - x)
                if abs(x_d - x) > 0.05 and abs(y_d - y) > 0.05:
                    if abs(fix_angle(theta_dp - theta)) > 0.05:
                        pos_d = np.array([x, y, theta_dp])
                    else:
                        pos_d = np.array([x_d, y_d, theta_dp])
                else:
                    if abs(fix_angle(theta_d - theta)) > 0.05:
                        pos_d = np.array([x_d, y_d, theta_d])
                    else:
                        Vel.linear.x = 0.0
                        Vel.angular.z = 0.0
                        pub.publish(Vel)
                        print("Stopped")
                        print("Final position:")
                        print("x = {0:>6.4f}" % x)
                        print("y = {1:>6.4f}" % y)
                        print("theta = {2:>6.4f}" % theta)
                        print("Final error:")
                        print("x = {0:>6.4f}" % (x_d - x))
                        print("y = {1:>6.4f}" % (y_d - y))
                        print("theta = {2:>6.4f}" % fix_angle(theta_d - theta))
                        plt.savefig("controlpos.png")
                        break

                pos = np.array([x, y, theta])
                error = pos_d - pos
                error[2] = fix_angle(error[2])
                dpos = np.dot(Kp, error)
                vel = np.dot(np.linalg.pinv(S), dpos)
                vel[0] = np.clip(vel[0], -v_max, v_max)
                vel[1] = np.clip(vel[1], -w_max, w_max)
                Vel.linear.x = vel[0]
                Vel.angular.z = vel[1]
                print(
                    "Desired position: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                        pos_d[0], pos_d[1], pos_d[2]
                    )
                )
                print(
                    "Current position: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                        x, y, theta
                    )
                )
                print(
                    "Error: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                        error[0], error[1], error[2]
                    )
                )
                print(
                    "Velocity: linear = {0:>6.4f} angular = {1:>6.4f}".format(
                        Vel.linear.x, Vel.angular.z
                    )
                )
                print("\n")

                t_now = rospy.get_time()
                tp.append(t_now)
                xp.append(x)
                yp.append(y)
                thetap.append(theta)
                xdp.append(pos_d[0])
                ydp.append(pos_d[1])
                thetadp.append(pos_d[2])
                xe.append(error[0])
                ye.append(error[1])
                thetae.append(error[2])

                axs[0].plot(xp, yp, "b")

                axs[1].plot(tp, xp, "b")
                axs[2].plot(tp, yp, "b")
                axs[3].plot(tp, thetap, "b")

                axs[1].plot(tp, xdp, "r")
                axs[2].plot(tp, ydp, "r")
                axs[3].plot(tp, thetadp, "r")

                axs[1].plot(tp, xe, "g")
                axs[2].plot(tp, ye, "g")
                axs[3].plot(tp, thetae, "g")

                axs[1].get_shared_x_axes().join(axs[1], axs[2], axs[3])
                axs[1].set_xticklabels([])
                axs[2].set_xticklabels([])

                plt.draw()
                plt.pause(0.001)
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopped by user")
        Vel.linear.x = 0.0
        Vel.angular.z = 0.0
        pub.publish(Vel)
        pass
