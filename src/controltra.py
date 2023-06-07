#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

global x, y, theta, S

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
    rospy.init_node("controltra", disable_signals=True)
    rospy.Subscriber("/odom", Odometry, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    Vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    # Initiliaze plot
    fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(6, 9))
    axs[0].set_xlim(-1.5, 1.5)
    axs[0].set_ylim(-1.5, 1.5)
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

    pos_d = np.array([0.0, 0.0, 0.0])
    init_pose = True
    # traj_x = np.zeros(n)
    # traj_y = np.zeros(n)
    # for i in range(n):
    #     if i < n / 4:
    #         traj_x[i] = -0.5 + 1.0 * i / (n / 4)
    #         traj_y[i] = -0.5
    #     elif i < n / 2:
    #         traj_x[i] = 0.5
    #         traj_y[i] = -0.5 + 1.0 * (i - n / 4) / (n / 4)
    #     elif i < 3 * n / 4:
    #         traj_x[i] = 0.5 - 1.0 * (i - n / 2) / (n / 4)
    #         traj_y[i] = 0.5
    #     else:
    #         traj_x[i] = -0.5
    #         traj_y[i] = 0.5 - 1.0 * (i - 3 * n / 4) / (n / 4)
    traj_x = np.array([-1.0, 1.0, 1.0, 0.5, 0.0, -0.5, -1.0, -1.0])
    traj_y = np.array([-1.0, -1.0, 1.0, 0.5, 1.0, 0.5, 1.0, -1.0])
    # traj_x = np.linspace(0.0,5.0,20)
    # traj_y = np.sin(traj_x)
    n = len(traj_x)
    axs[0].plot(traj_x, traj_y, "r--")
    plt.draw()
    plt.pause(0.001)

    Kp = np.diag([1.0, 1.0, 1.5])
    Kd = np.diag([0.25, 0.25, 0.15])
    Ki = np.diag([0.15, 0.15, 0.15])
    step = [True, True, False, False, False]
    iterator = 0
    error_p = None
    try:
        while not rospy.is_shutdown():
            if not init:
                os.system("clear")
                pub.publish(Vel)
                if iterator < n:
                    x_d = traj_x[iterator]
                    y_d = traj_y[iterator]
                print(x_d)
                print(y_d)
                if iterator == 0:
                    theta_dp = np.arctan2(y_d - y, x_d - x)
                    if (abs(x_d - x) > 0.02 or abs(y_d - y) > 0.02) and step[0]:
                        if abs(fix_angle(theta_dp - theta)) > 0.05 and step[1]:
                            pos_d = np.array([x, y, theta_dp])
                            print("Rotating to initial orientation")
                        else:
                            step[1] = False
                            pos_d = np.array([x_d, y_d, theta_dp])
                            print("Moving to initial position")
                    else:
                        step[0] = True
                        step[1] = True
                        iterator = 1
                elif iterator == n:
                    Vel.linear.x = 0.0
                    Vel.angular.z = 0.0
                    pub.publish(Vel)
                    print("Stopped")
                    print(
                        "Final position: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                            x, y, theta
                        )
                    )
                    print(
                        "Final error: x = {0:>6.4f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                            x_d - x, y_d - y, theta_dp - theta
                        )
                    )
                    plt.savefig("controltra.png")
                    plt.close()
                    break
                else:
                    theta_dp = np.arctan2(y_d - y, x_d - x)
                    if (abs(x_d - x) > 0.02 or abs(y_d - y) > 0.02) and step[0]:
                        if abs(fix_angle(theta_dp - theta)) > 0.05 and step[1]:
                            pos_d = np.array([x, y, theta_dp])
                            print("Rotating to next orientation")
                        else:
                            step[1] = False
                            pos_d = np.array([x_d, y_d, theta_dp])
                            print("Moving to next position")
                    else:
                        step[0] = True
                        step[1] = True
                        iterator = iterator + 1
                pos = np.array([x, y, theta])
                error = pos_d - pos
                error[2] = fix_angle(error[2])
                if error_p is not None:
                    error_d = (error - error_p) / 0.1
                    error_i = (error + error_p) * 0.1 / 2.0
                    error_p = error
                else:
                    error_d = np.zeros(3)
                    error_i = np.zeros(3)
                    error_p = error
                dpos = np.dot(Kp, error) + np.dot(Kd, error_d) + np.dot(Ki, error_i)
                vel = np.dot(np.linalg.pinv(S), dpos)
                vel[0] = np.clip(vel[0], -v_max, v_max)
                vel[1] = np.clip(vel[1], -w_max, w_max)
                Vel.linear.x = vel[0]
                Vel.angular.z = vel[1]
                print("Iteration: {0:>3d}".format(iterator))
                print("States: {}".format(step))
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
                    "Error: x = {0:>6.2f} y = {1:>6.4f} theta = {2:>6.4f}".format(
                        error[0], error[1], error[2]
                    )
                )
                print(
                    "Velocity: linear = {0:>6.4f} angular = {1:>6.4f}".format(
                        Vel.linear.x, Vel.angular.z
                    )
                )
                print("")

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
        plt.savefig("controltra.png")
        plt.close()
        pass
