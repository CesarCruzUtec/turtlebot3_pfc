#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import tf.transformations
import os
import matplotlib.pyplot as plt


global x, y, theta, k, S, thetaR
x = 0.0
y = 0.0
theta = 0.0
thetaR = 0.0
k = 0.0
S = np.zeros((3, 2))
init = True


# Fix orientatation greater than pi or less than -pi
def fix_angle(angle):
    if angle > np.pi:
        angle = angle - 2 * np.pi
    elif angle < -np.pi:
        angle = angle + 2 * np.pi
    return angle


# Get current position(x,y) and orientation(theta) from odom topic
def callback(data):
    global x, y, theta, k, S, init, thetaR
    if init:
        init = False

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    # Convert orientation from quatertion to euler angles to get theta(z-axis)
    quaternion = data.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )
    if (
        thetaR < np.pi
        and thetaR > 0.9 * np.pi
        and euler[2] > -np.pi
        and euler[2] < -0.9 * np.pi
    ):
        k = k + 1
    elif (
        thetaR > -np.pi
        and thetaR < -0.9 * np.pi
        and euler[2] < np.pi
        and euler[2] > 0.9 * np.pi
    ):
        k = k - 1
    thetaR = euler[2]
    theta = 2 * k * np.pi + thetaR
    S = np.array([[np.cos(theta), 0.0], [np.sin(theta), 0.0], [0.0, 1.0]])


# Position control to move the robot to a desired position and orientation
# using the current position and orientation from odom topic and
# publishing the velocity to cmd_vel topic

if __name__ == "__main__":
    rospy.init_node("controltra", disable_signals=True)
    rospy.Subscriber("/odom", Odometry, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    Vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    # Initiliaze plot
    fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(6, 9))
    axs[0].set_xlim(-2, 2)
    axs[0].set_ylim(-2, 2)
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

    # Velocity limits
    v_max = 0.26  # 0.26 m/s
    w_max = 1.82  # 1.82 rad/s

    init_pose = True
    pos_d = np.array([0.0, 0.0, 0.0])
    traj_x = np.array([-1.0, 1.0, 1.0, -1.0, -1.0])
    traj_y = np.array([-1.0, -1.0, 1.0, 1.0, -1.0])

    # Plot desired position
    axs[0].plot(traj_x, traj_y, "r--", label="Desired position")
    plt.draw()
    plt.pause(0.001)

    # Position control gains (matrix Kp)
    Kp = np.diag([2, 2, 2])
    Kd = np.diag([0.15, 0.15, 0.15])

    iterator = 0
    theta_dpp = 0
    try:
        while not rospy.is_shutdown():
            if not init:
                os.system("clear")
                pub.publish(Vel)

                if iterator == np.size(traj_x):
                    Vel.linear.x = 0.0
                    Vel.angular.z = 0.0
                    pub.publish(Vel)
                    print("Stopped")
                    plt.savefig("controltra.png")
                    break
                elif iterator == 0:
                    x_d = traj_x[0]
                    y_d = traj_y[0]
                    theta_dp = np.arctan2(y_d - y, x_d - x) + 2 * k * np.pi
                    theta_d = (
                        np.arctan2(traj_y[1] - y_d, traj_x[1] - x_d) + 2 * k * np.pi
                    )
                    if abs(x_d - x) > 0.02 and abs(y_d - y) > 0.02:
                        # if(abs(fix_angle(theta_dp-theta)) > 0.05 and init_pose):
                        if abs(theta_dp - theta) > 0.05 and init_pose:
                            pos_d = np.array([x, y, theta_dp])
                        else:
                            init_pose = False
                            pos_d = np.array([x_d, y_d, theta_dp])
                    else:
                        # if(abs(fix_angle(theta_d-theta)) > 0.05):
                        if abs(theta_d - theta) > 0.05:
                            pos_d = np.array([x, y, theta_d])
                        else:
                            iterator = 1
                            init_pose = True
                else:
                    x_d = traj_x[iterator]
                    y_d = traj_y[iterator]
                    theta_dp = np.arctan2(y_d - y, x_d - x) + 2 * k * np.pi
                    # if(abs(fix_angle(theta_dp-theta)) > 0.01 and init_pose):
                    if abs(theta_dp - theta) > 0.01 and init_pose:
                        pos_d = np.array(
                            [traj_x[iterator - 1], traj_y[iterator - 1], theta_dp]
                        )
                        # pos_d = np.array([x,y,theta_dp])
                    else:
                        init_pose = False
                        if abs(x_d - x) > 0.02 or abs(y_d - y) > 0.02:
                            pos_d = np.array([x_d, y_d, theta_dp])
                        else:
                            iterator = iterator + 1
                            theta_dpp = theta_dp
                            init_pose = True

                # Get current position and orientation from odom topic
                pos = np.array([x, y, theta])
                # Get the error between the current position and the desired position
                error = pos_d - pos
                # error[2] = fix_angle(error[2])
                # Derivate the error using the previous error
                if len(xe) > 0:
                    error_d = (error - np.array([xe[-1], ye[-1], thetae[-1]])) / 0.1
                else:
                    error_d = np.array([0.0, 0.0, 0.0])
                # Calculate the velocity (dx,dy,dtheta) using the position control gains (Kp)
                dpos = np.dot(Kp, error) + np.dot(Kd, error_d)
                # Convert from (dx,dy,dtheta) to (linear velocity, angular velocity)
                # using the inverse of the matrix S
                vel = np.dot(np.linalg.pinv(S), dpos)
                # Saturate the linear velocity to 0.26 m/s
                # and the angular velocity to 1.82 rad/s
                vel[0] = np.clip(vel[0], -v_max, v_max)
                vel[1] = np.clip(vel[1], -w_max, w_max)
                # Modify the velocity to publish it to cmd_vel topic
                Vel.linear.x = vel[0]
                Vel.angular.z = vel[1]
                # Print the current position, error and velocity
                # formatting the values to 2 decimal places and justifying to the right
                print("Iteration: {0:>3d}".format(iterator))
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
