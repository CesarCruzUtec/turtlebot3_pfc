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
        self.xd = xd
        self.yd = yd
        self.thetad = thetad
        self.x = []
        self.y = []
        self.theta = []
        self.k = 0.0
        self.S = np.zeros((3, 2))
        self.kp = np.diag(kp)
        self.kd = np.diag(kd)
        self.ki = np.diag(ki)

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
            self.theta.append(2 * self.k * np.pi + euler[2])
        self.S = np.array(
            [[np.cos(self.theta[-1]), 0.0], [np.sin(self.theta[-1]), 0.0], [0.0, 1.0]]
        )
    
    def control(self, pos_d):
        pos = np.array([self.x[-1], self.y[-1], self.theta[-1]])
        err = pos_d - pos
        
        
        dpos = np.dot(self.kp, err) + np.dot(self.kd, derr) + np.dot(self.ki, ierr)

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        vmax = 0.26
        wmax = 1.82
        try:
            while not rospy.is_shutdown():
                os.system("clear")

                self.newDesiredPosition()
                self.control()
                
                print("\n")
                rate.sleep()
        except KeyboardInterrupt:
            vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(vel)
            print("Shutting down")


if __name__ == "__main__":
    rospy.init_node("controlpos", disable_signals=True)
    kp = [1, 1, 1]
    kd = [0.5, 0.5, 0.5]
    ki = [0.2, 0.2, 0.2]
    cp = ControlPosition(1.0, 1.0, 45.0 * np.pi / 180.0)
    cp.run()
