#!/usr/bin/env python3

import os

import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import datetime


class RandomMov:
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.time_period = 0.0
        self.nextRand = True
        self.startTime = rospy.get_time()
        self.velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
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

    def randomMov(self):
        if self.nextRand:
            self.nextRand = False
            self.startTime = rospy.get_time()
            self.linear_vel = np.random.uniform(-0.26, 0.26)
            self.angular_vel = np.random.uniform(-1.82, 1.82)
            self.time_period = np.random.uniform(1, 2)
        else:
            if rospy.get_time() - self.startTime > self.time_period:
                self.nextRand = True

        self.velocity.linear.x = self.linear_vel
        self.velocity.angular.z = self.angular_vel

    def printAndsave(self):
        os.system("clear")
        print("X = %6.4f m, Y = %6.4f m" % (self.posx, self.posy))
        print("Ref = %6.4f m/s, Vel_lin = %6.4f m/s" % (self.linear_vel, self.vel_lin))
        print(
            "Ref = %6.4f rad/s, Vel_ang = %6.4f rad/s"
            % (self.angular_vel, self.vel_ang)
        )
        print(
            "Remaining: %6.4f s"
            % (self.time_period - (rospy.get_time() - self.startTime))
        )
        with open(self.file_name, "a") as f:
            f.write(
                "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n"
                % (
                    rospy.get_time(),
                    self.posx,
                    self.posy,
                    self.theta,
                    self.linear_vel,
                    self.vel_lin,
                    self.angular_vel,
                    self.vel_ang,
                )
            )

    def run(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.callback)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.randomMov()
                pub.publish(self.velocity)
                #self.printAndsave()
                rate.sleep()
        except KeyboardInterrupt:
            self.velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.velocity)
            print("Stopped")


if __name__ == "__main__":
    rospy.init_node("randommov", disable_signals=True)
    dk = RandomMov()
    dk.run()
