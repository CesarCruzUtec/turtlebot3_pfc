#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3


class DirectKinematics:
    def __init__(self, vel_lin=0.0, vel_ang=0.0):
        vel_lin = max(min(vel_lin, 0.26), -0.26)
        vel_ang = max(min(vel_ang, 1.82), -1.82)
        self.velocity = Twist(Vector3(vel_lin, 0.0, 0.0), Vector3(0.0, 0.0, vel_ang))
        print(
            "Velocity: linear = {0:>6.4f} m/s, angular = {1:>6.4f} rad/s".format(
                self.velocity.linear.x, self.velocity.angular.z
            )
        )

    def run(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                pub.publish(self.velocity)
                rate.sleep()
        except KeyboardInterrupt:
            self.velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.velocity)
            print("Stopped")


if __name__ == "__main__":
    rospy.init_node("directkinem", disable_signals=True)
    dk = DirectKinematics(vel_lin=0.26, vel_ang=1.82)
    dk.run()
