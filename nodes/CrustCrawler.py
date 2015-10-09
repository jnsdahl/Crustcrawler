#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from math import atan2, cos, sin, sqrt, pi


class CrustCrawler:
    def __init__(self):
        self.gripper_pub = rospy.Publisher("/gripper/command", Float64)
        self.client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def open_gripper(self):
        self.gripper_pub.publish(0.1)
        rospy.sleep(.3)

    def close_gripper(self):
        self.gripper_pub.publish(0.6)
        rospy.sleep(.3)

    def inverse_kinematics(self, x, y, z, theta):
        d1 = 10.0  # cm (height of 2nd joint)
        a1 = 0.0   # (distance along "y-axis" to 2nd joint)
        a2 = 20.0  # (distance between 2nd and 3rd joints)
        d4 = 20.0  # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

        q1 = atan2(y, x)

        # calculation of q2 and q3
        r2 = (x - a1 * cos(q1)) ** 2 + (y - a1 * sin(q1)) ** 2
        s = z - d1
        D = (r2 + s ** 2 - a2 ** 2 - d4 ** 2) / (2 * a2 * d4)

        q3 = atan2(-sqrt(1 - D ** 2), D)

        q2 = atan2(s, sqrt(r2)) - atan2(d4 * sin(q3), a2 + d4 * cos(q3)) - pi / 2

        q4 = theta + q1

        return q1, q2, q3, q4

    def move_to(self, x, y, z, theta):
        jtp = JointTrajectoryPoint(
            positions=self.inverse_kinematics(x, y, z, theta),
            velocities=[0.5] * 4,
            time_from_start=rospy.Duration(2)
        )

        jt = JointTrajectory(
            joint_names=["joint1", "joint2", "joint3", "joint4"],
            points=[jtp]
        )

        goal = FollowJointTrajectoryGoal(trajectory=jt, goal_time_tolerance=rospy.Duration(4))

        self.client.send_goal(goal)
        self.client.wait_for_result()

    def reset(self):
        self.move_to(0, 0, 50, 0)
        rospy.sleep(1)
