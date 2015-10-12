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
        self.gripper_pub.publish(0.75)
        rospy.sleep(.3)

    def inverse_kinematics_point(self, x, y, z):
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

        q4 = 0

        return q1, q2, q3, q4

    def inverse_kinematics_block(self, x, y, z, block):
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

        q4 = - block.find_orientation(q1)

        return q1, q2, q3, q4

    def move_to_block(self, x, y, z, block):
        jtp = JointTrajectoryPoint(
            positions=self.inverse_kinematics_block(x, y, z, block),
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

    def move_to_point(self, x, y, z):
        jtp = JointTrajectoryPoint(
            positions=self.inverse_kinematics_point(x, y, z),
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

    def pick_up_block(self, block):
        self.move_to_block(block.x, block.y, block.z, block)
        self.open_gripper()
        self.move_to_block(block.x, block.y, block.z - 12, block)
        self.close_gripper()
        self.move_to_block(block.x, block.y, block.z + 5, block)

    def place_block(self, block, x, y):
        self.pick_up_block(block)
        self.move_to_point(x, y, 10)
        self.move_to_point(x, y, 5)
        self.open_gripper()
        self.move_to_point(x, y, 10)

    def place_block_right(self, block, position):
        self.place_block(block, 0, -3 - 15 * position)

    def place_block_left(self, block, position):
        self.place_block(block, 0, 3 + 15 * position)

    def reset(self):
        self.move_to_point(0, 0, 50)
        rospy.sleep(1)
