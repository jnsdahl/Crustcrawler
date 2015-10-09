#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
from block_detection import get_blocks
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from math import atan2, cos, sin, sqrt, pi


def px2cm(px):
    return px / 8.765


def transformation(x, y):
    x = px2cm(x)
    y = px2cm(y)
    z = -5

    center = np.array([x, y, z, 1])
    center = center[:, None]

    tx = -34
    ty = 35
    tz = 0
    thetax = pi
    thetaz = pi / 2

    A = np.matrix([[1, 0, 0, tx], [0, cos(thetax), -sin(thetax), ty], [0, sin(thetax), cos(thetax), tz], [0, 0, 0, 1]])
    B = np.matrix([[cos(thetaz), sin(thetaz), 0, 0], [-sin(thetaz), cos(thetaz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    center = B.dot(A.dot(center))

    return center[0], center[1], center[2]


#def gripper(value):
    #grip_pub = rospy.Publisher('/gripper/command', Float64)
    #grip_pub.publish(Float64(value))


def findAngle(point1, point2):
    x1 = point1[0]
    y1 = point1[1]

    x2 = point2[0]
    y2 = point2[1]

    xy1_new = transformation(x1, y1)
    xy2_new = transformation(x2, y2)

    vector_x = xy1_new[0] - xy2_new[0]
    vector_y = xy1_new[1] - xy2_new[1]

    theta = atan2(vector_y, vector_x)

    return theta


def invkin(p, theta):
    x = p[0]
    y = p[1]
    z = p[2]

    d1 = 10.0   # cm (height of 2nd joint)
    a1 = 0.0    # (distance along "y-axis" to 2nd joint)
    a2 = 20.0   # (distance between 2nd and 3rd joints)
    d4 = 20.0   # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

    q1 = atan2(y, x)

    # calculation of q2 and q3
    r2 = (x - a1 * cos(q1)) ** 2 + (y - a1 * sin(q1)) ** 2
    s = (z - d1)
    D = (r2 + s ** 2 - a2 ** 2 - d4 ** 2) / (2 * a2 * d4)

    q3 = atan2(-sqrt(1 - D ** 2), D)

    q2 = atan2(s, sqrt(r2)) - atan2(d4 * sin(q3), a2 + d4 * cos(q3)) - pi/2

    q4 = theta + q1

    return q1, q2, q3, q4


class ActionExampleNode:
    N_JOINTS = 4

    def __init__(self, server_name):

        self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)
        self.joint_cmd_pub = rospy.Publisher("/gripper/command", Float64)

        self.joint_positions = []
        self.names = ["joint1",
                      "joint2",
                      "joint3",
                      "joint4"]

        blocks = get_blocks()
        block = blocks[0]

        # the list of xyz points we want to plan
        xy_positions = [
            [
                block[2][0] + (block[0][0] - block[2][0]) / 2,
                block[3][1] + (block[1][1] - block[3][1]) / 2
            ]
        ]

        # findAngle q4
        t1 = findAngle(block[0], block[1])
        t2 = findAngle(block[1], block[2])
        theta = t1 if t1 < t2 else t2

        # Transformation
        xyz_newpositions = []

        for pos in xy_positions:
            xyz_newpositions.append(transformation(pos[0], pos[1]))


        # initial duration
        dur = rospy.Duration(1)

        # construct a list of joint positions by calling invkin for each xyz point
        for p in xyz_newpositions:
            joint_positions = invkin(p, theta)
            jtp = JointTrajectoryPoint(positions=joint_positions, velocities=[0.5] * self.N_JOINTS, time_from_start=dur)
            dur += rospy.Duration(2)
            self.joint_positions.append(jtp)

        self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
        self.goal = FollowJointTrajectoryGoal(trajectory=self.jt, goal_time_tolerance=dur + rospy.Duration(2))

       

    def send_command(self):
	self.joint_cmd_pub.publish(0.1)        
	self.client.wait_for_server()     
	self.client.send_goal(self.goal)
        self.client.wait_for_result()
	self.joint_cmd_pub.publish(0.9)

if __name__ == "__main__":
    rospy.init_node("au_dynamixel_test_node")

    node = ActionExampleNode("/arm_controller/follow_joint_trajectory")

    node.send_command()
    
