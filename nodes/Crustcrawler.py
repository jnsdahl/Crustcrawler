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

class Crustcrawler:
     def __init__(self):
	self.gripper_pub = rospy.Publisher("/gripper/command", Float64)
	self.joint_positions = []
        self.names = ["joint1",
                      "joint2",
                      "joint3",
                      "joint4"]
     
     def open_gripper():
	self.gripper_pub.publish(0.1)
     
     def close_gripper():
	self.gripper_pub.publish(0.5)

     def move(p, theta)
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
		for p in xyz_newpositions:
            		joint_positions = invkin(p, theta)
            		jtp = JointTrajectoryPoint(positions=joint_positions, velocities=[0.5] * self.N_JOINTS, time_from_start=dur)
            		dur += rospy.Duration(2)
            		self.joint_positions.append(jtp)

     def reset_pos():
	move([0, 0, 50], 0)


