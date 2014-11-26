#!/usr/bin/env python


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory



class ActionExampleNode:

	POS_UP_HOME =  [-0.0, 0.0, 0.0, 0.0,0.0]
	POS_UP_HOLD =  [-0.0, -1.145, -1.563, 0.0, 0.0]
	POS_UP_PICK =  [-0.0, -1.145, -1.563, 0.0, 0.8]
	POS_UP_PICK2 =  [-0.0, -0.3, -0.1, 0.0,0.8]
	POS_UP_PICK_HOLD =  [-0.6, -1.145, -1.303, 0.0,0.8]
	POS_UP_RELEASE = 	[-0.6, -1.145, -1.303, 0.0,0.6]
	POS_UP_RELEASE2 = 	[-0.6, -1.145, -1.303, 0.0,0.0]
	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)


		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4",
				"gripper"
				]
		self.joint_positions = [ 
			JointTrajectoryPoint(positions=self.POS_UP_HOME,velocities=[0.5]*5 ,time_from_start=rospy.Duration(1)), 
			JointTrajectoryPoint(positions=self.POS_UP_HOLD,velocities=[0.5]*5 ,time_from_start=rospy.Duration(5)),  
			JointTrajectoryPoint(positions=self.POS_UP_PICK,velocities=[0.5]*5 ,time_from_start=rospy.Duration(7)), 
			JointTrajectoryPoint(positions=self.POS_UP_PICK2,velocities=[0.5]*5 ,time_from_start=rospy.Duration(12)), 
			JointTrajectoryPoint(positions=self.POS_UP_PICK_HOLD,velocities=[0.5]*5 ,time_from_start=rospy.Duration(16)), 
			JointTrajectoryPoint(positions=self.POS_UP_RELEASE,velocities=[0.5]*5 ,time_from_start=rospy.Duration(18)), 
			JointTrajectoryPoint(positions=self.POS_UP_RELEASE2,velocities=[0.5]*5 ,time_from_start=rospy.Duration(22)),
			JointTrajectoryPoint(positions=self.POS_UP_HOME,velocities=[0.5]*5 ,time_from_start=rospy.Duration(27)),
			]
		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=rospy.Duration(10) )

	def send_command(self):
		self.client.wait_for_server()
		
		self.client.send_goal(self.goal)
		
		self.client.wait_for_result()
		print self.client.get_result()
		
if __name__ == "__main__":
	rospy.init_node("au_dynamixel_test_node")
	
	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")
	
	node.send_command()
