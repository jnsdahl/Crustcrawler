#!/usr/bin/env python


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# -0.17 gripper open  -0.10 gripper closed

class ActionExampleNode:

	POS_UP_HOME =  [-0.07158577010132992, 1.2016182838437524, 1.7998707911191525, 0.0, -0.17]
	POS_UP_HOLD =  [-0.07158577010132992, 1.2016182838437524, 1.7998707911191525, 0.0, 0.16]
	PICK_UP_PRE = [1.5084144414208807, 1.2016182838437524, 1.7998707911191525, 0.0 , -0.13]
	PICK_UP = [1.5084144414208807, -0.12783173232380343, 1.7998707911191525, -0.0051132692929521375, -0.13]
	PICK_UP2 = [1.5084144414208807, -0.12783173232380343, 1.7998707911191525, -0.0051132692929521375, 0.16]
	POS_FORWARD =  [-0.07158577010132992, -0.02556634646476069, 1.7998707911191525, 0.0, -0.17]
	POS_RIGHT_MIDDLE = [-0.5880259686894959, -0.03579288505066496, 0.3272492347489368, 0.0, -0.17]
	RELEASE= [0.6851780852555864, 1.4930746335420242, 1.8049840604121046, 0.0,  0.16]
	RELEASE2= [0.6851780852555864, 1.4930746335420242, 1.8049840604121046, 0.0, -0.13]

	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)


		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4",
				"gripper"]
		self.joint_positions = [ 
			JointTrajectoryPoint(positions=self.POS_UP_HOME,velocities=[1.0]*5 ,time_from_start=rospy.Duration(1)), 
			JointTrajectoryPoint(positions=self.PICK_UP_PRE,velocities=[1.0]*5, time_from_start=rospy.Duration(2)), 
			JointTrajectoryPoint(positions=self.PICK_UP,velocities=[1.0]*5, time_from_start=rospy.Duration(4)), 
			JointTrajectoryPoint(positions=self.PICK_UP2,velocities=[1.0]*5, time_from_start=rospy.Duration(5)), 
			JointTrajectoryPoint(positions=self.POS_UP_HOLD,velocities=[1.0]*5 ,time_from_start=rospy.Duration(7)),  
			JointTrajectoryPoint(positions=self.POS_UP_HOME,velocities=[1.0]*5 ,time_from_start=rospy.Duration(8)),  
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
