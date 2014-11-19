Introduction
-----------
This package provides an example of how to configure the dynamixel components and how to
make a model of the robot arm. 

Clone the aucrustcrawlerbase package:

	cd ~/catkin_ws/src

	git clone https://github.com/au-crustcrawler/au_crustcrawler_base.git

Or if the folder exist just:

	cd ~/catkin_ws/src/au_crustcrawler_base/

	git pull

The robot is described using URDF. The URDF file enables other
ROS components to retrieve information about the robots configuration 
e.g the joints (name of joint and their associated frame in the transform tree (tf)).
The URDF file also provides a visual description of the robot which can be used for visualisation purposes (RVIZ).

The URDF description of the robot is linked to the actual joint controllers via the name of the joint. 
The robot hardware (joint motors and its controller) are specified in the joints.yaml which contains the configuration
of each joint on the robot. For each joint in the URDF file there is a corresponding entry in the joints.yaml file. 
 
Instead of having a topic per joint e.g (/arm_controller/joint1 /arm_controller/joint2 ...) one can create a master controller 
which provides a single action for commanding all joints. This makes it easier to synchronise commands and makes the code which commands
the joints simpler. The main_control.yaml does exactly this.

Simple simulation/viewing the robot
-----------

![demo image of view_urdf](https://raw.githubusercontent.com/au-crustcrawler/au_crustcrawler_base/master/doc/view_urdf.png)

	roslaunch au_crustcrawler_base view_urdf.launch

use the gui sliders to move the robot


Launching the real robot
-----------
	roslaunch au_crustcrawler_base base.launch

in another terminal after the base has been successfully brought up

	roslaunch au_crustcrawler_base meta.launch

after the meta has launched and exited again start the gui by

	rosrun au_crustcrawler_base joint_gui.py

You can now control the individual joints of the robot by using the sliders.

In order to see the robot in rviz live do 

	roslaunch au_crustcrawler_base view_urdf.launch simulate:=false


Commanding the real robot from python.
-----------

	roslaunch au_crustcrawler_base base.launch

in another terminal after the base has been successfully brought up

	roslaunch au_crustcrawler_base meta.launch

With the base robot interface up and running we can now use the 
action interface provided by the dynamixel controller. 

	rosrun au_crustcrawler_base au_dynamixel_test_node.py

see [au_dynamixel_test_node.py](https://github.com/au-crustcrawler/au_crustcrawler_base/blob/master/nodes/au_dynamixel_test_node.py) for an example of controlling the robot
from python.


.
