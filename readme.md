
Simple simulation/viewing the robot
-----------
`roslaunch au_crustcrawler_base view_urdf.launch`
use the gui sliders to move the robot


Launching the real robot
-----------
`roslaunch au_crustcrawler_base base.launch `

in another terminal after the base has been successfully brought up

`roslaunch au_crustcrawler_base meta.launch`

after the meta has launched and exited again start the gui by

`rosrun au_crustcrawler_base joint_gui.py`

You can now control the individual joints of the robot by using the sliders.


Commanding the real robot from python.
-----------
`roslaunch au_crustcrawler_base base.launch serial_port:=/dev/ttyUSB0`

in another terminal after the base has been successfully brought up

`roslaunch au_crustcrawler_base meta.launch`

With the base robot interface up and running we can now use the 
action interface provided by the dynamixel controller. 

`rosrun au_crustcrawler_base au_dynamixel_test_node.py`

see [au_dynamixel_test_node.py](https://github.com/au-crustcrawler/au_crustcrawler_base/blob/master/nodes/au_dynamixel_test_node.py) for an example of controlling the robot
from python.



