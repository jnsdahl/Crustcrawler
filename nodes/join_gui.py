#!/usr/bin/env python
from quickui.QuickUi import *


m = gui("TestExampleGui","vertical",
        group("Advanced creation","vertical",
	
              #
              # Besides integer ranges, strings can also be used and we can have
              # different entries here we want the label to be EncA,EncB...
              # but the topics are /example_topic0,/example_topic1
              *iterate( ros_slider, ("Joint{0}","/joint{1}/command","std_msgs/Float64",".data",(-1.8,1.8),0.0), [["1","1"], ["2",2],["3",3],["4",4]] )
              ),
		ros_slider("Gripper","/gripper/command","std_msgs/Float64",".data",(-0.2,0.2),0.0)
		
        )
#
# if debug=True a string representation of the gui is printed to the console
#
run(m,debug=True)
