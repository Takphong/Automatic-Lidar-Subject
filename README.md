Instruction
-


It's was currently work in simulation right now. So, we gonna use only "allinonesim" to run


I run in VirtualBox btw, it's may have overturn or have an error value anyway if you run in main ubuntu OS.


--------------------------------------------------------------------------------------------------


To run with simultaion:
-


ros2 launch turtlebot3_gazebo course_final.launch.py


ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/miracle_final/map/map.yaml


allinonesim


(To run node - no "ros2 run" anyway because I somehow create package in global and didn't change it's back yet)



--------------------------------------------------------------------------------------------------
