#!/bin/bash



cmd1="roslaunch anpl_interactive_demo pioneer3at_world.launch"
cmd2="roslaunch anpl_interactive_demo centralized_laser_interactive_demo_gazebo_debugplanner.launch"
cmd3="roscd anpl_interactive_demo/scripts/ && ./robot_setup_gazebo.sh A 5"
cmd4="roscd anpl_interactive_demo/scripts/ && ./robot_setup_gazebo.sh B 5"

printf "Will run:
		(1) gazebo world         	[$cmd1]
		(2) centrelazied        	[$cmd2]
		(3) robot A               	[$cmd3]
		(4) robot B               	[$cmd4]"

gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab  -e "sh -c 'sleep 10; $cmd2; '" \
               --tab  --command="bash -c '$cmd3; $SHELL'" \
               --tab  --command="bash -c '$cmd3; $SHELL'" \



printf "\n\n File End \n\n"