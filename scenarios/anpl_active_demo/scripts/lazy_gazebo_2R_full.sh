#!/bin/bash

# Fish-out scenario name from dir path
scenario_name=$(awk -F/ '{print $(NF-1)}' <<< "$(pwd)")

echo $1

cmd1="roslaunch $scenario_name pioneer3at_world.launch Robot2_ON:=$1"
#cmd2="roslaunch $scenario_name centralized_laser_active_demo_gazebo_debugplanner.launch"
cmd2="roslaunch $scenario_name centralized_laser_active_demo_gazebo.launch Robot2_ON:=$1"
cmd3="./robot_setup_gazebo.sh A 5"
cmd4="./robot_setup_gazebo.sh B 7"


printf "Will run:
		(1) gazebo world         	[$cmd1]
		(2) centrelazied        	[$cmd2]
		(3) robot A               	[$cmd3]
		(4) robot B               	[$cmd4]"


gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab -e "sh -c 'sleep 10; $cmd2; $SHELL'" \
               --tab -e "sh -c 'sleep 13; $cmd3; $SHELL'" \
               --tab -e "sh -c 'sleep 16; $cmd4; $SHELL'" 



printf "\n\n File End \n\n"