#!/bin/bash

# --------------------------------------------------------------------------------------
# Fish-out scenario name from dir path
# --------------------------------------------------------------------------------------
scenario_name=$(awk -F/ '{print $(NF-1)}' <<< "$(pwd)")


# --------------------------------------------------------------------------------------
# Parsing scenario name by "_" and filling array 'tmp'
# --------------------------------------------------------------------------------------
for i in $(echo $scenario_name | tr "_" "\n")
do
  tmp+=($i)
done
middle_scenario_name=${tmp[1]}


# N_MAX_ROBOTS=4

# for i in {1..$N_MAX_ROBOTS}
# do
# 	echo $i
# done 

# perl -CS -le 'print chr shift' 128520


# --------------------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------------------
Rob_str="A:=true B:=true C:=false D:=false"
cmd1="roslaunch $scenario_name pioneer3at_world.launch $Rob_str"
#cmd2="roslaunch $scenario_name centralized_laser_"$middle_scenario_name"_demo_gazebo_debugplanner.launch"
cmd2="roslaunch $scenario_name centralized_laser_${middle_scenario_name}_demo_gazebo.launch $Rob_str"
cmd3="./robot_setup_gazebo.sh A 5"
cmd4="./robot_setup_gazebo.sh B 6"
cmd5="./robot_setup_gazebo.sh C 7"
cmd6="./robot_setup_gazebo.sh D 8"

if [ 0 -eq 0 ]; then

printf "Will run:
		(1) gazebo world         	[$cmd1]
		(2) centrelazied        	[$cmd2]
		(3) robot A               	[$cmd3]
		(4) robot B               	[$cmd4]
		(3) robot C               	[$cmd5]
		(4) robot D               	[$cmd6]"


gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab -e "sh -c 'sleep 10; $cmd2; $SHELL'" \
               --tab -e "sh -c 'sleep 11; $cmd3; $SHELL'" \
               --tab -e "sh -c 'sleep 12; $cmd4; $SHELL'" 
  #             --tab -e "sh -c 'sleep 13; $cmd5; $SHELL'" \
 #              --tab -e "sh -c 'sleep 14; $cmd6; $SHELL'"
                

printf "\n\n File End \n\n"
fi