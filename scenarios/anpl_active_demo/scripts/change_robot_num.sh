#!/bin/bash

# @@@---Important---@@@ 
# This script modifyies on the computer it was run (think it through)!!!
# This script will chnage the number of robots in "planner_node/include/planner_node/config.h"
# This script will chnage the number of robots in "config_filers/scenario_config.yaml"
#-------------------------------------------------------------------------------#

# @@@--Script prerequisite---@@@
# Cenralized need to be modified
	# Example:
		# ...
		# <!-- Load robot params -->
		# <group ns="Robot_A">
		#   <rosparam command="load" file="$(arg config_files_dir)/gazebo_robot_A.yaml" />
		# </group>
		# <group ns="Robot_B">
		#   <rosparam command="load" file="$(arg config_files_dir)/gazebo_robot_B.yaml" />
		# </group>
		# <group ns="Robot_C">
		#   <rosparam command="load" file="$(arg config_files_dir)/gazebo_robot_C.yaml" />
		# </group>
		# <group ns="Robot_D">
		#   <rosparam command="load" file="$(arg config_files_dir)/gazebo_robot_D.yaml" />
		# </group>
		# ....
# YAML files for each robot needed to be created
# scenario_config.yaml needs to be loaded (robot setup or centralized)

exit
# NOT READY DO NOT RUN
# NOT READY DO NOT RUN
# NOT READY DO NOT RUN




# --------------------------------------------------------------------------------------
# Cheking default argument (number of robots)
# --------------------------------------------------------------------------------------
if [ $# -eq 0 ]
then
	NUM_ROBOTS=1
	echo "Default chosen number of robot is  '$NUM_ROBOTS' "
	else
	NUM_ROBOTS=$1
	echo "Chosen number of robot is  '$NUM_ROBOTS' "
fi

# --------------------------------------------------------------------------------------
# Function 'chr()' converts char to ASCII number
# Function 'ord()' converts number to ASCII char
# --------------------------------------------------------------------------------------
chr() {
  [ "$1" -lt 256 ] || return 1
  printf "\\$(printf '%03o' "$1")"
}

ord() {
  LC_CTYPE=C printf '%d' "'$1"
}


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


# --------------------------------------------------------------------------------------
# Taking the number of robots from "config.h"
# --------------------------------------------------------------------------------------
N_MAX_ROBOTS=4
PREVIOUS_NUM_RORBOTS=$(awk '/^#define NUM_ROBOTS/ {print $3}' ../../../planner/include/planner/config.h)
echo "Number of robot before was '$PREVIOUS_NUM_RORBOTS' " 
sleep 3s


# --------------------------------------------------------------------------------------
# Changing the number of robots, if needed, in the "config.h"
# --------------------------------------------------------------------------------------
if [ $PREVIOUS_NUM_RORBOTS != $NUM_ROBOTS ]
then
	sed "/#define NUM_ROBOTS $PREVIOUS_NUM_RORBOTS/ s/$PREVIOUS_NUM_RORBOTS/$NUM_ROBOTS/" ../../../planner/include/planner/config.h > ../../../planner/include/planner/configTEST.h
	mv ../../../planner/include/planner/configTEST.h ../../../planner/include/planner/config.h
	cd ../../../planner/
	catkin clean planner
	catkin build planner
	cd ../scenarios/$scenario_name/scripts/
else
	echo "No need to build planner !!!"
fi
sed "s/num_of_robots: .*/num_of_robots: $NUM_ROBOTS/" ../config_files/scenario_config.yaml > ../config_files/TMP.yaml
mv ../config_files/TMP.yaml ../config_files/scenario_config.yaml

# --------------------------------------------------------------------------------------
# Creating string for "pioneer3at_world.launch", 
# (Max robots = 4, chosen robots = 2) ---> STR="A:=true B:=true C:=false D:=false"
# --------------------------------------------------------------------------------------
if [ $N_MAX_ROBOTS -ge $NUM_ROBOTS ]
then
	for (( i=1; i <= $N_MAX_ROBOTS; ++i ))
	do
		BOOL="false"
		let LETTER=64+$i
		LETTER=$(chr $LETTER)
		if [ $NUM_ROBOTS -gt 0 ]
		then
			let NUM=$i+2
			# only robot id with true status, created command call (cmd3 and above)
			eval "cmd${NUM}='./robot_setup_gazebo.sh $LETTER 5'"
			BOOL="true"
		fi
		let NUM_ROBOTS--
		STR+=$LETTER:=$BOOL-
		STR="$(echo "$STR" | sed 's/-/ /g')"
		
	done
fi

echo $STR

# --------------------------------------------------------------------------------------
# Initial Commands
# --------------------------------------------------------------------------------------
# cmd1="roslaunch $scenario_name pioneer3at_world.launch A:=true B:=true C:=false D:=false"
cmd1="roslaunch $scenario_name pioneer3at_world.launch $STR"
# cmd2="roslaunch $scenario_name centralized_laser_"$middle_scenario_name"_demo_gazebo_debugplanner.launch"
cmd2="roslaunch $scenario_name centralized_laser_"$middle_scenario_name"_demo_gazebo.launch"
# cmd3="./robot_setup_gazebo.sh A 5"
# cmd4="./robot_setup_gazebo.sh B 5"
# cmd5="./robot_setup_gazebo.sh C 5"
# cmd6="./robot_setup_gazebo.sh D 5"

# --------------------------------------------------------------------------------------
# Print Commands
# --------------------------------------------------------------------------------------
printf "Will run:
		(1) gazebo world         	[$cmd1]
		(2) centrelazied        	[$cmd2]
		(3) robot A               	[$cmd3]
		(4) robot B               	[$cmd4]
		(3) robot C               	[$cmd5]
		(4) robot D               	[$cmd6]"
# --------------------------------------------------------------------------------------
# Excute Commands
# --------------------------------------------------------------------------------------
gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab -e "sh -c 'sleep 8; $cmd2; $SHELL'" \
               --tab -e "sh -c 'sleep 11; $cmd3; $SHELL'" \
               --tab -e "sh -c 'sleep 12; $cmd4; $SHELL'" \
               --tab -e "sh -c 'sleep 13; $cmd5; $SHELL'" \
               --tab -e "sh -c 'sleep 14; $cmd6; $SHELL'"
                

printf "\n\n File End \n\n"