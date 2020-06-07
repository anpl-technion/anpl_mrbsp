#!/bin/bash

# @@@---Important---@@@ 
# This script modifyies on the computer it was run (think it through)!!!
# This script will chnage the number of robots in "planner/include/planner/config.h"
# This script will chnage the number of robots in "config_filers/scenario_config.yaml"
#-------------------------------------------------------------------------------#


# --------------------------------------------------------------------------------------
# Cheking default argument (number of robots)
# --------------------------------------------------------------------------------------
if [ $# -eq 0 ]
then
	NUM_R=1
	echo "only: Default chosen number of robot is  '$NUM_R' "
else
	NUM_R=$1
	echo "only: Chosen number of robot is  '$NUM_R' "
fi


# --------------------------------------------------------------------------------------
# Taking the number of robots from "config.h"
# --------------------------------------------------------------------------------------
N_MAX_ROBOTS=4
PREVIOUS_NUM_RORBOTS=$(awk '/^#define NUM_ROBOTS/ {print $3}' ../../../planner/include/planner/config.h)
echo "only: Number of robot in 'planner/config.h' is '$PREVIOUS_NUM_RORBOTS' " 


# --------------------------------------------------------------------------------------
# Changing the number of robots in 'scenario_config.yaml'
# --------------------------------------------------------------------------------------
sed "s/num_of_robots: .*/num_of_robots: $NUM_R/" ../config_files/scenario_config.yaml > ../config_files/TMP.yaml
mv ../config_files/TMP.yaml ../config_files/scenario_config.yaml
echo "only: Changed 'num_of_robots:' in 'config_files/scenario_config.yaml' to '$NUM_R'"



if [ 1 -eq 1 ]; then
# --------------------------------------------------------------------------------------
# Changing the number of robots, if needed, in the "config.h"
# --------------------------------------------------------------------------------------
if [ "$PREVIOUS_NUM_RORBOTS" -ne "$NUM_R" ]
then
	read -p "
only: This will change ROBOT_NUM in 'planner/config.h' and force 'catkin build planner'
Continue [y/n]?" yn
	case $yn in
		[y*] ) echo "only: You choose to modify!";;
		[n*] ) exit;;	
	esac

	sed "/#define NUM_ROBOTS $PREVIOUS_NUM_RORBOTS/ s/$PREVIOUS_NUM_RORBOTS/$NUM_R/" ../../../planner/include/planner/config.h > ../../../planner/include/planner/configTEST.h
	mv ../../../planner/include/planner/configTEST.h ../../../planner/include/planner/config.h
	# cd ../../../planner/
	catkin clean planner
	catkin build planner
	# cd ../scenarios/$scenario_name/scripts/
else
	echo "only: No need to build planner !!!"
fi



fi