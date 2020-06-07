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



# --------------------------------------------------------------------------------------
# Cheking default argument (number of robots)
# --------------------------------------------------------------------------------------
if [ $# -eq 0 ]
then
	n_rob=1
	echo "lazy: Default chosen number of robot is  '$n_rob' "
else
	n_rob=$1
	echo "lazy: Chosen number of robot is  '$n_rob' "
fi


# --------------------------------------------------------------------------------------
# Robot string for the appropriate number of robots
# --------------------------------------------------------------------------------------
N_MAX_ROBOTS=4

if [ "$N_MAX_ROBOTS" -ge "$n_rob" ] && [ "$n_rob" -ge 1 ]; then
	case $n_rob in
		[1] ) Rob_str="A:=true B:=false C:=false D:=false";;
    	[2] ) Rob_str="A:=true B:=true  C:=false D:=false";;
		[3] ) Rob_str="A:=true B:=true  C:=true  D:=false";;
    	[4] ) Rob_str="A:=true B:=true  C:=true  D:=true";;	
	esac
else
	echo "lazy: The range for number of robots is  [ 1 - $N_MAX_ROBOTS ] "
	exit
fi



# --------------------------------------------------------------------------------------
# Change robot numbers in "planner/include/planner/config.h" and 
# "config_filers/scenario_config.yaml"
# --------------------------------------------------------------------------------------
bash only_change_num_of_robots.sh $n_rob $
if [ $? -eq 42 ]; then
	exit
fi
wait $!


# --------------------------------------------------------------------------------------
# Generating a string for robot setup file 
# Example: [cmd1="./robot_setup_gazebo.sh A 5 & wait $!]
# --------------------------------------------------------------------------------------
chars=( {A..Z} )
cmd_str=( cmd3 cmd4 cmd5 cmd6 )
for ((i=0; i<n_rob; i++))
do
  declare "${cmd_str[i]}"="./robot_setup_gazebo.sh ${chars[i]} 5 & wait "'$'"!"
  
done
# cmd3="./robot_setup_gazebo.sh A 5 & wait $!"
# cmd4="./robot_setup_gazebo.sh B 5 & wait $!"
# cmd5="./robot_setup_gazebo.sh C 5 & wait $!"
# cmd6="./robot_setup_gazebo.sh D 5 & wait $!"


# --------------------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------------------

cmd1="roslaunch $scenario_name pioneer3at_world.launch $Rob_str & wait "'$'"!"
#cmd2="roslaunch $scenario_name centralized_laser_"$middle_scenario_name"_demo_gazebo_debugplanner.launch"
cmd2="roslaunch $scenario_name centralized_laser_${middle_scenario_name}_demo_gazebo.launch $Rob_str & wait "'$'"!"



printf "Will run:
	(1) gazebo world  	[$cmd1]
	(2) centrelazied    	[$cmd2]
	(3) robot A             [$cmd3]
	(4) robot B             [$cmd4]
	(3) robot C             [$cmd5]
	(4) robot D             [$cmd6]

"


if [ 1 -eq 1 ]; then
# --------------------------------------------------------------------------------------
# Execution
# --------------------------------------------------------------------------------------


gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab -e "sh -c 'sleep 5; $cmd2; $SHELL'" \
               --tab -e "sh -c 'sleep 7; $cmd3; $SHELL'" \
               --tab -e "sh -c 'sleep 9; $cmd4; $SHELL'" \
 	       	   --tab -e "sh -c 'sleep 13; $cmd5; $SHELL'" \
               --tab -e "sh -c 'sleep 14; $cmd6; $SHELL'"
                

printf "\n\n File End \n\n"
fi
