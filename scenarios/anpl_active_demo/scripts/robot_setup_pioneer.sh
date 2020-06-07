#!/bin/bash
# Fish-out scenario name from dir path
scenario_name=$(awk -F/ '{print $(NF-1)}' <<< "$(pwd)")
#echo "$TEST"

# Start decentralized nodes. 
# First argument is the robot ID.
# Optional second argument is the time_delay in seconds between starting different groups of nodes.
# Controller shoud start only after all other nodes are initialized. 

# --------------------------------------------------------------------------------------
# Function that prints letter given the ASCII number
# --------------------------------------------------------------------------------------
chr() {
  [ "$1" -lt 256 ] || return 1
  printf "\\$(printf '%03o' "$1")"
}
# --------------------------------------------------------------------------------------
# Function that prints number given the ASCII letter
# --------------------------------------------------------------------------------------
ord() {
  LC_CTYPE=C printf '%d' "'$1"
}

# --------------------------------------------------------------------------------------
# Robot_id = $1 , pioner_num_id = $num
# --------------------------------------------------------------------------------------
num=$(ord $1)
# echo $num
let "num = num - 64"
# echo $num


bash run_robot_sensors_$1.sh

# --------------------------------------------------------------------------------------
# start odomtery node, state machine, collision detection ...
# --------------------------------------------------------------------------------------
roslaunch $scenario_name robot_setup_pioneer.launch R_id:=$1 p_id:=$num &
wait $!

# --------------------------------------------------------------------------------------
# make sure nodes are initialized before starting a controller
# --------------------------------------------------------------------------------------
if [ -z "$2" ]; then
	sleep 10
	echo "Controller enabled after 10 seconds."
else
	sleep $2
	echo "Controller enabled after $2 seconds."
fi

# --------------------------------------------------------------------------------------	
# start the controller
# --------------------------------------------------------------------------------------
roslaunch $scenario_name controller_setup_pioneer.launch R_id:=$1 p_id:=$num


# terminate all started processes including this script when SIGNAL received
trap "trap - SIGTERM && kill 0" SIGINT SIGTERM EXIT


