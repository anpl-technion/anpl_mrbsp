#!/bin/bash
# Fish-out scenario name from dir path
scenario_name=$(awk -F/ '{print $(NF-1)}' <<< "$(pwd)")
#echo "$TEST"

# Start decentralized nodes. 
# First argument is the robot ID.
# Optional second argument is the time_delay in seconds between starting different groups of nodes.
# Controller shoud start only after all other nodes are initialized. 


# --------------------------------------------------------------------------------------
# Print char from ASCII number
# --------------------------------------------------------------------------------------
chr() {
  [ "$1" -lt 256 ] || return 1
  printf "\\$(printf '%03o' "$1")"
}
# --------------------------------------------------------------------------------------
# Print number from ASCII char
# --------------------------------------------------------------------------------------
ord() {
  LC_CTYPE=C printf '%d' "'$1"
}
# --------------------------------------------------------------------------------------
# The number for pionner id
# --------------------------------------------------------------------------------------
num=$(ord $1)
let "num = num - 64"

# --------------------------------------------------------------------------------------
# start odomtery node, state machine, collision detection ...
# --------------------------------------------------------------------------------------
cmd1="roslaunch $scenario_name robot_setup_gazebo.launch R_id:=$1 p_id:=$num"
gnome-terminal --geometry 100x17-1150-300 --tab  -e "sh -c '$cmd1; $SHELL'"


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
cmd2="roslaunch $scenario_name controller_setup_gazebo.launch R_id:=$1 p_id:=$num"
gnome-terminal --geometry 100x17-1150-200 --tab  -e "sh -c '$cmd2; $SHELL'"

# --------------------------------------------------------------------------------------
# terminate all started processes including this script when SIGNAL received
# --------------------------------------------------------------------------------------
trap "trap - SIGTERM && kill 0" SIGINT SIGTERM EXIT

