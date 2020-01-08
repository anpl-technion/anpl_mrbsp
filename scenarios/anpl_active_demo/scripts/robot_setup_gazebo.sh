#!/bin/bash
# Fish-out scenario name from dir path
scenario_name=$(awk -F/ '{print $(NF-1)}' <<< "$(pwd)")
#echo "$TEST"

# Start decentralized nodes. 
# First argument is the robot ID.
# Optional second argument is the time_delay in seconds between starting different groups of nodes.
# Controller shoud start only after all other nodes are initialized. 


# start odomtery node, state machine, collision detection ...
roslaunch $scenario_name robot_setup_gazebo_$1.launch &

# make sure nodes are initialized before starting a controller
if [ -z "$2" ]; then
	sleep 15
	echo "Controller enabled after 15 seconds."
else
	sleep $2
	echo "Controller enabled after $2 seconds."
fi

# start the controller
roslaunch $scenario_name controller_setup_gazebo_$1.launch


# terminate all started processes including this script when SIGNAL received
trap "trap - SIGTERM && kill 0" SIGINT SIGTERM EXIT