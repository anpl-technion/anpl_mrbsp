#!/bin/bash
# Start decentralized nodes. 
# First argument is the robot ID.
# Second argument is the time_delay in seconds between starting different groups of nodes


# start odomtery node, state machine, collision detection ...
roslaunch anpl_mrbsp_tutorials exercise3_robot_$1.launch &

# make sure nodes are initialized before starting a controller
if [ -z "$2" ]; then
	sleep 10
	echo "Controller enabled after 10 seconds."
else
	sleep $2
	echo "Controller enabled after $2 seconds."
fi

# start the controller
roslaunch anpl_mrbsp_tutorials exercise3_controller_setup_gazebo_$1.launch


# terminate all started processes including this script when SIGNAL received
trap "trap - SIGTERM && kill 0" SIGINT SIGTERM EXIT
