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
# chr() {
#   [ "$1" -lt 256 ] || return 1
#   printf "\\$(printf '%03o' "$1")"
# }

# --------------------------------------------------------------------------------------
# Print number from ASCII char
# --------------------------------------------------------------------------------------
# ord() {
#   LC_CTYPE=C printf '%d' "'$1"
# }

# --------------------------------------------------------------------------------------
# The number for pioneer id
# --------------------------------------------------------------------------------------
# num=$(ord $1)
# let "num = num - 64"

ROBOT_NAME="Robot_$1"
echo 'export robot name: '$ROBOT_NAME
export $ROBOT_NAME 
sudo chmod a+rw /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0


# Needs to be physiclly checked on the robot 'is_lidar_usb', currently know setup: 
# Robot A - has Hokuyo UTM 30-LX lidar(is_lidar_usb = true)
# Robot B - has Hokuyo UTM 10-LX lidar(is_lidar_usb = false)
case $1 in
	["Aa"] ) condition="true";;
    ["Bb"] ) condition="false";;
	* ) echo "Not yet ready for more robots" 
		  exit;;
esac

echo $condition


cmd0="roslaunch $scenario_name launch_robot_sensors.launch robot_name:=$ROBOT_NAME is_lidar_usb:=$condition" 



# --------------------------------------------------------------------------------------
# start odomtery node, state machine, collision detection ...
# --------------------------------------------------------------------------------------
cmd1="roslaunch $scenario_name robot_setup_pioneer.launch R_id:=$1" # p_id:=$num"
gnome-terminal --geometry 100x17-1150-500 --tab  -e "sh -c '$cmd1; $SHELL'" \
               --tab -e "sh -c 'sleep 5; $cmd0; $SHELL'"


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
cmd2="roslaunch $scenario_name controller_setup_pioneer.launch R_id:=$1" # p_id:=$num"
gnome-terminal --geometry 100x17-1150-200 --tab  -e "sh -c '$cmd2; $SHELL'"

# --------------------------------------------------------------------------------------
# terminate all started processes including this script when SIGNAL received
# --------------------------------------------------------------------------------------
if [ 0 -eq 1 ]; then
trap "trap - SIGTERM && kill 0" SIGINT SIGTERM EXIT

	echo "The END"
fi