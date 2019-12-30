#!/bin/bash



cmd1="roscore"
cmd2="rosrun master_discovery_fkie master_discovery"
cmd3="rosrun master_sync_fkie master_sync"

printf "Will run:
		(1) roscore         	[$cmd1]
		(2) masterdiscovery 	[$cmd2]
		(3) mastersynch     	[$cmd3]"

gnome-terminal --geometry 100x17-1150-500 --tab  --command="bash -c '$cmd1; $SHELL'" \
               --tab  --command="bash -c '$cmd2; $SHELL'" \
               --tab  --command="bash -c '$cmd3; $SHELL'"



printf "\n\n File End \n\n"