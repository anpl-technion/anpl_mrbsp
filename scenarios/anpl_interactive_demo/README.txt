Running Gazebo simulation:

1. roslaunch anpl_interactive_demo pioneer3at_world.launch 

2. roslaunch anpl_interactive_demo centralized_laser_interactive_demo_gazebo.launch
## roslaunch anpl_interactive_demo centralized_laser_interactive_demo_gazebo_debugplanner.launch

# @ robot A
> roscd anpl_interactive_demo/scripts
3.a ./robot_setup_gazebo.sh A 5 

# decentralized nodes, run on each robot with its own ID: A, B, ..., second argument is the time delay between running different dependent group of nodes. Controller must wait until its inputs are ready, otherwise Gazebo teleports the robot to origin. Launch file does not guarantee the order of execution of nodes.

# @ robot B
3.b ./robot_setup_gazebo.sh B 5
