Running Gazebo simulation:

1. roslaunch topological_mr_active pioneer3at_world.launch # Gazebo world 

# @ central computer
2. roslaunch topological_mr_active centralized_laser_active_topological_gazebo.launch # centralized nodes

# @ robot A
> roscd topological_mr_active/scripts
3.a ./robot_setup_gazebo.sh A 5 # decentralized nodes, run on each robot with its own ID: A, B, ..., second argument is the time delay between running different dependent group of nodes. Controller must wait until its inputs are ready, otherwise Gazebo teleports the robot to origin. Launch file does not guarantee the order of execution of nodes.
# @ robot B
3.b ./robot_setup_gazebo.sh B 5



Running real experiment with Pioneer robots:

 - Robot setup (on the robots, robot a - top lidar, robot b - bot lidar):
  1) Connect to ANPL_LINKSYS network
  2) In new terminal, open roscore:
     $ roscore
  3) In new teminal, run master discovery:
     $ rosrun master_discovery_fkie master_discovery
  4) In new teminal, run master sync:
     $ rosrun master_sync_fkie master_sync
  5) Turn the robot on and connect the lidar to the battery:
  6) Go to the scenario folder:
     $ roscd topological_mr_active/scripts
  7) Connect to the sensor onboard the robot:
     $ ./run_robot_sensors_A.sh (or ./run_robot_sensors_B.sh)
  8) Launch the odometry node:
     $  roslaunch topological_mr_active robot_setup_pioneer_A.launch (or  roslaunch topological_mr_active robot_setup_pioneer_B.launch)


 - Centralized computer setup:
   1) Follow steps 1-4 in the robot setup
   2) In new terminal, launch the centralized launch file:
      $ roslaunch topological_mr_active centralized_laser_active_topological_pioneer.launch
