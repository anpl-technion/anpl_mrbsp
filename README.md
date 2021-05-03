# **Welcome to ANPL's Multi Robot Belief Space Planning with ROS (anpl_mrbsp) wiki**

![mrbsp_screenshot.png](https://bytebucket.org/ANPL/anpl_mrbsp/wiki/figures/mrbsp_screenshot.png?token=b151ff7095b20db88b091cad6e0e9fc313f0a1ff&rev=d3cf5d4c2c4f7c6255ef9522c5546395f932cd96)


## Useful wiki pages:
* [High level code structure](https://github.com/anpl-technion/anpl_mrbsp/wiki/High-level-code-structure)
* [Implement new node](https://github.com/anpl-technion/anpl_mrbsp/wiki/Implement_new_node/Implement%20new%20node)
* [Implement new scenario](https://github.com/anpl-technion/anpl_mrbsp/wiki/Generate_new_scenario/Generate%20new%20scenario)
* [Existing nodes](https://github.com/anpl-technion/anpl_mrbsp/wiki/Existing_nodes/Existing%20nodes)
* [Existing scenarios](https://github.com/anpl-technion/anpl_mrbsp/wiki/Existing_scenarios/Existing%20scenarios)
* [Tutorials](https://github.com/anpl-technion/anpl_mrbsp/wiki/Tutorials/Tutorials)
* [FAQ](https://github.com/anpl-technion/anpl_mrbsp/wiki/FAQ/FAQ)

## Overview

**anpl_mrbsp** repository is a set of ros packages design to support experiments in ANPL.
This software package provide active or passive SLAM solution in both simulation and real environment.
This software package is designed with modular to support minimum effort when integrating new components.

The code is constructed with 9 main blocks (nodes):

1. Odometry
2. Data association (DA)
3. Belief
4. Map
5. State machine
6. Action generator
7. Planner
8. Controller
9. Collision detector

A passive scenario will be constructed from nodes 1-4.
Active scenario will include also nodes 5-9. 

In addition, the software package contains a set of utility code:

1. Generated scenarios
2. Ros msgs package
3. Utility functions
4. Code for general configuration of a scenario
5. Offline code (with rosbags)

**Keywords:** SLAM, BSP, multi-robot


## License
Autonomous Navigation and Perception Lab (ANPL),
Technion, Israel Institute of Technology,
Faculty of Aerospace Engineering,
Haifa, Israel, 32000
All Rights Reserved

See LICENSE for the license information


The **anpl_mrbsp** packages have been tested under [ROS](http://wiki.ros.org) Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation (from sources)

For easy installation, check installation scripts repository: [mrbsp_installation_scripts](https://github.com/anpl-technion/mrbsp_installation_scripts.git)

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [Boost 1.58](https://www.boost.org/users/history/version_1_58_0.html) (C++ library)
- [GTSAM 3.2.1](https://borg.cc.gatech.edu/) (belief space library)
- [PCL 1.8.1](https://github.com/PointCloudLibrary/pcl) (Point Cloud library)
- [OCTOMAP](https://octomap.github.io/) (Probabilistic 3D Mapping)
- [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library)
- [MAVROS](http://wiki.ros.org/mavros) (link between mavlin and ros)
- [diverse_short_paths](https://bitbucket.org/caleb_voss/diverse_short_paths) (find k short paths to the goal)
- [pioneer desccription](https://github.com/MobileRobots/amr-ros-config)
- [anpl ros infrastructure](https://bitbucket.org/ANPL/anpl_ros_infrastructur)
- [pioneer keyboard controller](https://bitbucket.org/ANPL/pioneer_keyop)
- [ros aria](https://github.com/amor-ros-pkg/rosaria) (connect pioneer robot)

***
***
#### Create Workspace

The code is build within a [catkin workspace](http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces).
Please note that ros tutorials suggest to use `catkin_make` to build the packages in the workspace. We on the other hand use the command `catkin build`.

**note:** If a catkin workspace is already created on your system, you can skip this part and build mrbsp ros packages in your workspace.


```
#!bash

PROJECT_NAME=anpl_mrbsp
WS_NAME=mrbsp_ws
WS_PATH=~/ANPL/infrastructure/$WS_NAME
PREFIX=/usr/ANPLprefix
#remove old anpl_mrbsp workspace
sudo rm -rf $WS_PATH

mkdir -p $WS_PATH/src && cd $WS_PATH
catkin init
catkin build

source $WS_PATH/devel/setup.bash
echo "source $WS_PATH/devel/setup.bash" >> ~/.bashrc  
```
***
***

#### Building

To build from source, clone the latest version from this repository into your catkin workspace, run several scripts (located in mrbsp_ros_utils package), and compile the package using


```
#!bash

PROJECT_NAME=anpl_mrbsp
WS_NAME=mrbsp_ws
WS_PATH=~/ANPL/infrastructure/$WS_NAME
PREFIX=/usr/ANPLprefix

cd $WS_PATH/src
git clone https://bitbucket.org/ANPL/pioneer_keyop
git clone https://bitbucket.org/ANPL/anpl_ros_infrastructur anpl_inf
git clone https://bitbucket.org/ANPL/anpl_mrbsp
git clone https://github.com/MobileRobots/amr-ros-config

#copy cmake files that node will find anpl libs.
cd $WS_PATH/src/anpl_mrbsp/mrbsp_utils/scripts/
sudo sh install-find-cmakes.sh

./install-rosaria.sh
./install-rotors-simulation.sh

cd $WS_PATH
catkin build
```

For both create workspace and build from scratch use the following scrip. 
Attention: the script delete the old workspace. so back up and take everything you need before run the script.


```
#!bash

install-mrbsp_ros-ws.sh  (NOT TESTSED YET)

```
***
***


## Usage
#### **Running real experiment with Pioneer robots:**

In order to use **anpl_mrbsp** a scenario needs to be defined.
This scenario should include [launch files](http://wiki.ros.org/roslaunch/XML) (XML files) for each participating robot and for a centralized machine (if needed).

Example (anpl_active_demo scenario):
  
**@ Central computer**  
if the central computer is in the same computer skip 2-4th step:  
1) Connect to ANPL_LINKSYS or ANPL_monster network(depends to what router the centralized computer is connected).  
2) In new terminal, open roscore:  
```bash

roscore

```
3) In new teminal, run master discovery:  
```bash

rosrun master_discovery_fkie master_discovery

```
4) In new teminal, run master sync:  
```bash
rosrun master_sync_fkie master_sync

```
* To skip 2-4 steps launch 
```bash

roscd anpl_active_demo/scripts
./roscore_master_discovery_synch.sh

```

5)  In new terminal(tab), before launching the centralized launch file, decide on number of robots. For this Example let us choose 2.  
```bash

NUM_ROB=2
only_change_num_of_robots.sh $NUM_ROB
Rob_str="A:=true B:=true  C:=false D:=false"
roslaunch anpl_active_demo centralized_laser_active_demo_pioneer.launch $Rob_str

```

**@ Robot A**  
if the central computer is in the same computer skip 2-4 th steps:  
1) Connect to ANPL_LINKSYS or ANPL_monster network(depends to what router the centralized computer is connected).  
2) In new terminal, open roscore:  
```bash

roscore

```
3) In new teminal, run master discovery:  
```bash

rosrun master_discovery_fkie master_discovery

```
4) In new teminal, run master sync:  
```bash

rosrun master_sync_fkie master_sync

```
* To skip 2-4 steps launch 
```bash

roscd anpl_active_demo/scripts
./roscore_master_discovery_synch.sh

```

5) Turn the robot on and connect the lidar to the battery:  
6) On new terminal go to the scenario folder and connect to the sensor onboard the robot:  
```bash

roscd anpl_active_demo/scripts
./run_robot_sensors_A.sh

```
7) Launch the odometry node:
```bash

roslaunch anpl_active_demo robot_setup_pioneer_A.launch

```
* To skip 6-7 steps launch 
```bash

roscd anpl_active_demo/scripts
./robot_setup_pioneer.sh A 5

```
**note**:  
> Run on each robot with its own ID: A, B, ...  
> Second argument is the time delay between running different dependent group of nodes.   
> Controller must wait until its inputs are ready, otherwise Gazebo teleport the robot to origin.  
> Launch file does not guarantee the order of execution of nodes.

**@ Robot B**  
repeat step 1-5th like robot A. - not running centralize on the same computer.  
repeat step 6-7th change A to B respectively.

For more scenarios, see scenarios description.


#### **Running Gazebo simulation:**
1) Open Gazebo world: (for 2 robots)

```bash

NUM_ROB=2
only_change_num_of_robots.sh $NUM_ROB
INPUT_STR="A:=true B:=true  C:=false D:=false"
roslaunch anpl_active_demo pioneer3at_world.launch $INPUT_STR

```
2) Run in central computer the centralized nodes:

```bash

roslaunch anpl_active_demo centralized_laser_active_demo_gazebo.launch $INPUT_STR

```

3) Robot A:

```bash
roscd anpl_active_demo/scripts
./robot_setup_gazebo.sh A 5
```

4) Robot B:

```bash

roscd anpl_active_demo/scripts
./robot_setup_gazebo.sh B 5
```

* To skip 1-4 steps launch 

```bash

roscd anpl_active_demo/scripts
./lazy_gazebo_MR_full.sh 2

```
***
***
