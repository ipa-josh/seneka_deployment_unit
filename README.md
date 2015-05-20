SENEKA_DEPLOYMENT_UNIT
======================
This repository contains source code for the UGV Quanjo Dual Arm Pickup/Deploy process at Fraunhofer IPA used in the SENEKA PROJECT. Two UR10 arms from Universal Robot where synchronized to lift a 25Kg Sensornode. The detection process of the sensornode is done through visual markers (Fast Pi-Tag) placed on the surface of the Sensornode.

It was created by Matthias Nösner (matthiasnoesner@viphibian.com).

The repository is made to work with ros groovy and gazebo version 1.5
The seneka project uses the cob_fiducial package for marker detection. A documentation of cob_fiducials can be found at http://wiki.ros.org/cob_fiducials

----------------------------------------------------------------------

PACKAGES:
=========
* seneka_pnp -> pick and place state machine for picking up the sensorsonde
* seneka_sensornode_detection -> Brings up the cob_fiducial node and manages the detection process. It also publish all the necessary tf positions of the sensorsonde.
* seneka_moveit_config
* seneka_interactive -> Simulation and Teaching (only used in simulation branch)
* seneka\_ugv\_description
* seneka\_scenarios -> controlling the sensornode (talk to Joshua Hampp)

Getting started:
=========

##General:

When using this package consider it is subdivided into two logical units. Which practically means two branches.

**groovy\_dev branch:**  
This branch works with the **universal\_robot/seneka\_quanjo\_real** branch, it is for running the real robot

**simulation branch:**  
This branch works with the **universal\_robot/seneka\_quanjo\_simulation** branch, it is used for simulation and teaching new joint states


After simulation and testing merge simulation to groovy_dev.

##Required:
---------
**.. for running the real robot**

* git clone https://github.com/ipa320/seneka_deployment_unit (use the **groovy-dev** branch)
* git clone https://github.com/equanox/cob_object_perception (**seneka_fiducials** branch) [rosbuild]

With **rosws update** you'll get these repos 
* git clone https://github.com/equanox/universal_robot  (use the **seneka_quanjo_real** branch) 
* git clone https://github.com/ipa320/cob_perception_common (make sure you are using the **groovy_dev_catkin** branch) 
* git clone https://github.com/Equanox/moveit_ros (use the **groovy-devel** branch) (lma kinematics ported to groovy)

##Additional Installation
* sudo apt-get install ros-groovy-pr2
* sudo apt-get install ros-groovy-pr2-simulator
* sudo apt-get install ros-groovy-pr2-controllers
* gazebo_msgs (e.g. clone to your catkin\_ws and compile  https://github.com/ros-simulation/gazebo_ros_pkgs) Hint: Only gazebo_msgs is important. Ignore the other packages
* sudo apt-get install ros-groovy-freenect-stack
* on ubuntu 12.04 ( sudo modprobe -r gspca_kinect )
* on ubuntu 12.04 ( echo 'blacklist gspca_kinect' | sudo tee -a /etc/modprobe.d/blacklist.conf )
* sudo apt-get install ros-groovy-rosbridge-suite

##0.Prerequisites
---------------------------------------------------------------------
To make your live easier u can find a bashrc file with predefined aliases. USE IT! Especially when working with the real robot.

##1. Configuration
---------------------------------------------------------------------
**These steps are only necessary if you use the ipa320 stack of cob_object_perception**

* Go to the launchfile of the cob_fiducial package **../cob_object_perception/cob_fiducials/ros/launch/fiducials.launch**.
Make sure that the rgb_topic argument points to the right image publisher.
E.g. for the Kinect use **/camera/rgb/**.

* Additionaly assure that the Fast PiTag markers are used (the different marker types are loaded using a .yaml file). See http://wiki.ros.org/cob_fiducials for details.

---------------------------------------------------------------------
* All the coordinate systems of the sensorsonde (marker,handle, grabpoints) can be adjusted in **../seneka_deployment_unit/seneka_sensornode_detection/common/sensorsonde_coordinates.def**.
* gripper length, depth and camera position is hardcoded in **../seneka_deployment_unit/seneka_sensornode_detection/src/sensornode_detection.cpp**

##2. Let's Go!
---------------------------------------------------------------------
* roslaunch seneka_ugv_description ugv_bringup.launch robot_ip_r:=192.168.1.11 robot_ip_l:=192.168.1.12 (connection for two ur10's with static ip's. The pc needs a static ip in the same subnet)
* roslaunch seneka_sensornode_detection sensornode_detection.launch (starts freenect,cob_fiducial and sensornode_detection nodes)
* rostopic hz /fiducials/detect_fiducials (start marker detection)
* roslaunch seneka_moveit_config move_group.launch (load the move group and moveit config)
* roslaunch seneka_moveit_config moveit_rviz.launch (visualization of planed trajectories)
* rosrun image_view image_view image:=/fiducials/image

##3. Live Config
----------------------------------------------
It is necessary to calibrate the camera before you can pickup a sensorsonde.
Place marker number 3 on the appropriate positon and type **start-calib** (assuming you are using the bashrc aliases).  
To avoid these step after each reinitialization you can store the parameters in **seneka\_sensonode\_detection/launch/calibration\_ext.yaml**.   
There you can also adjust the fixed point for calibration.  
To check if the calibration is valid use rviz for visual feedback. For finer adjustment u need to grab the sensosonde.  

##4. Pick Up / Deploy
---------------------------------------------------------------------
Start the seneka_pnp node (states are defined in seneka_pnp)  
**roslaunch seneka_pnp seneka_pnp.launch start_state:="home"**

To start a pickup/deploy process you can use a action   
**rostopic pub /seneka_pick_and_place/goal seneka_pnp/QuanjoManipulationActionGoal**  
val=1  pickup-front  
val=2  deploy-front  
val=11 pickup-rear   
val=12 deploy-rear  

REMEMBER: Watch at rviz and **HANDS ON THE RED BUTTONS**.

##5. Useful Hints and Troubleshooting
------------------------------------------------------

###Mass
The mass of the sensornode can be adjusted programmatically in **seneka_pnp/src/seneka_pnp.cpp**


###Arms Back To Home State
It may occure that the robot gets stuck.  
The Actions assume the robot to be in home state. DONT try to start from home state when the arms are NOT in home state.

**First option:** Direct it manually near home pose and restart.  
**Better option:** Use the predefined states to go back to home pose automatically!

Use **roslaunch seneka\_pnp seneka\_pnp.launch start\_state:="STATENAME"** to launch pnp process in a specific state.  
Use **rosservice call /seneka\_pnp/setTransition 'TRANSITIONNAME'**  to initiate a transition between two states.

#####Helpful States and Transitions
Stuck near home state **STATENAME = collision\_free -> TRANSITIONNAME = toHome**  
Stuck near front pack pose  **STATENAME = packed-front, TRANSITIONNAME = packedFrontToHome**  
Stuck while picking up for front position **STATENAME = pregrasp , TRANSITION = preGraspToHome**

#Simulation
---------------------------------------------------------------------
When starting the Simulation make sure to use the simulation/seneka_quanjo_simulation branches 
in the seneka_deployment_unit and universal_robot stacks.  

###Start
* roslaunch seneka_ugv_description ugv.launch
* roslaunch seneka_moveit_config move_group.launch
* roslaunch seneka_moveit_config moveit_rviz.launch
* rosrun seneka_interactive seneka_interactive

###Commands
Seneka Interactive provides multiple services for interaction

####Command List
#####getStates
Show all states defined in seneka_interactive.cpp  
Parameter: -  
Example: rosservice call /seneka_interactive/getStates
  

#####setStartState
set the Start state  
Parameter: statename(string)  
Example: rosservice call /seneka_interactive/setStartState home
  

#####setGoalState
set the Goal State (see setStartState)
  

#####simulate
simulates a path between a start and goal state  
Parameter:  
option(string)  
jointtarget - creates a jointtarget  
cartesian  - plans as a cartesian path  
posetarget - creates a pose target  
, iterations(int)  
Example: rosservice call /seneka_interactive/simulate jointtarget 2
  

#####generateIK
generates ik states for both arms and handles close to the start state  
Parameter: equaljoint_states(bool)  
Example: rosservice call /seneka_interactive/generateIK true
  

#####createNewState
create a new state (normally used after generateIK)  
Parameter: statename(string)  
Example: rosservice call /seneka_interactive/createNewState statename
  

#####printStatesToFile
prints all states to file (newest state at the end)  
Parameter: filename(string)  
Example: rosservice call /seneka_interactive/printStatesToFile filename
  


---------------------------------------------------------------------

* Author: Matthias Nösner(matthiasnoesner@viphibian.com)
* Readme updated: 13.11.2014

