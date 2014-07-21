/*
  seneka_pnp (pick and place)
  Author: Matthias Nösner  
*/
#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <iomanip> 

#include <opencv/cv.h>

#include <SerializeIO.h>
#include <seneka_pnp_tools.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/JointConstraint.h>


#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

#include <moveit/robot_state/link_state.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h> 

#include "seneka_pnp/getState.h"
#include "seneka_pnp/setTransition.h"
#include "seneka_pnp/setStop.h"

#include "actionlib_msgs/GoalStatusArray.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

#include <boost/thread/mutex.hpp>

class SenekaPickAndPlace
{

struct trajectory_execution_validation{
  unsigned int dual_flag;
  bool finished;
  bool success;
};

  
private:
  ros::NodeHandle node_handle_;

  ros::Subscriber subscr_;
  
  std::vector<std::vector<double> > teached_wayp_r, teached_wayp_l;

  std::string currentState_;
  std::string transition_;
  ros::ServiceServer service_getstate_,service_settransition_,service_setstop_;
  ros::ServiceClient service_client, service_gazebo, service_gazebo_get;

  move_group_interface::MoveGroup *group_r_;
  move_group_interface::MoveGroup *group_l_;
  move_group_interface::MoveGroup *group_both_;


  //tje = trajectory execution
  trajectory_execution_validation tje_validation_;  
  boost::mutex tje_lock_;
  boost::mutex transition_lock_;

public:
  //Constructor
  SenekaPickAndPlace(ros::NodeHandle& nh){
    node_handle_ = nh;
    init();
  }

  //Destructor
  ~SenekaPickAndPlace(){
  }

  void init(){
    
    //params
    transition_ = "";
    tje_lock_.lock();
    tje_validation_.dual_flag = 0; 
    tje_validation_.finished = false;
    tje_validation_.success = true;//must be true
    tje_lock_.unlock(); 

    //read params from parameter server
    node_handle_.param<std::string>("/seneka_pnp/start_state", currentState_, "gazebo_home");

    //services to call
    service_client = node_handle_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
    service_gazebo = node_handle_.serviceClient<gazebo_msgs::SetModelState> ("/gazebo/set_model_state");
    service_gazebo_get = node_handle_.serviceClient<gazebo_msgs::GetModelState> ("/gazebo/get_model_state");

    loadTeachedPoints(&teached_wayp_r,&teached_wayp_l);
    loadMoveGroups();
    mainLoop();
  }  
    
  //--------------------------------------------------------- Transitions------------------------------------------------------------------

  //TRANSITION: toHomeState
  bool toHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
    bool ret = false;

    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit::planning_interface::MoveGroup::Plan lplan, rplan, merged_plan;
    moveit::planning_interface::MoveGroup::Plan myPlan;

    group_l->setNamedTarget("lhome");
    group_r->setNamedTarget("rhome");   
    
    if(seneka_pnp_tools::multiplan(group_l,&myPlan)){
    	sleep(5.0);
    	group_l->asyncExecute(myPlan);
    	ret = monitorArmMovement(true,false);
    }

    //r
    if(ret){
    	ret = false;
    	if(seneka_pnp_tools::multiplan(group_r,&myPlan)){
    		sleep(5.0);
    		group_r->asyncExecute(myPlan);
    		ret = monitorArmMovement(false,true);
    	}
    }

    return ret;
  }

  //TRANSITION:avoidCollisionState
  //Moves the arm to a collision free state
  bool toCollisionFree(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
    bool ret = false;

    moveit::planning_interface::MoveGroup::Plan myPlan;

    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    std::vector<double> group_variable_values;
    std::vector<std::string> group_variable_names;
    group_variable_values = group_l->getCurrentJointValues();
    group_variable_names = group_l->getJoints();

    if(group_variable_values.size() != 6 || group_variable_names.size() != 6){
      ROS_ERROR("Number of joints must be 6");
    }
    
    //Gazebo Simulation -> Drive left arm out of initial collision pose
    group_variable_values[1] = -1.0;
    group_l->setJointValueTarget(group_variable_values);

    if(group_l->plan(myPlan)){
      sleep(5.0);
      group_l->asyncExecute(myPlan); 
      ret = monitorArmMovement(true,false);
    }

    return ret;
  }


  //moves the arms to the initial pickup pose
  bool toPreGrasp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

    bool ret = false;
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    moveit::planning_interface::MoveGroup::Plan lplan,rplan, merged_plan;
    
    group_l->setNamedTarget("lpregrasp");
    group_r->setNamedTarget("rpregrasp");
    
    if(!group_l->plan(lplan))
      return false;
      
    if(!group_r->plan(rplan))
      return false;

    merged_plan = seneka_pnp_tools::mergePlan(lplan,rplan);

    group_both->asyncExecute(merged_plan);
    ret = monitorArmMovement(true,true);

    return ret;
  }
  
  bool toPreGraspRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  bool ret = false;

	  ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	  moveit_msgs::DisplayTrajectory display_trajectory;

	  moveit::planning_interface::MoveGroup::Plan lplan, rplan, merged_plan;
	  moveit::planning_interface::MoveGroup::Plan myPlan;

	  group_l->setNamedTarget("lpregrasp-rear");
	  group_r->setNamedTarget("rpregrasp-rear");  

	  if(seneka_pnp_tools::multiplan(group_l,&myPlan)){
		  sleep(5.0);
		  group_l->asyncExecute(myPlan);
		  ret = monitorArmMovement(true,false);
	  }

	  //r
	  if(ret){
		  ret = false;
		  if(seneka_pnp_tools::multiplan(group_r,&myPlan)){
			  sleep(5.0);
			  group_r->asyncExecute(myPlan);
			  ret = monitorArmMovement(false,true);
		  }
	  }

	  return ret;
  }

  bool toPickedUpRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	    bool ret = true;
	    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	    
	    unsigned int used_handle_r = 2;//use the real handle id 1-6
	    unsigned int used_handle_l = 5;//use the real handle id 1-6
	    used_handle_r--;
	    used_handle_l--;

	    moveit::planning_interface::MoveGroup::Plan myPlan, mergedPlan;

	    group_r->setStartStateToCurrentState();      
	    group_l->setStartStateToCurrentState();
	    group_both->setStartStateToCurrentState();

	    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	    geometry_msgs::Pose target_pose2_r = group_r->getCurrentPose().pose;
	    geometry_msgs::Pose target_pose2_l = group_l->getCurrentPose().pose;

	    tje_lock_.lock(); 
	    sensornode node = seneka_pnp_tools::getSensornodePose();
	    tje_lock_.unlock(); 
	    if(!node.success)
	    	return false;
	    
	    ROS_INFO("handholds:%d", (int)node.handholds.size());

	    //set start state to start_pose
	    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	    robot_state::JointStateGroup* joint_state_group_r = kinematic_state->getJointStateGroup("right_arm_group");
	    robot_state::JointStateGroup* joint_state_group_l = kinematic_state->getJointStateGroup("left_arm_group");
	    
	    std::vector<double> joint_values_r,joint_values_l;
	    joint_state_group_r->getVariableValues(joint_values_r);
	    joint_state_group_l->getVariableValues(joint_values_l);
		  
	    moveit_msgs::Constraints constraint_l, constraint_r;	  
	    moveit_msgs::JointConstraint joint_constraint_l, joint_constraint_r;
	    joint_constraint_l.joint_name = "left_arm_wrist_1_joint";
	    joint_constraint_l.position = joint_values_l[3];
	    joint_constraint_l.tolerance_above = M_PI/4;
	    joint_constraint_l.tolerance_below = M_PI/4;
	    joint_constraint_l.joint_name = "left_arm_wrist_3_joint";
	    joint_constraint_l.position = joint_values_l[5];
	    joint_constraint_l.tolerance_above = M_PI/8;
	    joint_constraint_l.tolerance_below = M_PI/8;
	    joint_constraint_r.joint_name = "right_arm_wrist_1_joint";
	    joint_constraint_r.position = joint_values_r[3];
	    joint_constraint_r.tolerance_above = M_PI/4;
	    joint_constraint_r.tolerance_below = M_PI/4;
	    joint_constraint_r.joint_name = "right_arm_wrist_3_joint";
	    joint_constraint_r.position = joint_values_r[5];
	    joint_constraint_r.tolerance_above = M_PI/8;
	    joint_constraint_r.tolerance_below = M_PI/8;		  

	    constraint_r.joint_constraints.push_back(joint_constraint_r);
		constraint_l.joint_constraints.push_back(joint_constraint_l);
		
		group_r->setPathConstraints(constraint_r);
		group_l->setPathConstraints(constraint_l);	      
	    //------------------------Pickup Position/Orientation -------------------------------------------------
	    waypoints_r.clear();
	    waypoints_l.clear();

	    target_pose2_r.position.x = node.handholds[used_handle_r].entry.translation.x;
	    target_pose2_r.position.y = node.handholds[used_handle_r].entry.translation.y;
	    target_pose2_r.position.z = node.handholds[used_handle_r].entry.translation.z;
	    waypoints_r.push_back(target_pose2_r);
	    target_pose2_r.orientation.w = node.handholds[used_handle_r].entry.rotation.w;
	    target_pose2_r.orientation.x = node.handholds[used_handle_r].entry.rotation.x;
	    target_pose2_r.orientation.y = node.handholds[used_handle_r].entry.rotation.y;
	    target_pose2_r.orientation.z = node.handholds[used_handle_r].entry.rotation.z;
	    waypoints_r.push_back(target_pose2_r);

	    target_pose2_l.position.x = node.handholds[used_handle_l].entry.translation.x;
	    target_pose2_l.position.y = node.handholds[used_handle_l].entry.translation.y;
	    target_pose2_l.position.z = node.handholds[used_handle_l].entry.translation.z;
	    waypoints_r.push_back(target_pose2_r);
	    target_pose2_l.orientation.w = node.handholds[used_handle_l].entry.rotation.w;
	    target_pose2_l.orientation.x = node.handholds[used_handle_l].entry.rotation.x;
	    target_pose2_l.orientation.y = node.handholds[used_handle_l].entry.rotation.y;
	    target_pose2_l.orientation.z = node.handholds[used_handle_l].entry.rotation.z;
	    waypoints_l.push_back(target_pose2_l);

	    mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01,1000);
	    
	    group_both->asyncExecute(mergedPlan);
	    ret = monitorArmMovement(true,true);  
	   	    
	    //------------------------PICK UP-----------------------------------------------------------------------------
	    if(ret){
	      waypoints_r.clear();
	      waypoints_l.clear();

	      target_pose2_r = group_r->getCurrentPose().pose;
	      target_pose2_l = group_l->getCurrentPose().pose;

	      target_pose2_r.position.x = node.handholds[used_handle_r].down.translation.x;
	      target_pose2_r.position.y = node.handholds[used_handle_r].down.translation.y;
	      target_pose2_r.position.z = node.handholds[used_handle_r].down.translation.z;
	      target_pose2_r.orientation.w = node.handholds[used_handle_r].entry.rotation.w;
	      target_pose2_r.orientation.x = node.handholds[used_handle_r].entry.rotation.x;
	      target_pose2_r.orientation.y = node.handholds[used_handle_r].entry.rotation.y;
	      target_pose2_r.orientation.z = node.handholds[used_handle_r].entry.rotation.z;
	      waypoints_r.push_back(target_pose2_r);
	      target_pose2_r.position.x = node.handholds[used_handle_r].up.translation.x;
	      target_pose2_r.position.y = node.handholds[used_handle_r].up.translation.y;
	      target_pose2_r.position.z = node.handholds[used_handle_r].up.translation.z;
	      waypoints_r.push_back(target_pose2_r);

	      target_pose2_l.position.x = node.handholds[used_handle_l].down.translation.x;
	      target_pose2_l.position.y = node.handholds[used_handle_l].down.translation.y;
	      target_pose2_l.position.z = node.handholds[used_handle_l].down.translation.z;
	      target_pose2_l.orientation.w = node.handholds[used_handle_l].entry.rotation.w;
	      target_pose2_l.orientation.x = node.handholds[used_handle_l].entry.rotation.x;
	      target_pose2_l.orientation.y = node.handholds[used_handle_l].entry.rotation.y;
	      target_pose2_l.orientation.z = node.handholds[used_handle_l].entry.rotation.z;
	      waypoints_l.push_back(target_pose2_l);
	      target_pose2_l.position.x = node.handholds[used_handle_l].up.translation.x;
	      target_pose2_l.position.y = node.handholds[used_handle_l].up.translation.y;
	      target_pose2_l.position.z = node.handholds[used_handle_l].up.translation.z; 
	      waypoints_l.push_back(target_pose2_l);
	      
	      mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01,1000);
	      group_both->asyncExecute(mergedPlan);
	      ret = monitorArmMovement(true,true);
	    }
	  
	    group_r->clearPathConstraints();
	    group_l->clearPathConstraints();
	    
	    return ret;
  }
  
  bool toPickedUp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

    bool ret = true;
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    unsigned int used_handle_r = 2;//use the real handle id 1-6
    unsigned int used_handle_l = 5;//use the real handle id 1-6
    used_handle_r--;
    used_handle_l--;

    moveit::planning_interface::MoveGroup::Plan myPlan, mergedPlan;

    group_r->setStartStateToCurrentState();      
    group_l->setStartStateToCurrentState();
    group_both->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
    geometry_msgs::Pose target_pose2_r = group_r->getCurrentPose().pose;
    geometry_msgs::Pose target_pose2_l = group_l->getCurrentPose().pose;

    tje_lock_.lock(); 
    sensornode node = seneka_pnp_tools::getSensornodePose();
    tje_lock_.unlock(); 
    if(!node.success)
    	return false;
    ROS_INFO("handholds:%d", (int)node.handholds.size());
      
    //------------------------Pickup Position/Orientation -------------------------------------------------
    waypoints_r.clear();
    waypoints_l.clear();

    target_pose2_r.position.x = node.handholds[used_handle_r].entry.translation.x;
    target_pose2_r.position.y = node.handholds[used_handle_r].entry.translation.y;
    target_pose2_r.position.z = node.handholds[used_handle_r].entry.translation.z;
    waypoints_r.push_back(target_pose2_r);
    target_pose2_r.orientation.w = node.handholds[used_handle_r].entry.rotation.w;
    target_pose2_r.orientation.x = node.handholds[used_handle_r].entry.rotation.x;
    target_pose2_r.orientation.y = node.handholds[used_handle_r].entry.rotation.y;
    target_pose2_r.orientation.z = node.handholds[used_handle_r].entry.rotation.z;
    waypoints_r.push_back(target_pose2_r);
    waypoints_r.push_back(target_pose2_r);
    waypoints_r.push_back(target_pose2_r);
    waypoints_r.push_back(target_pose2_r);

    target_pose2_l.position.x = node.handholds[used_handle_l].entry.translation.x;
    target_pose2_l.position.y = node.handholds[used_handle_l].entry.translation.y;
    target_pose2_l.position.z = node.handholds[used_handle_l].entry.translation.z;
    waypoints_l.push_back(target_pose2_l);
    target_pose2_l.orientation.w = node.handholds[used_handle_l].entry.rotation.w;
    target_pose2_l.orientation.x = node.handholds[used_handle_l].entry.rotation.x;
    target_pose2_l.orientation.y = node.handholds[used_handle_l].entry.rotation.y;
    target_pose2_l.orientation.z = node.handholds[used_handle_l].entry.rotation.z;
    waypoints_l.push_back(target_pose2_l);
    waypoints_l.push_back(target_pose2_l); 
    waypoints_l.push_back(target_pose2_l);
    waypoints_l.push_back(target_pose2_l);

    mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01);
    
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);  


    //------------------------PICK UP-----------------------------------------------------------------------------
    if(ret){
      waypoints_r.clear();
      waypoints_l.clear();

      target_pose2_r = group_r->getCurrentPose().pose;
      target_pose2_l = group_l->getCurrentPose().pose;

      target_pose2_r.position.x = node.handholds[used_handle_r].down.translation.x;
      target_pose2_r.position.y = node.handholds[used_handle_r].down.translation.y;
      target_pose2_r.position.z = node.handholds[used_handle_r].down.translation.z;
      target_pose2_r.orientation.w = node.handholds[used_handle_r].entry.rotation.w;
      target_pose2_r.orientation.x = node.handholds[used_handle_r].entry.rotation.x;
      target_pose2_r.orientation.y = node.handholds[used_handle_r].entry.rotation.y;
      target_pose2_r.orientation.z = node.handholds[used_handle_r].entry.rotation.z;
      waypoints_r.push_back(target_pose2_r);
      target_pose2_r.position.x = node.handholds[used_handle_r].up.translation.x;
      target_pose2_r.position.y = node.handholds[used_handle_r].up.translation.y;
      target_pose2_r.position.z = node.handholds[used_handle_r].up.translation.z;
      waypoints_r.push_back(target_pose2_r);

      target_pose2_l.position.x = node.handholds[used_handle_l].down.translation.x;
      target_pose2_l.position.y = node.handholds[used_handle_l].down.translation.y;
      target_pose2_l.position.z = node.handholds[used_handle_l].down.translation.z;
      target_pose2_l.orientation.w = node.handholds[used_handle_l].entry.rotation.w;
      target_pose2_l.orientation.x = node.handholds[used_handle_l].entry.rotation.x;
      target_pose2_l.orientation.y = node.handholds[used_handle_l].entry.rotation.y;
      target_pose2_l.orientation.z = node.handholds[used_handle_l].entry.rotation.z;
      waypoints_l.push_back(target_pose2_l);
      target_pose2_l.position.x = node.handholds[used_handle_l].up.translation.x;
      target_pose2_l.position.y = node.handholds[used_handle_l].up.translation.y;
      target_pose2_l.position.z = node.handholds[used_handle_l].up.translation.z; 
      waypoints_l.push_back(target_pose2_l);
      
      mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.0007);
      group_both->asyncExecute(mergedPlan);
      ret = monitorArmMovement(true,true);
    }

    return ret;    
  }

  bool toPrePack(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
    bool ret = false;
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    moveit::planning_interface::MoveGroup::Plan lplan,rplan, mergedPlan;

    group_r->setStartStateToCurrentState();      
    group_l->setStartStateToCurrentState();
    group_both->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
    geometry_msgs::Pose current_pose_r = group_r->getCurrentPose().pose;
    geometry_msgs::Pose current_pose_l = group_l->getCurrentPose().pose;    
    
    ROS_INFO("blabla");
    group_l->setNamedTarget("lprepack");
    group_r->setNamedTarget("rprepack");
    
    geometry_msgs::Pose pose_l,pose_r;
    std::vector<double> joint_positions_l;
    std::vector<double> joint_positions_r;
    //prepack-----------------------------------
    //l
	joint_positions_l.push_back(1.10002);
	joint_positions_l.push_back(-1.41744);
	joint_positions_l.push_back(-4.82702);
	joint_positions_l.push_back(-0.0431126);
	joint_positions_l.push_back(-3.61312);
	joint_positions_l.push_back(3.36654);
    //r
    joint_positions_r.push_back(-1.10186);
    joint_positions_r.push_back(-1.72479);
    joint_positions_r.push_back(4.82816);
    joint_positions_r.push_back(-3.10249);
    joint_positions_r.push_back(-2.67068);
    joint_positions_r.push_back(-3.34081);    
    seneka_pnp_tools::fk_solver(&node_handle_, joint_positions_r, joint_positions_l, &pose_l, &pose_r);
     
    waypoints_l.clear();
    waypoints_r.clear();
    
    //waypoints_l.push_back(current_pose_l);
    waypoints_l.push_back(pose_l);
    
    //waypoints_r.push_back(current_pose_r);
    waypoints_r.push_back(pose_r);
    
    mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01);
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);
    //prepack---------------------------------------
    //packed-------------------------------------------------
    joint_positions_l.clear();
    joint_positions_r.clear();
    
    joint_positions_l.push_back(-0.170856);
    joint_positions_l.push_back(-2.37488);
    joint_positions_l.push_back(-3.99752);
    joint_positions_l.push_back(0.0871943);
    joint_positions_l.push_back(-4.88399);
    joint_positions_l.push_back(3.37079);
    //r
    joint_positions_r.push_back(0.169261);
    joint_positions_r.push_back(-0.76683);
    joint_positions_r.push_back(3.99794);
    joint_positions_r.push_back(-3.23071);
    joint_positions_r.push_back(-1.39957);
    joint_positions_r.push_back(-3.34166);   
    seneka_pnp_tools::fk_solver(&node_handle_, joint_positions_r, joint_positions_l, &pose_l, &pose_r);
     
    waypoints_l.clear();
    waypoints_r.clear();
    
    //waypoints_l.push_back(current_pose_l);
    waypoints_l.push_back(pose_l);
    
    //waypoints_r.push_back(current_pose_r);
    waypoints_r.push_back(pose_r);
    
    mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01);
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);
    //packed--------------------------------------------
    //deployed_front-------------------------------------------------
    joint_positions_l.clear();
    joint_positions_r.clear();
    
    joint_positions_l.push_back(0.164887);
    joint_positions_l.push_back(-2.05895);
    joint_positions_l.push_back(-3.62224);
    joint_positions_l.push_back(-0.604017);
    joint_positions_l.push_back(-4.54825);
    joint_positions_l.push_back(3.37011);
    //r
    joint_positions_r.push_back(-0.166678);
    joint_positions_r.push_back(-1.08266);
    joint_positions_r.push_back(3.62272);
    joint_positions_r.push_back(-2.53966);
    joint_positions_r.push_back(-1.73551);
    joint_positions_r.push_back(-3.34153);   
    seneka_pnp_tools::fk_solver(&node_handle_, joint_positions_r, joint_positions_l, &pose_l, &pose_r);
     
    waypoints_l.clear();
    waypoints_r.clear();
    
    //waypoints_l.push_back(current_pose_l);
    waypoints_l.push_back(pose_l);
    
    //waypoints_r.push_back(current_pose_r);
    waypoints_r.push_back(pose_r);
    
    mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l,0.01);
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);
    //deployed_front--------------------------------------------

    return ret;
  }
  
  bool toPrePackRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  bool ret = false;

	  ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	  moveit_msgs::DisplayTrajectory display_trajectory;

	  moveit::planning_interface::MoveGroup::Plan myPlan;

	  moveit_msgs::Constraints constraint;	  
	  moveit_msgs::JointConstraint joint_constraint;
	  joint_constraint.joint_name = "left_arm_shoulder_pan_joint";
	  joint_constraint.position = -1.55;
	  joint_constraint.tolerance_above = M_PI/4;
	  joint_constraint.tolerance_below = M_PI/4;
	  joint_constraint.joint_name = "right_arm_shoulder_pan_joint";
	  joint_constraint.position = 1.55;
	  joint_constraint.tolerance_above = M_PI/4;
	  joint_constraint.tolerance_below = M_PI/4;
//	  joint_constraint.joint_name = "left_arm_wrist_1_joint";
//	  joint_constraint.position = -1.52;
//	  joint_constraint.tolerance_above = M_PI/4;
//	  joint_constraint.tolerance_below = M_PI/4;
//	  joint_constraint.joint_name = "right_arm_wrist_1_joint";
//	  joint_constraint.position = joint_state_group_r[3];
//	  joint_constraint.tolerance_above = M_PI/4;
//	  joint_constraint.tolerance_below = M_PI/4;		  
			    
	  constraint.joint_constraints.push_back(joint_constraint);
	  //group_both->setPathConstraints(constraint);
	  	  	   
	  //joint Value Target
	  std::vector<double> joints_combined;
	  joints_combined.push_back(-1.558);//l----
	  joints_combined.push_back(-2.79);
	  joints_combined.push_back(1.12702);
	  joints_combined.push_back(-4.762);
	  joints_combined.push_back(0.012214);
	  joints_combined.push_back(3.532);
	  joints_combined.push_back(1.55604);//r----
	  joints_combined.push_back(-0.348553);
	  joints_combined.push_back(-1.12702);
	  joints_combined.push_back(1.50556);
	  joints_combined.push_back(-0.0127962);
	  joints_combined.push_back(-3.372);  
	  group_both->setJointValueTarget(joints_combined);
	  
	  //set pose Target from joints	  
//	  geometry_msgs::Pose pose_l,pose_r;
//	  std::vector<double> joint_positions_l;
//	  std::vector<double> joint_positions_r;
		  
//	  joint_positions_r.push_back(1.55604);
//	  joint_positions_r.push_back(-0.348553);
//	  joint_positions_r.push_back(-1.12702);
//	  joint_positions_r.push_back(1.50556);
//	  joint_positions_r.push_back(-0.0127962);
//	  joint_positions_r.push_back(2.91043);
//	  joint_positions_l.push_back(-1.558);
//	  joint_positions_l.push_back(-2.79);
//	  joint_positions_l.push_back(1.12702);
//	  joint_positions_l.push_back(1.52883);
//	  joint_positions_l.push_back(0.012214);
//	  joint_positions_l.push_back(-2.74885);
//
//	  seneka_pnp_tools::fk_solver(&node_handle_, joint_positions_r, joint_positions_l, &pose_l, &pose_r);
//	  group_both->setPoseTarget(pose_l, "left_arm_ee_link");
//	  group_both->setPoseTarget(pose_r, "right_arm_ee_link");	  
	  
	  if(seneka_pnp_tools::multiplan(group_both,&myPlan)){
		  sleep(5.0);
		  group_both->asyncExecute(myPlan);
		  ret = monitorArmMovement(true,true);
	  }
	  
	  if(ret){
		  
		  joints_combined.clear();

		  joints_combined.push_back(-1.60649);//l
		  joints_combined.push_back(-2.11752);
		  joints_combined.push_back(1.97315);
		  joints_combined.push_back(-6.082);
		  joints_combined.push_back(-0.0364981);
		  joints_combined.push_back(3.315);
		  joints_combined.push_back(1.60587);//r
		  joints_combined.push_back(-1.01243);
		  joints_combined.push_back(-1.98466);
		  joints_combined.push_back(2.98633);
		  joints_combined.push_back(0.0370389);
		  joints_combined.push_back(-3.32);
		  group_both->setJointValueTarget(joints_combined);
		  
		  if(seneka_pnp_tools::multiplan(group_both,&myPlan)){
			  sleep(5.0);
			  group_both->asyncExecute(myPlan);
			  ret = monitorArmMovement(true,true);
		  }		  
	  }
	  
	  if(ret){
		  
		  joints_combined.clear();
		  
		  joints_combined.push_back(-1.58925);//l
		  joints_combined.push_back(-1.69424);
		  joints_combined.push_back(1.79061);
		  joints_combined.push_back(-6.082);
		  joints_combined.push_back(-0.0193046);
		  joints_combined.push_back(3.26574);
		  joints_combined.push_back(1.58806);//r
		  joints_combined.push_back(-1.43113);
		  joints_combined.push_back(-1.80313);
		  joints_combined.push_back(3.21233);
		  joints_combined.push_back(0.019241);
		  joints_combined.push_back(-3.319);
		  group_both->setJointValueTarget(joints_combined);
		  
		  if(seneka_pnp_tools::multiplan(group_both,&myPlan)){
			  sleep(5.0);
			  group_both->asyncExecute(myPlan);
			  ret = monitorArmMovement(true,true);
		  }		  
	  }
	  
	  return ret;
  }
  //--------------------------------------------------------- Transitions------------------------------------------------------------------

  bool monitorArmMovement(bool left,bool right){

    ros::Subscriber subscr_l,subscr_r;
    bool dual_mode = false;

    if(left && right)
      dual_mode = true;

    tje_lock_.lock();
    tje_validation_.dual_flag = 0;
    tje_validation_.finished = false;
    tje_validation_.success = true;//muste be true
    tje_lock_.unlock();
   
    if(left)
      subscr_l = node_handle_.subscribe("/left_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);     
    if(right)
      subscr_r = node_handle_.subscribe("/right_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);     

    //both needed finished for single mode .. dual_flag for dual mode
    while(!tje_validation_.finished || (dual_mode && (tje_validation_.dual_flag < 2))){
    	ROS_INFO("WAITING FOR TRAJECTORY EXECUTION TO FINISH");
    	if(tje_validation_.success == false){//break while loop and stop execution when one arm controller fails
    		this->setStop();
    		break;
    	}
    }
    subscr_l = ros::Subscriber();//ugly way to unsubscribe
    subscr_r = ros::Subscriber();//ugly way to unsubscribe
      
    return tje_validation_.success;
  }  
  
  //workaround to check asynch trajectory execution
  //for simulation with gazebo use this topic /left_arm_controller/follow_joint_trajectory/result
  //TODO: add error handling (http://mirror.umd.edu/roswiki/doc/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html)
  void trajectoryStatus(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg){

    ROS_INFO("RESULT %i",msg->status.status);
    ROS_INFO("RESULT %i",msg->result.error_code);

    int status = msg->status.status;
    int error_code = msg->result.error_code;

    tje_lock_.lock();
    tje_validation_.dual_flag = tje_validation_.dual_flag + 1;
    if(status == actionlib_msgs::GoalStatus::SUCCEEDED || status == actionlib_msgs::GoalStatus::ABORTED){
      tje_validation_.success = true;
    } else {
      tje_validation_.success = false;
    }
    
    tje_validation_.finished = true;
    tje_lock_.unlock();    
  }

  move_group_interface::MoveGroup::Plan mergedPlanFromWaypoints(std::vector<geometry_msgs::Pose> &waypoints_r, std::vector<geometry_msgs::Pose> &waypoints_l, double eef_step, double jump_threshold = 1000.0){

	  double visualizationtime = 2;

	  move_group_interface::MoveGroup group_r("right_arm_group");
	  move_group_interface::MoveGroup group_l("left_arm_group");

	  moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
	  moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, mergedPlan;

	  group_r.setWorkspace (0, 0, 0, 5, 5, 5);
	  group_r.setStartStateToCurrentState();
	  group_r.setGoalOrientationTolerance(0.01);
	  group_r.setPlanningTime(10.0);

	  group_l.setWorkspace (0, 0, 0, 5, 5, 5);
	  group_l.setStartStateToCurrentState();
	  group_l.setGoalOrientationTolerance(0.01);
	  group_l.setPlanningTime(10.0);

	  double fraction_r = 0;
	  double fraction_l = 0;
	  uint attempts = 100;
	  //-------RIGHT-------------------------
	  for(uint i=0; fraction_r < 1.0 && i < attempts; i++){
		  fraction_r = group_r.computeCartesianPath(waypoints_r,
				  eef_step,  // eef_step
				  jump_threshold,   // jump_threshold
				  trajectory_r);
		  ROS_INFO("fraction_r: %f",fraction_r);
	  }
	  linear_plan_r.trajectory_ = trajectory_r;
	  sleep(visualizationtime);

	  //-------LEFT-------------------------
	  for(uint i=0; fraction_l < 1.0 && i < attempts; i++){
		  fraction_l = group_l.computeCartesianPath(waypoints_l,
				  eef_step,  // eef_step
				  jump_threshold,   // jump_threshold
				  trajectory_l);  
		  ROS_INFO("fraction_l :%f",fraction_l);
	  }      
	  linear_plan_l.trajectory_ = trajectory_l;
	  sleep(visualizationtime);

	  double spd = 0.1;
	  //moveit::planning_interface::MoveGroup::Plan scaled_plan_r = seneka_pnp_tools::scaleTrajSpeed(linear_plan_r,spd);
	  //moveit::planning_interface::MoveGroup::Plan scaled_plan_l = seneka_pnp_tools::scaleTrajSpeed(linear_plan_l,spd);

	  //mergedPlan = seneka_pnp_tools::mergePlan(scaled_plan_r,scaled_plan_l);
	  mergedPlan = seneka_pnp_tools::mergePlan(linear_plan_r,linear_plan_l);

	  return mergedPlan;
  }

  bool sensornodePosValid()
  {
	  tje_lock_.lock(); 
	  sensornode node = seneka_pnp_tools::getSensornodePose();
	  tje_lock_.unlock();
	  if(!node.success)
		  return false; 

	  double x_lower_bound =  1.3;
	  double x_upper_bound =  1.7;
	  double y_lower_bound = -0.2;
	  double y_upper_bound =  0.2;

	  ROS_INFO("Sensornode pose is X:%f Y:%f", node.pose.translation.x, node.pose.translation.y);
	  ROS_INFO("Bounds X: %f and %f", x_lower_bound, x_upper_bound);
	  ROS_INFO("Bounds Y: %f and %f", y_lower_bound, y_upper_bound);

	  if((x_lower_bound < node.pose.translation.x && node.pose.translation.x < x_upper_bound) &&
			  (y_lower_bound < node.pose.translation.y && node.pose.translation.y < y_upper_bound)){
		  return true;	  
	  }   

	  return false;
  }

  

  // ---------------------------------------- SET AND GET ------------------------------------------------------------------
  bool setSensornodeState(double x, double y)
  {
    gazebo_msgs::ModelState state;
    geometry_msgs::Pose pose;
    gazebo_msgs::SetModelState::Request req;
    gazebo_msgs::SetModelState::Response resp;

    gazebo_msgs::GetModelState::Request get_req;
    gazebo_msgs::GetModelState::Response get_resp;
    

    //set pose in world_dummy_link
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.01;

    state.reference_frame = "world_dummy_link";
    state.model_name = "sensornode";
    state.pose = pose;    

    req.model_state = state;
    service_gazebo.call(req,resp);   

    //get pose in world frame
    get_req.model_name = "sensornode";
    get_req.relative_entity_name = "world";
    service_gazebo_get.call(get_req,get_resp);

    pose = get_resp.pose;


    //set pose in world frame
    //use orientation in world frame (don't use "world_dummy_link")
    pose.orientation.w =  0.438058031738; 
    pose.orientation.x =  0.438036624618;
    pose.orientation.y = -0.555087759704;
    pose.orientation.z = -0.555073558504;

    state.reference_frame = "world";
    state.model_name = "sensornode";
    state.pose = pose;    

    req.model_state = state;
    service_gazebo.call(req,resp);    

    ROS_INFO("SetModelState : %u", resp.success);
  }
    
  std::string getTransition(){
    return transition_;
  } 
  // ---------------------------------------- SET AND GET ------------------------------------------------------------------

  
  //--------------------------------------- Services ----------------------------------------
  bool getState(seneka_pnp::getState::Request &req,
		  seneka_pnp::getState::Response &res)
  {
	  res.state = currentState_;
	  return true;
  }

  bool setTransition(seneka_pnp::setTransition::Request &req,
		  seneka_pnp::setTransition::Response &res)
  {
	  std::string transition_tmp = req.transition; 

	  if(transition_tmp.compare("toPickedUp") == 0){
		  if(!sensornodePosValid()){
			  res.transition = "Sensornode is not in a valid grab position";
			  transition_ = "";
			  return true;
		  }	  	  
	  }

	  res.transition = transition_tmp;
	  transition_ = transition_tmp;    
	  return true;
  }

  bool setStop(seneka_pnp::setStop::Request &req,
		  seneka_pnp::setStop::Response &res)
  {
	  this->setStop();

	  res.success = true;

	  return true;
  }

  bool setStop(){

	  transition_ = "";

	  group_l_->stop();
	  group_r_->stop();
	  group_both_->stop();

	  return true;
  }
  //--------------------------------------- Services ----------------------------------------

  
  
  
  //-----------------------------------STATE MACHINE--------------------------------------------------
  //STATES: home, gazebo_home, collision_free
  //TRANSITSIONS: avoidCollisionState
  std::string stateMachine(std::string currentState)
  {
	  std::string transition = getTransition();
      group_r_->setStartStateToCurrentState();      
	  group_l_->setStartStateToCurrentState();
	  group_both_->setStartStateToCurrentState();

	  
	  //---------GAZEBO_HOME----------------------------------
	  if(currentState.compare("gazebo_home") == 0){

		  //Transitions
		  if(transition.compare("toCollisionFree") == 0){
			  if(toCollisionFree(group_l_,group_r_,group_both_)){
				  return "collision_free";
			  } else {
				  return "unknown_state";
			  }
		  }      
		  return "gazebo_home" ;
	  }

	  //------COLLISION_FREE----------------------------------
	  else if(currentState.compare("collision_free") == 0){

		  //Transitions
		  if(transition.compare("toHome") == 0){
			  if(toHome(group_l_,group_r_,group_both_)){
				  return "home";
			  } else {
				  return "unknown_state";
			  }
		  }      
		  return "collision_free";
	  }

	  //------HOME-------------------------------------------
	  else if(currentState.compare("home") == 0){

		  //Transitions
		  if(transition.compare("toPreGrasp") == 0){
			  if(toPreGrasp(group_l_,group_r_,group_both_)){
				  return "pregrasp";
			  } else {
				  return "unknown_state";
			  }
		  }    
		  if(transition.compare("toPreGraspRear") == 0){
			  if(toPreGraspRear(group_l_,group_r_,group_both_)){
				  return "pregrasp-rear";
			  } else {
				  return "unknown_state";
			  }
		  }  
		  return "home";
	  }

	  //------PREGRASP-------------------------------------------
	  else if(currentState.compare("pregrasp") == 0){

		  //Transitions
		  if(transition.compare("toHome") == 0){
			  if(toHome(group_l_,group_r_,group_both_)){
				  return "home";
			  } else {
				  return "unknown_state";
			  }
		  }     

		  if(transition.compare("toPickedUp") == 0){
			  if(toPickedUp(group_l_,group_r_,group_both_)){
				  return "pickedup";
			  } else {
				  return "unknown_state";
			  }
		  } 

		  return "pregrasp";
	  }
	  
	  //------PREGRASP-REAR-------------------------------------------
	  else if(currentState.compare("pregrasp-rear") == 0){
		  
		  if(transition.compare("toPickedUpRear") == 0){
			  if(toPickedUpRear(group_l_,group_r_,group_both_)){
				  return "pickedup-rear";
			  } else {
				  return "unknown_state";
			  }
		  } 
		  
		  return "pregrasp-rear";
	  }

    //------PICKED_UP-------------------------------------------
    else if(currentState.compare("pickedup") == 0){

    	//Transitions
    	if(transition.compare("toHome") == 0){
    		if(toHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
    	}    
    	if(transition.compare("toPrePack") == 0){
    		if(toPrePack(group_l_,group_r_,group_both_)){
    			return "prepack";
    		} else {
    			return "unknown_state";
    		}
    	} 

    	return "pickedup";
    }
	  
	  //------PICKED_UP_REAR-------------------------------------------
    else if(currentState.compare("pickedup-rear") == 0){

    	//Transitions
    	if(transition.compare("toHome") == 0){
    		if(toHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
    	}    
    	if(transition.compare("toPreGraspRear") == 0){
    		if(toPreGraspRear(group_l_,group_r_,group_both_)){
    			return "pregrasp-rear";
    		} else {
    			return "unknown_state";
    		}
    	} 
    	if(transition.compare("toPrePackRear") == 0){
    		if(toPrePackRear(group_l_,group_r_,group_both_)){
    			return "prepack-rear";
    		} else {
    			return "unknown_state";
    		}
    	}
    	return "pickedup-rear";
    }

    //------PREPACK-------------------------------------------
    else if(currentState.compare("prepack") == 0){

      //Transitions
    	if(transition.compare("toHome") == 0){
    		if(toHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
      }    

    	return "prepack";
    }
	  
	  //------PREPACK-REAR-------------------------------------------
    else if(currentState.compare("prepack-rear") == 0){

    	//Transitions
    	if(transition.compare("toHome") == 0){
    		if(toHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
    	}    

    	return "prepack-rear";
    }



    //------UNKNOWN_STATE-------------------------------------------
    else if(currentState.compare("unknown_state") == 0){
    	//Debug
    	if(transition.compare("setToGH") == 0){
    		return "gazebo_home";
    	}      

    	if(transition.compare("setToHome") == 0){
    		return "home";
    	} 
    	if(transition.compare("setToCF") == 0){
    		return "collision_free";
    	}
    	//Debug

    	return "unknown_state";
    }

    else{
    	return "unknown_state";
    }

    return "unknown_state";
  }
  //-----------------------------------STATE MACHINE--------------------------------------------------
  
  //--------------------------------------- Load on Init -----------------------------------------------------------------------------------------
  void loadTeachedPoints(std::vector<std::vector<double> >* vec_r,std::vector<std::vector<double> >* vec_l){

	  SerializeIO *ser = new SerializeIO("/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_pnp/common/teached_dual_arm_movement.def",'i');
	  std::vector<std::vector<double> > tmp;

	  ser->openArray("dual_arm_movement");
	  ser->readArray("dual_arm_movement",&tmp);
	  ser->closeArray();
	  ser->close();

	  for(unsigned int i = 0; i < tmp.size(); i++){

		  if((i%2)==0){
			  vec_r->push_back(tmp[i]);
		  } else {
			  vec_l->push_back(tmp[i]);
		  }
	  }

	  delete ser;

  }	
  
  void loadMoveGroups()
  {
    
    group_r_ =  new move_group_interface::MoveGroup("right_arm_group");
    group_l_ =  new move_group_interface::MoveGroup("left_arm_group");
    group_both_ = new move_group_interface::MoveGroup("both_arms");

    //planning settings
    double planning_time = 10.0;
    double orientation_tolerance = 0.01;
    double position_tolerance = 0.01;

    group_r_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_r_->setStartStateToCurrentState();
    group_l_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_l_->setStartStateToCurrentState();
    group_both_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_both_->setStartStateToCurrentState();

    group_r_->setGoalOrientationTolerance(orientation_tolerance);
    group_r_->setGoalPositionTolerance(position_tolerance);
    group_r_->setPlanningTime(planning_time);
    group_l_->setGoalOrientationTolerance(orientation_tolerance);
    group_l_->setGoalPositionTolerance(position_tolerance);
    group_l_->setPlanningTime(planning_time);
    group_both_->setGoalOrientationTolerance(orientation_tolerance);
    group_both_->setGoalPositionTolerance(position_tolerance);
    group_both_->setPlanningTime(planning_time);

  }
  //--------------------------------------- Load on Init ------------------------------------
  //main loop
  //check Sensornode position and start planning
  void mainLoop(){

    service_getstate_ = node_handle_.advertiseService("seneka_pnp/getState", &SenekaPickAndPlace::getState, this);
    service_settransition_ = node_handle_.advertiseService("seneka_pnp/setTransition", &SenekaPickAndPlace::setTransition, this);
    service_setstop_ = node_handle_.advertiseService("seneka_pnp/setStop", &SenekaPickAndPlace::setStop, this);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Rate loop_rate(1);
    while(ros::ok()){
      
      currentState_ = stateMachine(currentState_);
      ROS_INFO("Current state: %s",currentState_.c_str());
      loop_rate.sleep();
    }
  }
};


int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "SenekaPickAndPlace");
    ros::NodeHandle nh;

    /// Create SenekaPickAndPlace instance with mainLoop inside
    SenekaPickAndPlace* seneka_pnp = new SenekaPickAndPlace(nh);
    
    delete seneka_pnp;
    return 0;
}
