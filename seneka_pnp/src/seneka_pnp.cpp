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
#include <moveit_msgs/GetPositionFK.h>
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

#include <geometry_msgs/Wrench.h>
#include <ur_driver/URSetPayload.h>

#include "seneka_pnp/getState.h"
#include "seneka_pnp/setTransition.h"
#include "seneka_pnp/setStop.h"

#include "actionlib_msgs/GoalStatusArray.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

#include <actionlib/server/simple_action_server.h>
#include <seneka_pnp/QuanjoManipulationAction.h>
#include <seneka_pnp/quanjo_manipulation.h>

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
  
  bool extforceflag_;
  
  bool trajexec_;
  double mass_, unloadmass_;
  double safety_duration_;
  double extforce_limit_;

  ros::Subscriber subscr_;
  ros::Subscriber subscr_result_l,subscr_result_r;
  std::vector<dualArmJointState> armstates_;
  
  std::vector<std::vector<double> > teached_wayp_r, teached_wayp_l;

  std::string currentState_;
  std::string transition_;
  ros::ServiceServer service_getstate_,service_settransition_,service_setstop_;
  ros::ServiceClient service_client, service_gazebo, service_gazebo_get, service_computefk;

  move_group_interface::MoveGroup *group_r_;
  move_group_interface::MoveGroup *group_l_;
  move_group_interface::MoveGroup *group_both_;


  //tje = trajectory execution
  trajectory_execution_validation tje_validation_;  
  boost::mutex tje_lock_;
  boost::mutex transition_lock_;
  boost::mutex extforce_lock_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<seneka_pnp::QuanjoManipulationAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  seneka_pnp::QuanjoManipulationFeedback feedback_;
  seneka_pnp::QuanjoManipulationResult result_;

public:
  //Constructor
  SenekaPickAndPlace(ros::NodeHandle& nh, std::string name) :
	  as_(nh, name, boost::bind(&SenekaPickAndPlace::quanjoArmSupervisorCB, this, _1), false),
	  action_name_(name)
  {
    node_handle_ = nh;
    init();
  }

  //Destructor
  ~SenekaPickAndPlace(){
  }
  
   void init(){
    
	as_.start();
	  
    //params
    transition_ = "";
    tje_lock_.lock();
    tje_validation_.dual_flag = 0; 
    tje_validation_.finished = false;
    tje_validation_.success = true;//must be true
    tje_lock_.unlock();
    
    mass_ = 24;
    unloadmass_ = 0;    
    extforce_limit_ = 35;//Nm
    
    safety_duration_ = 0.0;
    
    extforce_lock_.lock();
    extforceflag_ = false;
    extforce_lock_.unlock();
    
    //Allows trajectory execution.. !!!Be patient with this!!!
    trajexec_ = true;

    armstates_ = seneka_pnp_tools::createArmStates();
    
    //read params from parameter server
    node_handle_.param<std::string>("/seneka_pnp/start_state", currentState_, "gazebo_home");
    ROS_INFO("CURRENT STATE: %s", currentState_.c_str());
    

    //services to call
    service_client = node_handle_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
    service_computefk = node_handle_.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
    service_gazebo = node_handle_.serviceClient<gazebo_msgs::SetModelState> ("/gazebo/set_model_state");
    service_gazebo_get = node_handle_.serviceClient<gazebo_msgs::GetModelState> ("/gazebo/get_model_state");
    
    subscr_result_r = node_handle_.subscribe("/right_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);
    subscr_result_l = node_handle_.subscribe("/left_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this); 
    
    //set initial payload
    smoothSetPayload(unloadmass_);

    loadTeachedPoints(&teached_wayp_r,&teached_wayp_l);
    loadMoveGroups();
        
    mainLoop();
  }  

  //--------------------------------------------------------- ACTION ------------------------------------------------------------------
  void quanjoArmSupervisorCB(const seneka_pnp::QuanjoManipulationGoalConstPtr &goal)
  {
	  bool success = true;
	  bool checksuccess = false;
	  double sleepme = 3.0;

//	  for(int i=1; i<=100; i++)
//	  {
//		  // check that preempt has not been requested by the client
//		  if (as_.isPreemptRequested() || !ros::ok())
//		  {
//			  ROS_INFO("%s: Preempted", action_name_.c_str());
//			  // set the action state to preempted
//			  as_.setPreempted();
//			  success = false;
//			  break;
//		  }
//		  feedback_.percentage = i;
//		  as_.publishFeedback(feedback_);		  
//	  }
	  
	  boost::mutex lock;
	  lock.lock();
	  bool sensornodeposevalid = true;//sensornodePosValid();
	  lock.unlock();
	  
	  seneka_pnp::QuanjoManipulationResult  manipulation;
	  if(!currentState_.compare("home") && success){
		  		  
		  if(goal->goal.val == manipulation.result.GOAL_PICKUP_FRONT && sensornodeposevalid ){//GOAL_PICKUP_FRONT
			  
			  toPreGrasp(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  toPickedUp(group_l_,group_r_,group_both_);;
			  seneka_pnp_tools::keyPress();
			  
			  toPrePack(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  toPackedFront(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  packedFrontDrop(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  packedFrontToHome(group_l_,group_r_,group_both_);			  
		  }
		  else if(goal->goal.val == manipulation.result.GOAL_DEPLOY_FRONT){ //GOAL_DEPLOY_FRONT
			  
			  ROS_INFO("Received goal deploy front");
			  
			  toDeployFrontPreGrasp(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  deployFrontPickedUp(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  deployFront(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  deployFrontDrop(group_l_,group_r_,group_both_);
			  seneka_pnp_tools::keyPress();
			  
			  deployedFrontToHome(group_l_,group_r_,group_both_);
		  }
		  else if(goal->goal.val == manipulation.result.GOAL_PICKUP_REAR && sensornodeposevalid ) { // PICKUP REAR
			  
			  homeToPreGraspRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
			  
			  toPickedUpRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
			  
			  toPrePackRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
			  
			  toPackedRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
			  
			  packedRearDrop(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
		  	  
		  	  packedRearDropToHome(group_l_,group_r_,group_both_);
		  	  
		  	  
		  	  
//			  //homeToPreGraspRear
//			  //toPickedUpRear - critical -
//			  //toPrePackRear
//			  //toPackedRear
//			  //packedRearDrop - critical -
//			  //packedRearDropToHome
//			  ROS_INFO("Received goal pickup rear");
//			  if(success || !checksuccess)
//				  success = homeToPreGraspRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = toPickedUpRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = toPrePackRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = toPackedRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = packedRearDrop(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = packedRearDropToHome(group_l_,group_r_,group_both_);
			  
		  }
		  else if(goal->goal.val == manipulation.result.GOAL_DEPLOY_REAR) { // DEPLOY REAR
			  
			  homeToPackedRearDrop(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
			  
			  deployRearPickUp(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();

		  	  deployRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
		  	  
		  	  success = deployRearDrop(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
		  	  
			  deployRearToPreGraspRear(group_l_,group_r_,group_both_);
		  	  seneka_pnp_tools::keyPress();
		  	  
		  	  preGraspRearToHome(group_l_,group_r_,group_both_);
			  
//			  //homeToPackedRearDrop
//			  //deployRearPickUp - critical -
//			  //deployRear
//			  //deployRearDrop - critical -
//			  //deployRearToPreGraspRear
//			  //preGraspRearToHome  
//			  ROS_INFO("Received goal deploy rear");
//			  if(success || !checksuccess)
//				  success = homeToPackedRearDrop(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = deployRearPickUp(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = deployRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = deployRearDrop(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();
//			  if(success || !checksuccess)
//				  success = deployRearToPreGraspRear(group_l_,group_r_,group_both_);
//			  	  seneka_pnp_tools::keyPress();	
//			  if(success || !checksuccess)
//				  success = preGraspRearToHome(group_l_,group_r_,group_both_);
			  
		  }		  
		  
		  if(!success && checksuccess){
			  result_.result.val = manipulation.result.RESULT_HARD_MANIPULATION_FAILURE;
			  currentState_ = "unknown_state";
			  success = false;
		  }
		  
	  } else {
		  ROS_INFO("NOT IN HOME STATE");
		  result_.result.val = manipulation.result.RESULT_STARTSTATE_FAILURE;
		  success = false;
	  }
	  
	  if(success && checksuccess){
		  result_.result.val = manipulation.result.RESULT_SUCCESS;
	  }
	  
	  as_.setSucceeded(result_);
	  //result_.result.val = quanjo_manipulation::RESULT_SUCCESS;
  }
  //--------------------------------------------------------- ACTION ------------------------------------------------------------------
  
  
  //--------------------------------------------------------- Transitions------------------------------------------------------------------

  //TRANSITION: toHomeState
  bool toHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
	group_r_->setStartStateToCurrentState();      
	group_l_->setStartStateToCurrentState();
	group_both_->setStartStateToCurrentState();
  
    moveit::planning_interface::MoveGroup::Plan plan;
    dualArmJointState state;
    bool ret = false;
    
    if(!seneka_pnp_tools::getArmState(armstates_, "home", &state))
    	return false;
    
    group_r->setJointValueTarget(state.right.position);
    group_l->setJointValueTarget(state.left.position);    
    
    if(seneka_pnp_tools::multiplan(group_l,&plan)){
    	sleep(safety_duration_);
    	initTrajectoryMonitoring();
    	group_l->asyncExecute(plan);
    	ret = monitorArmMovement(true,false);
    }

    //r
    if(ret){
    	ret = false;
    	if(seneka_pnp_tools::multiplan(group_r, &plan)){
    		sleep(safety_duration_);
    		initTrajectoryMonitoring();
    		group_r->asyncExecute(plan);
    		ret = monitorArmMovement(false,true);
    	}
    }

    return ret;
  }
  
  //----------------------------------------------------- CRITICAL MOVES --------------------------------------------------------
  
  
  //packed-rear-drop -> packed-rear
  bool deployRearPickUp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose pose_l,pose_r;
	  
	  waypoints_l.clear();
	  waypoints_r.clear();

	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear", &state))
		  return false;

	  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	  waypoints_r.push_back(pose_r);
	  waypoints_l.push_back(pose_l);	        

	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true,true); 
	  //ret = monitorArmMovement(true,true,true); 
	  
	  extforce_lock_.lock();
	  bool extforceflag = extforceflag_;
	  extforce_lock_.unlock();
	  //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	  //check for external force and replan..
	  if(!ret && extforceflag){
		  smoothSetPayload(mass_/2);
		  
		  waypoints_l.clear();
		  waypoints_r.clear();

		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true); 		  
	  }
	  
	  //check distance to goal and replan 
	  if(!seneka_pnp_tools::checkGoalDistance("packed-rear", armstates_,group_r_, group_l_, group_both_)){
		  
		  smoothSetPayload(mass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	
		  
		  ret = seneka_pnp_tools::checkGoalDistance("packed-rear", armstates_, group_r_, group_l_, group_both_);
	  }
	  
	  return ret;
  }
  
  //deploy-rear -> deploy-rear-drop
  bool deployRearDrop(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose pose_l,pose_r;
	  
	  waypoints_l.clear();
	  waypoints_r.clear();

	  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-rear-drop", &state))
		  return false;

	  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	  waypoints_r.push_back(pose_r);
	  waypoints_l.push_back(pose_l);	        

	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both, waypoints_r, waypoints_l, 0.01);
	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true,true);
	  
	  extforce_lock_.lock();
	  bool extforceflag = extforceflag_;
	  extforce_lock_.unlock();
	  //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	  //check for external force and replan..
	  if(!ret && extforceflag){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-rear-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both, waypoints_r, waypoints_l, 0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);
	  }
	  
	  //check distance to goal and replan 
	  if(!seneka_pnp_tools::checkGoalDistance("deploy-rear-drop", armstates_,group_r_, group_l_, group_both_)){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-rear-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	
		  
		  ret = seneka_pnp_tools::checkGoalDistance("deploy-rear-drop", armstates_, group_r_, group_l_, group_both_);
	  }
	  
	  return ret;
  }
  
  //packed-rear -> packed-rear-drop
  bool packedRearDrop(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose pose_l,pose_r;
	  
	  waypoints_l.clear();
	  waypoints_r.clear();

	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-drop", &state))
		  return false;

	  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	  waypoints_r.push_back(pose_r);
	  waypoints_l.push_back(pose_l);	        

	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true,true);
	  
	  extforce_lock_.lock();
	  bool extforceflag = extforceflag_;
	  extforce_lock_.unlock();
	  //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	  //check for external force and replan..
	  if(!ret && extforceflag){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  waypoints_l.clear();
		  waypoints_r.clear();

		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);

	  }
	  
	  //check distance to goal and replan 
	  if(!seneka_pnp_tools::checkGoalDistance("packed-rear-drop", armstates_,group_r_, group_l_, group_both_)){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	
		  
		  ret = seneka_pnp_tools::checkGoalDistance("packed-rear-drop", armstates_, group_r_, group_l_, group_both_);
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

	    moveit::planning_interface::MoveGroup::Plan mergedPlan;

	    group_r->setStartStateToCurrentState();      
	    group_l->setStartStateToCurrentState();
	    group_both->setStartStateToCurrentState();

	    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	    geometry_msgs::Pose target_pose_r = group_r->getCurrentPose().pose;
	    geometry_msgs::Pose target_pose_l = group_l->getCurrentPose().pose;

	    bool checkInaccuracy = false;
	    sensornode node;
	    
	    tje_lock_.lock();
	    checkInaccuracy = seneka_pnp_tools::compensateInaccuracyDO(node_handle_);
	    tje_lock_.unlock();
	    
	    tje_lock_.lock();
	    node = seneka_pnp_tools::getSensornodePose();
	    tje_lock_.unlock(); 
	    
	    tje_lock_.lock();
	    checkInaccuracy = seneka_pnp_tools::compensateInaccuracyUNDO(node_handle_);
	    tje_lock_.unlock();
	    
	    if(!node.success || !checkInaccuracy)
	    	return false;
	    
	    std::vector<double> joint_values_r,joint_values_l;
	    std::vector<std::string> joint_names_r,joint_names_l;
	    
	    joint_values_r = group_r->getCurrentJointValues();
	    joint_values_l = group_l->getCurrentJointValues();
	    joint_names_r = group_r->getJoints();
	    joint_names_l = group_l->getJoints();
		  
	    for(uint i = 0; i < joint_values_r.size(); i++ )
	    	 ROS_INFO("%f",joint_values_r[i]);
	    for(uint i = 0; i < joint_values_l.size(); i++ )
	    	ROS_INFO("%f",joint_values_l[i]);
	    
    
	    moveit_msgs::Constraints constraint_r = seneka_pnp_tools::generateIKConstraints("copy all", joint_names_r, joint_values_r, 2);
	    moveit_msgs::Constraints constraint_l = seneka_pnp_tools::generateIKConstraints("copy all", joint_names_l, joint_values_l, 2);
	    //------------------------Pickup Position/Orientation -------------------------------------------------
	    std::vector<double> joint_positions_r = group_r->getCurrentJointValues();
	    std::vector<double> joint_positions_l = group_l->getCurrentJointValues();
	    
	    target_pose_r = group_r->getCurrentPose().pose;
	    target_pose_l = group_l->getCurrentPose().pose;	 
	    
	    target_pose_r.position = node.handholds[used_handle_r].entry.position;
	    target_pose_r.orientation = node.handholds[used_handle_r].entry.orientation;
	    target_pose_l.position = node.handholds[used_handle_l].entry.position;
	    target_pose_l.orientation = node.handholds[used_handle_l].entry.orientation;
	    
	    dual_arm_joints goal_joints = seneka_pnp_tools::generateIkSolutions(node_handle_, joint_positions_r, joint_positions_l, target_pose_r, target_pose_l, constraint_r, constraint_l);
	    
	    group_both->setJointValueTarget(goal_joints.both);	    
	    if(seneka_pnp_tools::multiplan(group_both,&mergedPlan)){
	    	initTrajectoryMonitoring();
	    	group_both->asyncExecute(mergedPlan);
	    	ret = monitorArmMovement(true,true);
	    }

	    //------------------------PICK UP REAR-----------------------------------------------------------------------------
	    
	    //down pose
	    if(ret){
	    	ret = false;	
	    	joint_positions_r = group_r->getCurrentJointValues();
	    	joint_positions_l = group_l->getCurrentJointValues();

	    	target_pose_r.position = node.handholds[used_handle_r].down.position;
	    	target_pose_r.orientation = node.handholds[used_handle_r].entry.orientation;
	    	target_pose_l.position = node.handholds[used_handle_l].down.position;
	    	target_pose_l.orientation = node.handholds[used_handle_l].entry.orientation;

	    	goal_joints = seneka_pnp_tools::generateIkSolutions(node_handle_, joint_positions_r, joint_positions_l, target_pose_r, target_pose_l, constraint_r, constraint_l);

	    	group_both->setJointValueTarget(goal_joints.both);
	    	if(seneka_pnp_tools::multiplan(group_both,&mergedPlan)){
	    		initTrajectoryMonitoring();	    		
	    		group_both->asyncExecute(mergedPlan);
	    		ret = monitorArmMovement(true,true);
	    	}

	    	//up pose
	    	if(ret){

	    		ret = false;
	    		joint_positions_r = group_r->getCurrentJointValues();
	    		joint_positions_l = group_l->getCurrentJointValues();

	    		target_pose_r.position = node.handholds[used_handle_r].up.position;
	    		target_pose_r.orientation = node.handholds[used_handle_r].entry.orientation;
	    		target_pose_l.position = node.handholds[used_handle_l].up.position;
	    		target_pose_l.orientation = node.handholds[used_handle_l].entry.orientation;

	    		goal_joints = seneka_pnp_tools::generateIkSolutions(node_handle_, joint_positions_r, joint_positions_l, target_pose_r, target_pose_l, constraint_r, constraint_l);

	    		group_both->setJointValueTarget(goal_joints.both);
	    		if(seneka_pnp_tools::multiplan(group_both,&mergedPlan)){
	    			
	    			mergedPlan = seneka_pnp_tools::scaleTrajSpeed(mergedPlan,0.5);//scale trajectory
	    			initTrajectoryMonitoring();
	    			group_both->asyncExecute(mergedPlan);
	    			ret = monitorArmMovement(true,true,true);
	    		} 	  	    

	    		extforce_lock_.lock();
	    		bool extforceflag = extforceflag_;
	    		extforce_lock_.unlock();
	    		//ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	    		//check for external force and replan..
	    		if(extforceflag){
	    			
	    			smoothSetPayload(mass_/2);
	    			//REPLAN
	    			
		    		joint_positions_r = group_r->getCurrentJointValues();
		    		joint_positions_l = group_l->getCurrentJointValues();

		    		target_pose_r.position = node.handholds[used_handle_r].up.position;
		    		target_pose_r.orientation = node.handholds[used_handle_r].entry.orientation;
		    		target_pose_l.position = node.handholds[used_handle_l].up.position;
		    		target_pose_l.orientation = node.handholds[used_handle_l].entry.orientation;

		    		goal_joints = seneka_pnp_tools::generateIkSolutions(node_handle_, joint_positions_r, joint_positions_l, target_pose_r, target_pose_l, constraint_r, constraint_l);
	    			
		    		group_both->setJointValueTarget(goal_joints.both);
		    		if(seneka_pnp_tools::multiplan(group_both,&mergedPlan)){
		    			initTrajectoryMonitoring();
		    			group_both->asyncExecute(mergedPlan);
		    			ret = monitorArmMovement(true,true);
		    		} 	    			
	    		}	    		
	    	}
	    }
   
	    return ret;
  }
  
  bool toPickedUp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("toPickedUp");

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
	  geometry_msgs::Pose target_pose_r = group_r->getCurrentPose().pose;
	  geometry_msgs::Pose target_pose_l = group_l->getCurrentPose().pose;

	  tje_lock_.lock(); 
	  sensornode node = seneka_pnp_tools::getSensornodePose();
	  tje_lock_.unlock(); 
	  if(!node.success)
		  return false;

	  //------------------------Pickup Position/Orientation -------------------------------------------------
	  waypoints_r.clear();
	  waypoints_l.clear();

	  target_pose_r.position.x = node.handholds[used_handle_r].entry.position.x;
	  target_pose_r.position.y = node.handholds[used_handle_r].entry.position.y;
	  target_pose_r.position.z = node.handholds[used_handle_r].entry.position.z;
	  waypoints_r.push_back(target_pose_r);
	  target_pose_r.orientation.w = node.handholds[used_handle_r].entry.orientation.w;
	  target_pose_r.orientation.x = node.handholds[used_handle_r].entry.orientation.x;
	  target_pose_r.orientation.y = node.handholds[used_handle_r].entry.orientation.y;
	  target_pose_r.orientation.z = node.handholds[used_handle_r].entry.orientation.z;
	  waypoints_r.push_back(target_pose_r);
	  waypoints_r.push_back(target_pose_r);
	  waypoints_r.push_back(target_pose_r);
	  waypoints_r.push_back(target_pose_r);

	  target_pose_l.position.x = node.handholds[used_handle_l].entry.position.x;
	  target_pose_l.position.y = node.handholds[used_handle_l].entry.position.y;
	  target_pose_l.position.z = node.handholds[used_handle_l].entry.position.z;
	  waypoints_l.push_back(target_pose_l);
	  target_pose_l.orientation.w = node.handholds[used_handle_l].entry.orientation.w;
	  target_pose_l.orientation.x = node.handholds[used_handle_l].entry.orientation.x;
	  target_pose_l.orientation.y = node.handholds[used_handle_l].entry.orientation.y;
	  target_pose_l.orientation.z = node.handholds[used_handle_l].entry.orientation.z;
	  waypoints_l.push_back(target_pose_l);
	  waypoints_l.push_back(target_pose_l);
	  waypoints_l.push_back(target_pose_l);
	  waypoints_l.push_back(target_pose_l);

	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);


	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true);


	  //------------------------PICK UP-----------------------------------------------------------------------------
	  if(ret){
		  waypoints_r.clear();
		  waypoints_l.clear();

		  target_pose_r = group_r->getCurrentPose().pose;
		  target_pose_l = group_l->getCurrentPose().pose;

		  target_pose_r.position.x = node.handholds[used_handle_r].down.position.x;
		  target_pose_r.position.y = node.handholds[used_handle_r].down.position.y;
		  target_pose_r.position.z = node.handholds[used_handle_r].down.position.z;
		  target_pose_r.orientation.w = node.handholds[used_handle_r].entry.orientation.w;
		  target_pose_r.orientation.x = node.handholds[used_handle_r].entry.orientation.x;
		  target_pose_r.orientation.y = node.handholds[used_handle_r].entry.orientation.y;
		  target_pose_r.orientation.z = node.handholds[used_handle_r].entry.orientation.z;
		  waypoints_r.push_back(target_pose_r);
		  target_pose_r.position.x = node.handholds[used_handle_r].up.position.x;
		  target_pose_r.position.y = node.handholds[used_handle_r].up.position.y;
		  target_pose_r.position.z = node.handholds[used_handle_r].up.position.z;
		  waypoints_r.push_back(target_pose_r);

		  target_pose_l.position.x = node.handholds[used_handle_l].down.position.x;
		  target_pose_l.position.y = node.handholds[used_handle_l].down.position.y;
		  target_pose_l.position.z = node.handholds[used_handle_l].down.position.z;
		  target_pose_l.orientation.w = node.handholds[used_handle_l].entry.orientation.w;
		  target_pose_l.orientation.x = node.handholds[used_handle_l].entry.orientation.x;
		  target_pose_l.orientation.y = node.handholds[used_handle_l].entry.orientation.y;
		  target_pose_l.orientation.z = node.handholds[used_handle_l].entry.orientation.z;
		  waypoints_l.push_back(target_pose_l);
		  target_pose_l.position.x = node.handholds[used_handle_l].up.position.x;
		  target_pose_l.position.y = node.handholds[used_handle_l].up.position.y;
		  target_pose_l.position.z = node.handholds[used_handle_l].up.position.z;
		  waypoints_l.push_back(target_pose_l);

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both, waypoints_r,waypoints_l,0.005);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true,true);//stop on external force

		  extforce_lock_.lock();
		  bool extforceflag = extforceflag_;
		  extforce_lock_.unlock();
		  //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

		  //check for external force and replan..
		  if(extforceflag){

			  waypoints_r.clear();
			  waypoints_l.clear();

			  target_pose_r = group_r->getCurrentPose().pose;
			  target_pose_l = group_l->getCurrentPose().pose;

			  target_pose_r.position.x = node.handholds[used_handle_r].up.position.x;
			  target_pose_r.position.y = node.handholds[used_handle_r].up.position.y;
			  target_pose_r.position.z = node.handholds[used_handle_r].up.position.z;
			  waypoints_r.push_back(target_pose_r);

			  target_pose_l.position.x = node.handholds[used_handle_l].up.position.x;
			  target_pose_l.position.y = node.handholds[used_handle_l].up.position.y;
			  target_pose_l.position.z = node.handholds[used_handle_l].up.position.z;
			  waypoints_l.push_back(target_pose_l);


		      smoothSetPayload(mass_/2);
		  
			  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both, waypoints_r,waypoints_l,0.005);
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(mergedPlan);
			  ret = monitorArmMovement(true,true);
		  }
		  ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);
	  }

	  return ret;
  }

  bool packedFrontDrop(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  extforce_limit_ = 45;

	  moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose pose_l,pose_r;

	  waypoints_l.clear();
	  waypoints_r.clear();

	  //------------packed-front-drop-----------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-drop", &state))
		  return false;

	  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	  waypoints_r.push_back(pose_r);
	  waypoints_l.push_back(pose_l);


	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.005);
	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true,true);	  
	  //------------packed-front-drop-----------------
	  
	  extforce_lock_.lock();
	  bool extforceflag = extforceflag_;
	  extforce_lock_.unlock();
	  ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	  //check for external force and replan..
	  if(!ret && extforceflag){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.005);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);
	  }
	  
	  //check distance to goal and replan 
	  if(!seneka_pnp_tools::checkGoalDistance("packed-front-drop", armstates_,group_r_, group_l_, group_both_)){
		  
		  smoothSetPayload(unloadmass_/2);
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-drop", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	
		  
		  ret = seneka_pnp_tools::checkGoalDistance("packed-front-drop", armstates_, group_r_, group_l_, group_both_);
	  }
	
	  extforce_limit_ = 35;	  

	  return ret;
  }
  
  bool deployFrontPickedUp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
		moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, mergedPlan;
		dualArmJointState state;
		bool ret = false;

	    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;

	    geometry_msgs::Pose pose_l,pose_r;
	    std::vector<double> joint_positions_l;
	    std::vector<double> joint_positions_r;


	    if(!seneka_pnp_tools::getArmState(armstates_, "packed-front", &state))
	    	return false;

	    seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	    waypoints_l.clear();
	    waypoints_r.clear();
	    waypoints_r.push_back(pose_r);
	    waypoints_l.push_back(pose_l);


	    mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	    initTrajectoryMonitoring();
	    group_both->asyncExecute(mergedPlan);
	    ret = monitorArmMovement(true,true,true);

	    extforce_lock_.lock();
	    bool extforceflag = extforceflag_;
	    extforce_lock_.unlock();
	    //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	    //check for external force and replan..
	    if(!ret && extforceflag){
	    	
	    	smoothSetPayload(mass_/2);
	    	
		    if(!seneka_pnp_tools::getArmState(armstates_, "packed-front", &state))
		    	return false;

		    seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		    waypoints_l.clear();
		    waypoints_r.clear();
		    waypoints_r.push_back(pose_r);
		    waypoints_l.push_back(pose_l);

		    mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		    initTrajectoryMonitoring();
		    group_both->asyncExecute(mergedPlan);
		    ret = monitorArmMovement(true,true);    	
	    }

	    //check distance to goal and replan 
	    if(!seneka_pnp_tools::checkGoalDistance("packed-front", armstates_,group_r_, group_l_, group_both_)){

	    	smoothSetPayload(mass_/2);

	    	if(!seneka_pnp_tools::getArmState(armstates_, "packed-front", &state))
	    		return false;

	    	seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	    	waypoints_l.clear();
	    	waypoints_r.clear();
	    	waypoints_r.push_back(pose_r);
	    	waypoints_l.push_back(pose_l);	        

	    	mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	    	initTrajectoryMonitoring();
	    	group_both->asyncExecute(mergedPlan);
	    	ret = monitorArmMovement(true,true);	

	    	ret = seneka_pnp_tools::checkGoalDistance("packed-front", armstates_, group_r_, group_l_, group_both_);
	    }
	    
	    return ret;
  }
  
  
  bool deployFrontDrop(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose current_pose_r = group_r->getCurrentPose().pose;
	  geometry_msgs::Pose current_pose_l = group_l->getCurrentPose().pose;    

	  geometry_msgs::Pose pose_l,pose_r;
	  std::vector<double> joint_positions_l;
	  std::vector<double> joint_positions_r;

	  ret = false;
	  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-front", &state))
		  return false;

	  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	  waypoints_l.clear();
	  waypoints_r.clear();
	  waypoints_r.push_back(pose_r);
	  waypoints_l.push_back(pose_l);	        

	  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	  initTrajectoryMonitoring();
	  group_both->asyncExecute(mergedPlan);
	  ret = monitorArmMovement(true,true,true);	    
	  
	  extforce_lock_.lock();
	  bool extforceflag = extforceflag_;
	  extforce_lock_.unlock();
	  //ROS_INFO("ret:%d extforceflag:%d",ret,extforceflag);

	  //check for external force and replan..
	  if(!ret && extforceflag){
		  smoothSetPayload(unloadmass_/2);
		  //REPLAN
		  
		  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-front", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	 
	  }
	  
	  //check distance to goal and replan 
	  if(!seneka_pnp_tools::checkGoalDistance("deploy-front", armstates_,group_r_, group_l_, group_both_)){

		  smoothSetPayload(unloadmass_/2);

		  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-front", &state))
			  return false;

		  seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		  waypoints_l.clear();
		  waypoints_r.clear();
		  waypoints_r.push_back(pose_r);
		  waypoints_l.push_back(pose_l);	        

		  mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(mergedPlan);
		  ret = monitorArmMovement(true,true);	

		  ret = seneka_pnp_tools::checkGoalDistance("deploy-front", armstates_, group_r_, group_l_, group_both_);
	  }
	    
	  return ret;
  }
  //----------------------------------------------------- CRITICAL MOVES --------------------------------------------------------
  
  //deploy-rear-drop -> pregrasp-rear
  bool deployRearToPreGraspRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  //---------deploy-rear-drop-free------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-rear-drop-free", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }

	  //---------pregrasp-rear------------
	  if(ret){
		  ret = false;

		  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear", &state))
			  return false;	  

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }
	  }

	  return ret;
  }

  //packed-rear -> deploy-rear
  bool deployRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	  dualArmJointState state;
	  bool ret = false;

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	  geometry_msgs::Pose pose_l,pose_r;

	  //---------packed-rear------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-h1", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);

		  //---------prepack-rear------------
		  if(ret){
			  ret = false;

			  if(!seneka_pnp_tools::getArmState(armstates_, "prepack-rear", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }

			  //---------deploy-rear------------
			  if(ret){
				  ret = false;

				  if(!seneka_pnp_tools::getArmState(armstates_, "deploy-rear", &state))
					  return false;	  

				  group_both->setJointValueTarget(state.both.position);	  
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_both->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }
			  }
		  }
	  }  
	  
	  return ret;
  }

  
  //packed-rear -> home
  bool packedRearDropToHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  //---------packed-rear-tidy1------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-1", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }
	
	  //---------packed-rear-tidy-2------------
	  if(ret){
		  ret = false;
	
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-2", &state))
			  return false;	  

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }
		  
		  //---------packed-rear-tidy-3------------
		  if(ret){
			  ret = false;

			  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-3", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }
			  
			  //---------packed-rear-tidy-4------------
			  if(ret){
				  ret = false;

				  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-4", &state))
					  return false;	  

				  group_both->setJointValueTarget(state.both.position);	  
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_both->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }
				  
				  //---------packed-rear-tidy-5------------
				  if(ret){
					  ret = false;

					  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-5", &state))
						  return false;	  

					  group_both->setJointValueTarget(state.both.position);	  
					  if(seneka_pnp_tools::multiplan(group_both,&plan)){
						  initTrajectoryMonitoring();
						  group_both->asyncExecute(plan);
						  ret = monitorArmMovement(true,true);
					  }
					  
					  //---------packed-rear-tidy-6------------
					  if(ret){
						  ret = false;

						  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-6", &state))
							  return false;	  

						  group_both->setJointValueTarget(state.both.position);	  
						  if(seneka_pnp_tools::multiplan(group_both,&plan)){
							  initTrajectoryMonitoring();
							  group_both->asyncExecute(plan);
							  ret = monitorArmMovement(true,true);
						  }
						  
						  //---------packed-rear-tidy-7------------
						  if(ret){
							  ret = false;

							  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-7", &state))
								  return false;	  

							  group_both->setJointValueTarget(state.both.position);	  
							  if(seneka_pnp_tools::multiplan(group_both,&plan)){
								  initTrajectoryMonitoring();
								  group_both->asyncExecute(plan);
								  ret = monitorArmMovement(true,true);
							  }
							  
							  //---------packed-rear-tidy-8------------
							  if(ret){
								  ret = false;

								  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-8", &state))
									  return false;	  

								  group_both->setJointValueTarget(state.both.position);	  
								  if(seneka_pnp_tools::multiplan(group_both,&plan)){
									  initTrajectoryMonitoring();
									  group_both->asyncExecute(plan);
									  ret = monitorArmMovement(true,true);
								  }
								  
								  //---------home------------
							      if(ret){
							    	ret = toHome(group_l,group_r,group_both);		
							      }	
							  }//8
						  }//7
					  }//6
				  }//5
			  }//4
		  }//3
	  }//2
	  
	  return ret;
  }	
  
  //home -> packed-rear-drop
  bool homeToPackedRearDrop(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  //---------packed-rear-tidy8------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-8", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }
	
	  //---------packed-rear-tidy-7------------
	  if(ret){
		  ret = false;
	
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-7", &state))
			  return false;	  

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }
		  
		  //---------packed-rear-tidy-6------------
		  if(ret){
			  ret = false;

			  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-6", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }
			  
			  //---------packed-rear-tidy-5------------
			  if(ret){
				  ret = false;

				  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-5", &state))
					  return false;	  

				  group_both->setJointValueTarget(state.both.position);	  
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_both->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }
				  
				  //---------packed-rear-tidy-4------------
				  if(ret){
					  ret = false;

					  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-4", &state))
						  return false;	  

					  group_both->setJointValueTarget(state.both.position);	  
					  if(seneka_pnp_tools::multiplan(group_both,&plan)){
						  initTrajectoryMonitoring();
						  group_both->asyncExecute(plan);
						  ret = monitorArmMovement(true,true);
					  }
					  
					  //---------packed-rear-tidy-3------------
					  if(ret){
						  ret = false;

						  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-3", &state))
							  return false;	  

						  group_both->setJointValueTarget(state.both.position);	  
						  if(seneka_pnp_tools::multiplan(group_both,&plan)){
							  initTrajectoryMonitoring();
							  group_both->asyncExecute(plan);
							  ret = monitorArmMovement(true,true);
						  }
						  
						  //---------packed-rear-tidy-2------------
						  if(ret){
							  ret = false;

							  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-2", &state))
								  return false;	  

							  group_both->setJointValueTarget(state.both.position);	  
							  if(seneka_pnp_tools::multiplan(group_both,&plan)){
								  initTrajectoryMonitoring();
								  group_both->asyncExecute(plan);
								  ret = monitorArmMovement(true,true);
							  }
							  
							  //---------packed-rear-tidy-1------------
							  if(ret){
								  ret = false;

								  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-tidy-1", &state))
									  return false;	  

								  group_both->setJointValueTarget(state.both.position);	  
								  if(seneka_pnp_tools::multiplan(group_both,&plan)){
									  initTrajectoryMonitoring();
									  group_both->asyncExecute(plan);
									  ret = monitorArmMovement(true,true);
								  }
								  
								  //---------packed-rear-drop------------
								  if(ret){
									  ret = false;

									  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-drop", &state))
										  return false;	  

									  group_both->setJointValueTarget(state.both.position);	  
									  if(seneka_pnp_tools::multiplan(group_both,&plan)){
										  initTrajectoryMonitoring();
										  group_both->asyncExecute(plan);
										  ret = monitorArmMovement(true,true);
									  }
								  }//drop
							  }//1
						  }//2
					  }//3
				  }//4
			  }//5
		  }//6
	  }//7
	  
	  return ret;
  }	
  

  
  
  bool deployFront(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("deployFront");
	  
	    moveit::planning_interface::MoveGroup::Plan plan, mergedPlan;
	    dualArmJointState state;
	    bool ret = false;
	    
	    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	    geometry_msgs::Pose current_pose_r = group_r->getCurrentPose().pose;
	    geometry_msgs::Pose current_pose_l = group_l->getCurrentPose().pose;    
	        
	    geometry_msgs::Pose pose_l,pose_r;
	    std::vector<double> joint_positions_l;
	    std::vector<double> joint_positions_r;
	    
	    waypoints_l.clear();
	    waypoints_r.clear();
	    
	    //------------prepack-----------------
	    if(!seneka_pnp_tools::getArmState(armstates_, "prepack", &state))
	    	return false;

	    seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
	    waypoints_r.push_back(pose_r);
	    waypoints_l.push_back(pose_l);


	    mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	    initTrajectoryMonitoring();
	    group_both->asyncExecute(mergedPlan);
	    ret = monitorArmMovement(true,true);
	    //------------prepack-----------------
	    
	    if(ret){   
	    	ret = false;
		    
	    	//------------pre-deploy-front-----------------
	    	if(!seneka_pnp_tools::getArmState(armstates_, "pre-deploy-front", &state))
	    		return false;

	    	seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
		    waypoints_l.clear();
		    waypoints_r.clear();
	    	waypoints_r.push_back(pose_r);
	    	waypoints_l.push_back(pose_l);	        

	    	mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
	    	initTrajectoryMonitoring();
	    	group_both->asyncExecute(mergedPlan);
	    	ret = monitorArmMovement(true,true);
	    	//------------pre-deploy-front-----------------
	    	
		    if(ret){   
		    	ret = false;
		    	
		    	//------------deploy-front-legs-down-----------------
		    	if(!seneka_pnp_tools::getArmState(armstates_, "deploy-front-legs-down", &state))
		    		return false;

		    	seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
			    waypoints_l.clear();
			    waypoints_r.clear();
		    	waypoints_r.push_back(pose_r);
		    	waypoints_l.push_back(pose_l);	        

		    	mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
		    	initTrajectoryMonitoring();
		    	group_both->asyncExecute(mergedPlan);
		    	ret = monitorArmMovement(true,true);
		    	//------------deploy-front-legs-down-----------------
		    }
	    }
	    return ret;	  
  }

  bool deployedFrontToHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("deployedFrontToHome");
	  
		moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, merged_plan;
		dualArmJointState state;
	    bool ret = false;
	    
	    if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-jointflip", &state))
	    	return false;
	    
	    group_r->setJointValueTarget(state.right.position);
	    group_l->setJointValueTarget(state.left.position);   
	        
	    if(!group_l->plan(lplan))
	      return false;
	      
	    if(!group_r->plan(rplan))
	      return false;

	    merged_plan = seneka_pnp_tools::mergePlan(lplan,rplan);

	    initTrajectoryMonitoring();
	    group_both->asyncExecute(merged_plan);
	    ret = monitorArmMovement(true,true);

	    if(ret){
	    	ret = false;
	    	//pregrasp
	    	if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-h1", &state))
	    		return false;	  

	    	group_both->setJointValueTarget(state.both.position);	  
	    	if(seneka_pnp_tools::multiplan(group_both,&plan)){
	    		initTrajectoryMonitoring();
	    		group_l->asyncExecute(plan);
	    		ret = monitorArmMovement(true,true);
	    	}
	    	
	    	if(ret){
	    		ret = toHome(group_l,group_r,group_both);		
	    	}	    	
	    }
	    return ret;
  }
  
  bool toDeployFrontPreGrasp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("toDeployFrontPreGrasp");

	  moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, merged_plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  //packed-front-tidy3
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-3", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }

	  if(ret){
		  ret = false;
		  //packed-front-tidy2
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-2", &state))
			  return false;	  

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }
		  
		  if(ret){
			  ret = false;
			  //packed-front-tidy1
			  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-1", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }
			  
			  if(ret){
				  ret = false;
				  //packed-front-drop
				  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-drop", &state))
					  return false;

				  group_both->setJointValueTarget(state.both.position);	  
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_both->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }
			  }
		  }
	  }	    
	  return ret;
  }
  
  //TRANSITION:avoidCollisionState
  //Moves the arm to a collision free state
  bool toCollisionFree(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("toCollisionFree");
    
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
      sleep(safety_duration_);
      initTrajectoryMonitoring();
      group_l->asyncExecute(myPlan); 
      ret = monitorArmMovement(true,false);
    }

    return ret;
  }


  //moves the arms to the initial pickup pose
  bool toPreGrasp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both) {
	  ROS_INFO("toPreGrasp");

	moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, merged_plan;
	dualArmJointState state;
    bool ret = true;
    
    if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-h1", &state))
    	return false;
    
    group_r->setJointValueTarget(state.right.position);
    group_l->setJointValueTarget(state.left.position);   
        
    if(!seneka_pnp_tools::multiplan(group_r,&rplan))
      return false;
      
    if(!seneka_pnp_tools::multiplan(group_l,&lplan))
      return false;

    merged_plan = seneka_pnp_tools::mergePlan(lplan,rplan);

    initTrajectoryMonitoring();
    group_both->asyncExecute(merged_plan);
    ret = monitorArmMovement(true,true);

    if(ret){
    	//pregrasp-jointflip
    	if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-jointflip", &state))
    		return false;	  

    	group_both->setJointValueTarget(state.both.position);	  
    	if(seneka_pnp_tools::multiplan(group_both,&plan)){
    		//sleep(10.0);
    		initTrajectoryMonitoring();
    		group_l->asyncExecute(plan);
    		ret = monitorArmMovement(true,true);
    	}
    }

    return ret;
  }
  
  bool packedFrontToHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("packedFrontToHome");

	  moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, merged_plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  //packed-front-tidy1
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-1", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_l->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }

	  if(ret){
		  ret = false;
		  //packed-front-tidy2
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-2", &state))
			  return false;	  

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_l->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }
		  
		  if(ret){
			  ret = false;
			  //packed-front-tidy3
			  if(!seneka_pnp_tools::getArmState(armstates_, "packed-front-tidy-3", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_l->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }

	                  if(ret){
        	                  ret = false;
                	          //packed-front-tidy3
                        	  if(!seneka_pnp_tools::getArmState(armstates_, "home", &state))
                                	  return false;

	                          group_both->setJointValueTarget(state.both.position);
        	                  if(seneka_pnp_tools::multiplan(group_both,&plan)){
                	                  initTrajectoryMonitoring();
                        	          group_both->asyncExecute(plan);
                                	  ret = monitorArmMovement(true,true);
	                          }
			}
			  
			  //if(ret){
			  //	  ret = toHome(group_l,group_r,group_both);				  
			  //}
		  }
	  }	    
	  return ret;
  }
  
  bool preGraspToHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;

	  //topregrasp
	  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-jointflip", &state))
		  return false;	  

	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }
	  
	  if(ret){
		  ret = false;

		  //topregrasp
	      if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-h1", &state))
          	return false;

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }  

		  if(ret){
			  ret = false;
			  //topregrasp
			  if(!seneka_pnp_tools::getArmState(armstates_, "home", &state))
				  return false;	  

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }  
		  }
	  }
	  
	  return ret;
  }
  
//  bool packedFrontToPreGrasp(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
//	  
//	  moveit::planning_interface::MoveGroup::Plan plan;
//	  dualArmJointState state;
//	  bool ret = false;
//	  
//	  return ret;
//  }
  
  bool homeToPreGraspRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;	  

	  //pregrasp-rear-h1
	  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h1", &state))
	      	return false;	  
	  
	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }

	  //pregrasp-rear-h2
	  if(ret){
		  ret = false;
		  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h2", &state))
		      	return false;
		  
		  group_both->setJointValueTarget(state.both.position);
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }

		  //pregrasp-rear-h3
		  if(ret){
			  ret = false;
			  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h3", &state))
			      	return false;
			  
			  group_both->setJointValueTarget(state.both.position);
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }

			  //pregrasp-rear-h4
			  if(ret){
				  ret = false;
				  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h4", &state))
					  return false;		

				  group_both->setJointValueTarget(state.both.position);				 
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_both->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }
				  
				  //pregrasp-rear-h5
				  if(ret){
					  ret = false;
					  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h5", &state))
						  return false;		

					  group_both->setJointValueTarget(state.both.position);				 
					  if(seneka_pnp_tools::multiplan(group_both,&plan)){
						  initTrajectoryMonitoring();
						  group_both->asyncExecute(plan);
						  ret = monitorArmMovement(true,true);
					  }
				  }
				  
				  //topregrasp-rear
				  if(ret){
					  ret = toPreGraspRearDual(group_l, group_r, group_both);
				  }
			  }
		  }
	  }  
	  return ret;
  }
	  
  bool toPreGraspRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  group_r->setStartStateToCurrentState();      
	  group_l->setStartStateToCurrentState();
	  group_both->setStartStateToCurrentState();
	  	  	  
	  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear", &state))
	      	return false;
	  
	  group_r->setJointValueTarget(state.right.position);
	  group_l->setJointValueTarget(state.left.position);
	  
	  if(seneka_pnp_tools::multiplan(group_l,&plan)){
		  sleep(safety_duration_);
		  initTrajectoryMonitoring();
		  group_l->asyncExecute(plan);
		  ret = monitorArmMovement(true,false);
	  }

	  //r
	  if(ret){
		  ret = false;
		  if(seneka_pnp_tools::multiplan(group_r,&plan)){
			  sleep(safety_duration_);
			  initTrajectoryMonitoring();
			  group_r->asyncExecute(plan);
			  ret = monitorArmMovement(false,true);
		  }
	  }

	  return ret;
  }
  
  //planning and executing both arms syncroniously
  bool toPreGraspRearDual(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
	  group_both->setStartStateToCurrentState();
	  	  	  
	  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear", &state))
	      	return false;	  
	  
	  group_both->setJointValueTarget(state.both.position);	
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  sleep(safety_duration_);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }

	  return ret;
  }

  bool smoothSetPayload(double payload){
	  
      //set the payload through service call
      ros::ServiceClient client_r = node_handle_.serviceClient<ur_driver::URSetPayload>("/right_arm_controller/ur_driver/setPayload");
      ros::ServiceClient client_l = node_handle_.serviceClient<ur_driver::URSetPayload>("/left_arm_controller/ur_driver/setPayload");
      ur_driver::URSetPayload srv_r, srv_l;
           
      srv_r.request.payload = payload/2;
      srv_l.request.payload = payload/2;
            
      if (!client_r.call(srv_r))
    	 return false;
      if (!client_l.call(srv_l))
    	 return false;
      
      sleep(2.0);

      srv_r.request.payload = payload;
      srv_l.request.payload = payload;

      if (!client_r.call(srv_r))
         return false;
      if (!client_l.call(srv_l))
         return false;


      
      return true;
  }

  bool toPrePack(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  ROS_INFO("toPrePack");
    
	moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, mergedPlan;
	dualArmJointState state;
	bool ret = false;

    group_r->setStartStateToCurrentState();      
    group_l->setStartStateToCurrentState();
    group_both->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
    geometry_msgs::Pose current_pose_r = group_r->getCurrentPose().pose;
    geometry_msgs::Pose current_pose_l = group_l->getCurrentPose().pose;    
        
    geometry_msgs::Pose pose_l,pose_r;
    std::vector<double> joint_positions_l;
    std::vector<double> joint_positions_r;
    
    waypoints_l.clear();
    waypoints_r.clear();
    //------------prepack-----------------
    if(!seneka_pnp_tools::getArmState(armstates_, "prepack", &state))
    	return false;
    
    seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
    waypoints_r.push_back(pose_r);
    waypoints_l.push_back(pose_l);
        
    
    mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
    initTrajectoryMonitoring();
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);
    //prepack---------------------------------------
   
    
    return ret;
  }
  
  bool toPackedFront(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
	moveit::planning_interface::MoveGroup::Plan plan, lplan,rplan, mergedPlan;
	dualArmJointState state;
	bool ret = false;

    group_r->setStartStateToCurrentState();      
    group_l->setStartStateToCurrentState();
    group_both->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
    geometry_msgs::Pose current_pose_r = group_r->getCurrentPose().pose;
    geometry_msgs::Pose current_pose_l = group_l->getCurrentPose().pose;    
        
    geometry_msgs::Pose pose_l,pose_r;
    std::vector<double> joint_positions_l;
    std::vector<double> joint_positions_r;
    
     
    waypoints_l.clear();
    waypoints_r.clear();
    //------------packed-front-----------------
    if(!seneka_pnp_tools::getArmState(armstates_, "packed-front", &state))
    	return false;
    
    seneka_pnp_tools::fk_solver(&node_handle_, state.right.position, state.left.position, &pose_l, &pose_r);
    waypoints_r.push_back(pose_r);
    waypoints_l.push_back(pose_l);
        
    
    mergedPlan = mergedPlanFromWaypoints(group_l, group_r, group_both,waypoints_r,waypoints_l,0.01);
    initTrajectoryMonitoring();
    group_both->asyncExecute(mergedPlan);
    ret = monitorArmMovement(true,true);
    //------------packed-front-----------------
  }
  
  bool packedRearToPreGraspRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
      
	  
      //------------to packed-rear-----------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear", &state))
	      	return false;
	  
	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  sleep(safety_duration_);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }
	  
	  if(ret){

		  //------------to packed-rear-h1-----------------
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-h1", &state))
			  return false;

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  sleep(safety_duration_);
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }

	  }
	  
	  if(ret){
		  ret = false;
	      //------------to prepack-rear-----------------
		  if(!seneka_pnp_tools::getArmState(armstates_, "prepack-rear", &state))
			  return false;

		  group_both->setJointValueTarget(state.both.position);	  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  sleep(safety_duration_);
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }

		  if(ret){
			  ret = false;
			  //------------to pregrasp-rear-----------------
			  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear", &state))
				  return false;

			  group_both->setJointValueTarget(state.both.position);	  
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  sleep(safety_duration_);
				  initTrajectoryMonitoring();
				  group_both->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }
		  }
	  }	  
	  return ret;	  
  }
  
  bool preGraspRearToHome(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;
	  
      //------------to pregrasp-rear-h5-----------------	  
	  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h5", &state))
	      	return false;		
	  
	  group_both->setJointValueTarget(state.both.position);				 
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  initTrajectoryMonitoring();
		  group_l->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }	  
	  
	  if(ret){
		  ret = false;	

		  //------------to pregrasp-rear-h4-----------------	  
		  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h4", &state))
			  return false;		

		  group_both->setJointValueTarget(state.both.position);				 
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  initTrajectoryMonitoring();
			  group_l->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }	  

		  if(ret){
			  ret = false;	
			  //------------to pregrasp-rear-h3-----------------	  
			  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h3", &state))
				  return false;		

			  group_both->setJointValueTarget(state.both.position);				 
			  if(seneka_pnp_tools::multiplan(group_both,&plan)){
				  initTrajectoryMonitoring();
				  group_l->asyncExecute(plan);
				  ret = monitorArmMovement(true,true);
			  }

			  if(ret){
				  ret = false;
				  //------------to pregrasp-rear-h2-----------------	  
				  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h2", &state))
					  return false;		

				  group_both->setJointValueTarget(state.both.position);				 
				  if(seneka_pnp_tools::multiplan(group_both,&plan)){
					  initTrajectoryMonitoring();
					  group_l->asyncExecute(plan);
					  ret = monitorArmMovement(true,true);
				  }

				  if(ret){
					  ret = false;
					  //------------to pregrasp-rear-h1-----------------	  
					  if(!seneka_pnp_tools::getArmState(armstates_, "pregrasp-rear-h1", &state))
						  return false;		

					  group_both->setJointValueTarget(state.both.position);				 
					  if(seneka_pnp_tools::multiplan(group_both,&plan)){
						  initTrajectoryMonitoring();
						  group_l->asyncExecute(plan);
						  ret = monitorArmMovement(true,true);
					  }

					  if(ret){
						  //---- to Home ------
						  ret = toHome(group_l,group_r,group_both);  
					  }
				  }
			  }
		  }
	  }

	  return ret;
  }
  
  
  
  bool toPrePackRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
	  
	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;	        
	  geometry_msgs::Pose pose_l,pose_r;
	  
	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;

      //------------to prepack-rear-----------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "prepack-rear", &state))
	      	return false;
	  
	  group_both->setJointValueTarget(state.both.position);	  
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
	  	  smoothSetPayload(mass_/2);
	  	  smoothSetPayload(mass_/2);
		  sleep(safety_duration_);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }	  
	  
	  return ret;
  }
  
  bool toPackedRear(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){

	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;	        
	  geometry_msgs::Pose pose_l,pose_r;

	  moveit::planning_interface::MoveGroup::Plan plan;
	  dualArmJointState state;
	  bool ret = false;

	  //------------to packed-rear-h1-----------------
	  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear-h1", &state))
		  return false;

	  group_both->setJointValueTarget(state.both.position);		   
	  if(seneka_pnp_tools::multiplan(group_both,&plan)){
		  sleep(safety_duration_);
		  initTrajectoryMonitoring();
		  group_both->asyncExecute(plan);
		  ret = monitorArmMovement(true,true);
	  }		  

	  //------------to packed-rear------------------
	  if(ret){
		  ret = false;
		  if(!seneka_pnp_tools::getArmState(armstates_, "packed-rear", &state))
			  return false;

		  group_both->setJointValueTarget(state.both.position);			  
		  if(seneka_pnp_tools::multiplan(group_both,&plan)){
			  sleep(safety_duration_);
			  initTrajectoryMonitoring();
			  group_both->asyncExecute(plan);
			  ret = monitorArmMovement(true,true);
		  }		
	  }
	  
	  return ret;
  }
  
  //--------------------------------------------------------- Transitions------------------------------------------------------------------

  bool extForceDetection(const geometry_msgs::Wrench::ConstPtr& msg){
	  
	  double force_z = std::sqrt(msg->force.z * msg->force.z);
	  
	  //force in N, torque in Nm
	  if(force_z > extforce_limit_){
		  extforce_lock_.lock();
		  extforceflag_ = true;
		  extforce_lock_.unlock();
	  }
	  return true;
  }
  
  void wrenchCBL(const geometry_msgs::Wrench::ConstPtr& msg){
	  ROS_INFO("LEFT ARM FORCE z:%f",msg->force.z);
	  extForceDetection(msg);
  }
  
  void wrenchCBR(const geometry_msgs::Wrench::ConstPtr& msg){
	  ROS_INFO("RIGHT ARM FORCE z:%f",msg->force.z);
	  extForceDetection(msg);
  }
  
  /* initTrajectoryMonitoring
   * 
   * Has to be called before the asyncExecute and monitorArmMovement
   * 
   * @param left enable monitoring for left arm
   * @param right enable monitoring for right arm
   * @param extforce enables checking for a external force
   * */
  void initTrajectoryMonitoring(){
	  
	    tje_lock_.lock();
	    tje_validation_.dual_flag = 0;
	    tje_validation_.finished = false;
	    tje_validation_.success = true;//must be true
	    tje_lock_.unlock();
	    
	    extforce_lock_.lock();
	    extforceflag_ = false;
	    extforce_lock_.unlock();
  }
  
  /* monitorArmMovement
   * 
   * This is the core function to monitor trajectory execution
   * 
   * @param left enable monitoring for left arm
   * @param right enable monitoring for right arm
   * @param extforce enables checking for a external force
   * */
  bool monitorArmMovement(bool left,bool right, bool extforce=false){
	  
    ros::Subscriber subscr_force_l, subscr_force_r;
    bool dual_mode = false;
        
    //check that at least one arm is set
    if(!(left || right))
    	return false;

    if(left && right)
    	dual_mode = true;
        
    if(left && extforce){
    	subscr_force_l = node_handle_.subscribe("/left_arm_controller/ur_driver/wrench", 1, &SenekaPickAndPlace::wrenchCBL, this);
    }
    if(right && extforce){
        subscr_force_r = node_handle_.subscribe("/right_arm_controller/ur_driver/wrench", 1, &SenekaPickAndPlace::wrenchCBR, this);
    }
    	
    ROS_INFO("WAITING FOR TRAJECTORY EXECUTION TO FINISH...");
    
    //both needed finished for single mode .. dual_flag for dual mode
    while(!tje_validation_.finished || (dual_mode && (tje_validation_.dual_flag < 2))){
   	
   		if(tje_validation_.success == false){//break while loop and stop execution when one arm controller fails
    		this->setStop();
    		break;
    	}
    	if(extforce){
    		extforce_lock_.lock();
    		bool extforceflag = extforceflag_;
    		extforce_lock_.unlock();
    		if(extforceflag){
        		this->setStop();
        		break;
    		}
     	}
    	ros::spinOnce();
    }
    
    ROS_INFO("TRAJECTORY EXECUTION IS FINISHED...");
    
    subscr_force_l = ros::Subscriber();//ugly way to unsubscribe
    subscr_force_r = ros::Subscriber();//ugly way to unsubscribe
      
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
    // -> gazebo sends aborted all the time || status == actionlib_msgs::GoalStatus::ABORTED
    if(status == actionlib_msgs::GoalStatus::SUCCEEDED){
      tje_validation_.success = true;
    } else {
      tje_validation_.success = false;
    }
    
    tje_validation_.finished = true;
    tje_lock_.unlock();    
  }

  //HERE_ME
  move_group_interface::MoveGroup::Plan mergedPlanFromWaypoints(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both, 
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	std::vector<geometry_msgs::Pose> &waypoints_r, std::vector<geometry_msgs::Pose> &waypoints_l, double eef_step, double jump_threshold = 10.0)
  {
//	  gazebo_msgs::ModelState state;
//	  geometry_msgs::Pose pose;
//	  gazebo_msgs::SetModelState::Request req;
//	  gazebo_msgs::SetModelState::Response resp;

	  double visualizationtime = 0;

	  moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
	  moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, mergedPlan;

	  double fraction_r = 0;
	  double fraction_l = 0;

	  group_r->setStartStateToCurrentState();      
	  group_l->setStartStateToCurrentState();
	  group_both->setStartStateToCurrentState();
		
	  uint attempts = 100;
	  //-------RIGHT-------------------------
	  for(uint i=0; fraction_r < 1.0 && i < attempts; i++){
		  fraction_r = group_r->computeCartesianPath(waypoints_r,
				  eef_step,  // eef_step
				  jump_threshold,   // jump_threshold
				  trajectory_r);
		  ROS_INFO("fraction_r: %f",fraction_r);
	  }
	  linear_plan_r.trajectory_ = trajectory_r;
	  sleep(visualizationtime);

	  //-------LEFT-------------------------
	  for(uint i=0; fraction_l < 1.0 && i < attempts; i++){
		  fraction_l = group_l->computeCartesianPath(waypoints_l,
				  eef_step,  // eef_step
				  jump_threshold,   // jump_threshold
				  trajectory_l);  
		  ROS_INFO("fraction_l :%f",fraction_l);
	  }      
	  linear_plan_l.trajectory_ = trajectory_l;
	  sleep(visualizationtime);

	  //double spd = 0.1;
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

	  double x_lower_bound =  1.45;
	  double x_upper_bound =  1.6;
	  double y_lower_bound = -0.2;
	  double y_upper_bound =  0.2;

	  ROS_INFO("Sensornode pose is X:%f Y:%f", node.pose.position.x, node.pose.position.y);
	  ROS_INFO("Bounds X: %f and %f", x_lower_bound, x_upper_bound);
	  ROS_INFO("Bounds Y: %f and %f", y_lower_bound, y_upper_bound);

	  if((x_lower_bound < node.pose.position.x && node.pose.position.x < x_upper_bound) &&
			  (y_lower_bound < node.pose.position.y && node.pose.position.y < y_upper_bound)){
		  return true;	  
	  }   

	  return false;
  }

  

  // ---------------------------------------- SET AND GET ------------------------------------------------------------------    
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

	  if(transition_tmp.compare("toPickedUp") == 0 || transition_tmp.compare("toPickedUpRear") == 0){
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
		  if(transition.compare("homeToPreGraspRear") == 0){
			  if(homeToPreGraspRear(group_l_,group_r_,group_both_)){
				  return "pregrasp-rear";
			  } else {
				  return "unknown_state";
			  }
		  }  
		  if(transition.compare("homeToPackedRearDrop") == 0){
			  if(homeToPackedRearDrop(group_l_,group_r_,group_both_)){
				  return "packed-rear";
			  } else {
				  return "unknown_state";
			  }
		  }  
		  return "home";
	  }

	  //------PREGRASP-------------------------------------------
	  else if(currentState.compare("pregrasp") == 0){

		  //Transitions
		  if(transition.compare("preGraspToHome") == 0){
			  if(preGraspToHome(group_l_,group_r_,group_both_)){
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
		  
		  if(transition.compare("toHome") == 0){
			  if(toHome(group_l_,group_r_,group_both_)){
				  return "home";
			  } else {
				  return "unknown_state";
			  }
		  }    
		  if(transition.compare("toPickedUpRear") == 0){
			  if(toPickedUpRear(group_l_,group_r_,group_both_)){
				  return "pickedup-rear";
			  } else {
				  return "unknown_state";
			  }
		  } 		  
		  if(transition.compare("preGraspRearToHome") == 0){
			  if(preGraspRearToHome(group_l_,group_r_,group_both_)){
				  return "home";
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
		  	  
		  return "pregrasp-rear";
	  }

	  //------PACKED-FRONT-------------------------------------------
	  else if(currentState.compare("packed-front") == 0){

		  //Transitions
		  if(transition.compare("packedFrontToHome") == 0){
			  if(packedFrontToHome(group_l_,group_r_,group_both_)){
				  return "home";
			  } else {
				  return "unknown_state";
			  }
		  } 

		  return "packed-front";
	  }

	  //------PACKED-REAR-------------------------------------------
	  else if(currentState.compare("packed-rear") == 0){
		  
		  if(transition.compare("packedRearToPreGraspRear") == 0){
			  if(packedRearToPreGraspRear(group_l_,group_r_,group_both_)){
				  return "pregrasp-rear";
			  } else {
				  return "unknown_state";
			  }
		  }     
		  
		  if(transition.compare("packedRearDropToHome") == 0){
			  if(packedRearDropToHome(group_l_,group_r_,group_both_)){
				  return "home";
			  } else {
				  return "unknown_state";
			  }
		  }      
		  
		  if(transition.compare("packedRearDrop") == 0){
			  if(packedRearDrop(group_l_,group_r_,group_both_)){
				  return "packed-rear-drop";
			  } else {
				  return "unknown_state";
			  }
		  }   
		  
		  if(transition.compare("deployRear") == 0){
			  if(deployRear(group_l_,group_r_,group_both_)){
				  return "deploy-rear";
			  } else {
				  return "unknown_state";
			  }
		  }     
		  
		  return "packed-rear";
	  }
	  
    else if(currentState.compare("deploy-rear") == 0){

    	//Transitions
    	if(transition.compare("deployRearDrop") == 0){
    		if(deployRearDrop(group_l_,group_r_,group_both_)){
    			return "deployed-rear";
    		} else {
    			return "unknown_state";
    		}
    	}    

    	return "deploy-rear";
    }
    else if(currentState.compare("deployed-rear") == 0){

    	//Transitions
    	if(transition.compare("toPreGraspRear") == 0){
    		if(toPreGraspRear(group_l_,group_r_,group_both_)){
    			return "pickedup-rear";
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

    	return "deployed-rear";
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
    	if(transition.compare("toPackedFront") == 0){
    		if(toPackedFront(group_l_,group_r_,group_both_)){
    			return "packed-front";
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
    	else if(transition.compare("toPackedRear") == 0){
    		if(toPackedRear(group_l_,group_r_,group_both_)){
    			return "packed-rear";
    		} else {
    			return "unknown_state";
    		}
    	}    

    	return "prepack-rear";
    }

	//------DEPLOY-FRONT-------------------------------------------
    else if(currentState.compare("deployed-front") == 0){

    	//Transitions
    	if(transition.compare("deployedFrontToHome") == 0){
    		if(deployedFrontToHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
    	}    

    	return "deployed-front";    	
    }
	  
	//------packed-rear-drop-------------------------------------------
    else if(currentState.compare("packed-rear-drop") == 0){

    	//Transitions
    	if(transition.compare("packedRearDropToHome") == 0){
    		if(packedRearDropToHome(group_l_,group_r_,group_both_)){
    			return "home";
    		} else {
    			return "unknown_state";
    		}
    	}    

    	return "packed-rear-drop";    	
    }
	 
	//------packed-rear-drop-------------------------------------------
    else if(currentState.compare("test_auto_rotate") == 0){

    	seneka_pnp_tools::move_turret_to(node_handle_, seneka_pnp_tools::TURRET_POSE_PICKUP_FRONT);

    	return "home";    	
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

	  SerializeIO *ser = new SerializeIO("/home/quanjo/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_pnp/common/teached_dual_arm_movement.def",'i');
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
    double planning_time = 20.0;
    double orientation_tolerance = 0.01;
    double position_tolerance = 0.001;

    group_r_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_r_->setStartStateToCurrentState();
    group_l_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_l_->setStartStateToCurrentState();
    group_both_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_both_->setStartStateToCurrentState();

    group_r_->setGoalOrientationTolerance(orientation_tolerance);
    group_r_->setGoalPositionTolerance(position_tolerance);
    group_r_->setPlannerId("RRTConnectkConfigDefault");
    group_r_->setPlanningTime(planning_time);
    group_l_->setGoalOrientationTolerance(orientation_tolerance);
    group_l_->setGoalPositionTolerance(position_tolerance);
    group_l_->setPlannerId("RRTConnectkConfigDefault");
    group_l_->setPlanningTime(planning_time);
    group_both_->setGoalOrientationTolerance(orientation_tolerance);
    group_both_->setGoalPositionTolerance(position_tolerance);
    group_both_->setPlannerId("RRTConnectkConfigDefault");
    group_both_->setPlanningTime(planning_time);
  }
  //------------ Services -----------------------------------

  //main loop
  //check Sensornode position and start planning
  void mainLoop(){

    service_getstate_ = node_handle_.advertiseService("seneka_pnp/getState", &SenekaPickAndPlace::getState, this);
    service_settransition_ = node_handle_.advertiseService("seneka_pnp/setTransition", &SenekaPickAndPlace::setTransition, this);
    service_setstop_ = node_handle_.advertiseService("seneka_pnp/setStop", &SenekaPickAndPlace::setStop, this);
    
    //QuanjoArmSupervisorAction supervisor(ros::this_node::getName(), this);
    
    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();

    ros::Rate loop_rate(1);
    while(ros::ok()){
      
      //test sensornode yaw axis 
//      sensornode tmp_node = seneka_pnp_tools::getSensornodePose();
//      if(tmp_node.success)
//    	  seneka_pnp_tools::sensornodeYawRotation(tmp_node.pose);
      //test sensornode yaw axis
        
      currentState_ = stateMachine(currentState_);
      //ROS_INFO("Current state: %s",currentState_.c_str());
      loop_rate.sleep();
    }
  }
};


int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "SenekaPickAndPlace");
    ros::NodeHandle nh;
    
//    boost::mutex lock;
//    
//    lock.lock();
//    seneka_pnp_tools::move_turret_to(nh, seneka_pnp_tools::TURRET_POSE_PICKUP_REAR);
//    lock.unlock();
//    sleep(5);
//    seneka_pnp_tools::move_legs(nh, seneka_pnp_tools::MOVE_LEGS_DOWN);
//    ROS_INFO("FINISH");
//    sleep(5);
//    seneka_pnp_tools::move_legs(nh, seneka_pnp_tools::MOVE_LEGS_UP);
//    ROS_INFO("FINISH");

    /// Create SenekaPickAndPlace instance with mainLoop inside
    //SenekaPickAndPlace seneka_pnp(ros::this_node::getName());
    SenekaPickAndPlace* seneka_pnp = new SenekaPickAndPlace(nh,ros::this_node::getName());
    
    delete seneka_pnp;
    return 0;
}
