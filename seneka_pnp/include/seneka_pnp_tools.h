/*
 * seneka_pnp_tools.h
 *
 *  Created on: 18.06.2014
 *      Author: Matthias Nösner
 */

#ifndef SENEKA_PNP_TOOLS_H_
#define SENEKA_PNP_TOOLS_H_

#include <ros/ros.h>
#include <moveit_msgs/GetPositionFK.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

namespace seneka_pnp_tools{

	//wrapper for calling the fk service due to some failures when using it with move_group.h
	void fk_solver(ros::NodeHandle *node_handle, std::vector<double> &joint_positions_r, std::vector<double> &joint_positions_l, geometry_msgs::Pose *pose_l, geometry_msgs::Pose *pose_r){
	
		ros::ServiceClient service_computefk;
		service_computefk = node_handle->serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");		
		
		robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
		robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
		robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
		robot_state::JointStateGroup* joint_state_group_l = kinematic_state_l->getJointStateGroup("left_arm_group");  
		robot_state::JointStateGroup* joint_state_group_r = kinematic_state_l->getJointStateGroup("right_arm_group"); 
	
		moveit_msgs::GetPositionFK::Request service_request;
		moveit_msgs::GetPositionFK::Response service_response; 
		sensor_msgs::JointState js;
	
		//------left-----------------------
		js.name = joint_state_group_l->getJointNames();
		js.position = joint_positions_l;
	
		service_request.header.frame_id = "world_dummy_link";
		service_request.fk_link_names.push_back("left_arm_ee_link");
		service_request.robot_state.joint_state = js;
	
		service_computefk.call(service_request, service_response);
		if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
			for(uint i=0;i<service_response.pose_stamped.size();i++){
				std::cout << service_response.pose_stamped[i].pose.position << std::endl;
				std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
				*pose_l = service_response.pose_stamped[i].pose;
			}
		}   
	
		//-----right-----------------
		js.name = joint_state_group_r->getJointNames();
		js.position = joint_positions_r;
	
		service_request.header.frame_id = "world_dummy_link";
		service_request.fk_link_names.push_back("right_arm_ee_link");
		service_request.robot_state.joint_state = js;
	
		service_computefk.call(service_request, service_response);
		if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
			for(uint i=0;i<service_response.pose_stamped.size();i++){
				std::cout << service_response.pose_stamped[i].pose.position << std::endl;
				std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
				*pose_r = service_response.pose_stamped[i].pose;
			}
		}  
	}

	bool multiplan(move_group_interface::MoveGroup* group, moveit::planning_interface::MoveGroup::Plan* plan){

		uint maxattempts = 20;
		uint attempt = 0;

		bool validplan = false;

		while(!validplan && attempt < maxattempts){

			validplan = group->plan(*plan);
			attempt++;
		}

		return validplan;	  
	}
	
	//code from prace project, IPA: Benjamin Maidel
	move_group_interface::MoveGroup::Plan mergePlan(move_group_interface::MoveGroup::Plan plan1, move_group_interface::MoveGroup::Plan plan2)
	{
		move_group_interface::MoveGroup::Plan mergedPlan;
		mergedPlan = plan1;
		mergedPlan.trajectory_.joint_trajectory.joint_names.insert(mergedPlan.trajectory_.joint_trajectory.joint_names.end(),
				plan2.trajectory_.joint_trajectory.joint_names.begin(),
				plan2.trajectory_.joint_trajectory.joint_names.end());

		size_t i;
		for(i = 0; (i < mergedPlan.trajectory_.joint_trajectory.points.size())
		&& (i < plan2.trajectory_.joint_trajectory.points.size()); i++){

			mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.insert(
					mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.end(),
					plan2.trajectory_.joint_trajectory.points[i].accelerations.begin(),
					plan2.trajectory_.joint_trajectory.points[i].accelerations.end());

			mergedPlan.trajectory_.joint_trajectory.points[i].positions.insert(
					mergedPlan.trajectory_.joint_trajectory.points[i].positions.end(),
					plan2.trajectory_.joint_trajectory.points[i].positions.begin(),
					plan2.trajectory_.joint_trajectory.points[i].positions.end());

			mergedPlan.trajectory_.joint_trajectory.points[i].velocities.insert(
					mergedPlan.trajectory_.joint_trajectory.points[i].velocities.end(),
					plan2.trajectory_.joint_trajectory.points[i].velocities.begin(),
					plan2.trajectory_.joint_trajectory.points[i].velocities.end());
		}

		if(plan1.trajectory_.joint_trajectory.points.size() > plan2.trajectory_.joint_trajectory.points.size()){

			for(size_t j = i; j < plan1.trajectory_.joint_trajectory.points.size(); j++){

				mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.insert(
						mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.end(),
						plan2.trajectory_.joint_trajectory.points.back().accelerations.begin(),
						plan2.trajectory_.joint_trajectory.points.back().accelerations.end());

				mergedPlan.trajectory_.joint_trajectory.points[j].positions.insert(
						mergedPlan.trajectory_.joint_trajectory.points[j].positions.end(),
						plan2.trajectory_.joint_trajectory.points.back().positions.begin(),
						plan2.trajectory_.joint_trajectory.points.back().positions.end());

				mergedPlan.trajectory_.joint_trajectory.points[j].velocities.insert(
						mergedPlan.trajectory_.joint_trajectory.points[j].velocities.end(),
						plan2.trajectory_.joint_trajectory.points.back().velocities.begin(),
						plan2.trajectory_.joint_trajectory.points.back().velocities.end());
			}
		}

		if(plan1.trajectory_.joint_trajectory.points.size() < plan2.trajectory_.joint_trajectory.points.size()){

			trajectory_msgs::JointTrajectoryPoint point;
			for(size_t j = i; j < plan2.trajectory_.joint_trajectory.points.size(); j++){

				point  = mergedPlan.trajectory_.joint_trajectory.points.back();

				point.accelerations.insert(
						point.accelerations.end(),
						plan2.trajectory_.joint_trajectory.points[j].accelerations.begin(),
						plan2.trajectory_.joint_trajectory.points[j].accelerations.end());

				point.positions.insert(
						point.positions.end(),
						plan2.trajectory_.joint_trajectory.points[j].positions.begin(),
						plan2.trajectory_.joint_trajectory.points[j].positions.end());

				point.velocities.insert(
						point.velocities.end(),
						plan2.trajectory_.joint_trajectory.points[j].velocities.begin(),
						plan2.trajectory_.joint_trajectory.points[j].velocities.end());

				point.time_from_start = plan2.trajectory_.joint_trajectory.points[j].time_from_start;

				mergedPlan.trajectory_.joint_trajectory.points.push_back(point);
			}
		}

		return mergedPlan;
	}

	moveit::planning_interface::MoveGroup::Plan scaleTrajSpeed(moveit::planning_interface::MoveGroup::Plan plan, double factor){

		std::vector<trajectory_msgs::JointTrajectoryPoint> points = plan.trajectory_.joint_trajectory.points;

		for(uint i = 0; i < points.size();i++){

			for(uint j = 0; j < points[i].velocities.size();j++)
				points[i].velocities[j] = points[i].velocities[j] * factor;
			for(uint j = 0; j < points[i].accelerations.size();j++)
				points[i].velocities[j] = points[i].accelerations[j] * factor * factor;

			points[i].time_from_start = points[i].time_from_start * (1/factor);
		}

		plan.trajectory_.joint_trajectory.points = points;

		return plan;
	  }

}

#endif