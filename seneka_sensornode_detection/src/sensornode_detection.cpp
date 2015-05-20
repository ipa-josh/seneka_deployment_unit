/**********************
* Author: Matthias Nösner
********************** */
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cob_object_detection_msgs/DetectObjects.h>

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <seneka_sensornode_detection/setCameraPose.h>
#include <seneka_sensornode_detection/getCameraPose.h>
#include <seneka_sensornode_detection/calibrateCamera.h>
#include <seneka_sensornode_detection/getExtCalibration.h>
#include <seneka_sensornode_detection/compensateInaccuracy.h>
#include <seneka_sensornode_detection/setSensornodePose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <SerializeIO.h>

#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>

const double PI = 3.14159265359;

struct pose{
  double x;
  double y;
  double z;
};

struct quaternion{
  double w;
  double x;
  double y;
  double z;
};

struct pose3d{
  pose translation;
  quaternion rotation;
};

struct point4d{
  double x;
  double y;
  double z;
  double e;
  bool detected;
};

struct fiducialmarker{
  unsigned int id;
  cv::Point3d trans;
  cv::Point3d rot;
};

struct handle{
  cv::Point3d trans;
  cv::Point3d rot;
};

struct trigger_points{
  cv::Point3d up;
  cv::Point3d down;
};

using namespace std;

void sensornodeLocalCoordinates();
void sensornodeGlobalCoordinates();
void senekaCalibrateCamera(geometry_msgs::Pose);
bool loadParameters(std::vector<fiducialmarker>*,std::vector<handle>*, trigger_points*, pose*);
void init(ros::NodeHandle);

//---------------------------<coordinate transformations>---------------------------------------
cv::Mat eulerToMatrix(double rot1, double rot2, double rot3);
void multiplyFrame7(std::vector<double> q1,std::vector<double> q2,std::vector<double>* q);
std::vector<double> FrameToVec7(const cv::Mat frame);
//---------------------------</coordinate transformations>---------------------------------------

std::vector<fiducialmarker> fiducialmarkers;
std::vector<handle> handles;
trigger_points trigger_offset;
pose grab_entry;
double gripper_length = 0.26; 
double gripper_depth = 0.005;
pose3d sensornode_pose;

boost::mutex tf_lock_;
ros::Timer tf_pub_timer_;
tf::StampedTransform marker_tf_;

bool extrinsic_camera_calib_ = false;

//camera in quanjo system as vec7
//tf::Quaternion qt_ = tf::createQuaternionFromRPY(-PI/2,0,-PI/2);

geometry_msgs::Pose reference_pose_;
geometry_msgs::Pose camera_pose_;
ros::ServiceServer service_setCameraPose_, service_getCameraPose_, service_calibrateCamera_,
				   service_getExtCalibration_, service_compensateInaccuracy_, service_setSensornodePose_ ;

bool simulation_ = false;
geometry_msgs::Pose sensornodeSimulatedPose_;

//------------------------------<Callbacks>----------------------------------------------------------
//The sensorsondeCoordinateManager manages and publishes all coordinate transform for the sensorsonde
//IN:Marker positions from cob_fiducials
//OUT:sensornode position, handles and trigger points for sensornode manipulation 
//--------
//TODO: Compute sensorsonde pose from all markers. (Right now only marker 1 is used)


void init(ros::NodeHandle nh){
	
	geometry_msgs::Point rpy, rpy_in_rad;	
	
	nh.getParam("external_calibration_position_x", camera_pose_.position.x);
	nh.getParam("external_calibration_position_y", camera_pose_.position.y);
	nh.getParam("external_calibration_position_z", camera_pose_.position.z);
	nh.getParam("external_calibration_orientation_w", camera_pose_.orientation.w);
	nh.getParam("external_calibration_orientation_x", camera_pose_.orientation.x);
	nh.getParam("external_calibration_orientation_y", camera_pose_.orientation.y);
	nh.getParam("external_calibration_orientation_z", camera_pose_.orientation.z);
	
	nh.getParam("reference_position_x", reference_pose_.position.x);
	nh.getParam("reference_position_y", reference_pose_.position.y);
	nh.getParam("reference_position_z", reference_pose_.position.z);
	nh.getParam("reference_orientation_w", reference_pose_.orientation.w);
	nh.getParam("reference_orientation_x", reference_pose_.orientation.x);
	nh.getParam("reference_orientation_y", reference_pose_.orientation.y);
	nh.getParam("reference_orientation_z", reference_pose_.orientation.z);
	
	sensornodeSimulatedPose_.position.x = 5;
	sensornodeSimulatedPose_.position.y = 0;
	sensornodeSimulatedPose_.position.z = 0;
	
	sensornodeSimulatedPose_.orientation.x = 0;
	sensornodeSimulatedPose_.orientation.y = 0;
	sensornodeSimulatedPose_.orientation.z = 0;
	sensornodeSimulatedPose_.orientation.w = 1;	
}

//Coordinate transformation for simulation in gazebo, Something is wrong with the trafo
void getSensornodeStateGazebo(ros::NodeHandle& n){  

  sensornodeGlobalCoordinates();

  gazebo_msgs::GetModelState gms_r,gms_p;
  ros::ServiceClient gms_c;

  //get sensornode state from gazebo in world frame (rot)
  gms_r.request.model_name="sensornode";
  gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gms_c.call(gms_r);//gms_r now holds the current state

  //get sensornode state from gazebo in world_dummy_link/quanjo frame (pos)
  gms_p.request.model_name="sensornode";
  gms_p.request.relative_entity_name="world_dummy_link";
  gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gms_c.call(gms_p);//gms now holds the current state

  
  tf::Quaternion quat(gms_r.response.pose.orientation.x,gms_r.response.pose.orientation.y,gms_r.response.pose.orientation.z,gms_r.response.pose.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);  
  tf::Quaternion quat_t;
  quat_t.setRPY(roll,pitch,yaw+0.78);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  transform.setOrigin( tf::Vector3(gms_p.response.pose.position.x,gms_p.response.pose.position.y,gms_p.response.pose.position.z) );
  transform.setRotation(quat_t);
  br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/quanjo_body", "/sensornode"));

  sensornodeLocalCoordinates();  
}

//Coordinate transformation for real world sensornode detection
void getSensornodeStateCamera(const cob_object_detection_msgs::DetectionArray::ConstPtr& msg)
{
  ROS_INFO("[sensornode_detection] I heard from  [%u] fiducial detections", (unsigned int)msg->detections.size());

  sensornodeGlobalCoordinates();
  

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  //publish reference point (for camera calibration)
  transform.setOrigin( tf::Vector3(reference_pose_.position.x,reference_pose_.position.y,reference_pose_.position.z) );
  transform.setRotation( tf::Quaternion(reference_pose_.orientation.x,reference_pose_.orientation.y,reference_pose_.orientation.z,reference_pose_.orientation.w));
  br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/quanjo_body", "/calibration_reference"));

  pose origin;
  quaternion rotation;

  //Right now only the first detected marker is used
  //TODO: Make use of all detected markers
  if(msg->detections.size() > 0){

	  if(extrinsic_camera_calib_){

		  geometry_msgs::Pose markerpose;		
		  markerpose = msg->detections[0].pose.pose;		
		  senekaCalibrateCamera(markerpose);

		  extrinsic_camera_calib_ = false;
	  }

	  //use marker 1 or a the marker with the lowest id
	  unsigned int position = 0;
	  for(unsigned int i = 0; i < msg->detections.size(); i++){
		  
		  if(msg->detections[i].id == 1){
			  position = i;
			  break;
		  }		  
	  }
	  if(msg->detections[position].id != 1){
		  for(unsigned int i = 0; i < msg->detections.size(); i++){
			  if(msg->detections[i].id < msg->detections[position].id){
				  position = i;
			  }
		  }
	  }

	  ROS_INFO("I'M USING MARKER [%u]", msg->detections[position].id);
	  
	  //only the first detected marker is used
	  origin.x = msg->detections[position].pose.pose.position.x;
	  origin.y = msg->detections[position].pose.pose.position.y;
	  origin.z = msg->detections[position].pose.pose.position.z;    
	  rotation.w = msg->detections[position].pose.pose.orientation.w;
	  rotation.x = msg->detections[position].pose.pose.orientation.x;
	  rotation.y = msg->detections[position].pose.pose.orientation.y;
	  rotation.z = msg->detections[position].pose.pose.orientation.z;

	  transform.setOrigin( tf::Vector3(origin.x,origin.y,origin.z) );
	  transform.setRotation( tf::Quaternion(rotation.x,rotation.y,rotation.z,rotation.w));
	  br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/quanjo_camera", "/seneka_marker"));
	  	
	  unsigned int i = position;
	  unsigned int id = msg->detections[i].id;
 
	  //another transform
	  geometry_msgs::Pose markerpose;
	  markerpose.position.x = fiducialmarkers[id-1].trans.x;
	  markerpose.position.y = fiducialmarkers[id-1].trans.y;
	  markerpose.position.z = fiducialmarkers[id-1].trans.z;
	  markerpose.orientation = tf::createQuaternionMsgFromRollPitchYaw( fiducialmarkers[id-1].rot.x, fiducialmarkers[id-1].rot.y, fiducialmarkers[id-1].rot.z );
	 
	  geometry_msgs::Pose sensornodepose;
	  sensornodepose.position.x = -markerpose.position.x;
	  sensornodepose.position.y = -markerpose.position.y;
	  sensornodepose.position.z = -markerpose.position.z;
	  
	  tf::Quaternion qt;
	  tf::quaternionMsgToTF(markerpose.orientation, qt);
	  qt.inverse();
	  tf::quaternionTFToMsg(qt, sensornodepose.orientation);
	  
	  transform.setOrigin( tf::Vector3(sensornodepose.position.x, sensornodepose.position.y, sensornodepose.position.z) );
	  transform.setRotation( qt );
	  br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/seneka_marker", "/sensornode"));
	  
	  sensornode_pose.translation.x = sensornodepose.position.x;
	  sensornode_pose.translation.y = sensornodepose.position.y;
	  sensornode_pose.translation.z = sensornodepose.position.z;
	  sensornode_pose.rotation.w = sensornodepose.orientation.w;
	  sensornode_pose.rotation.x = sensornodepose.orientation.x;
	  sensornode_pose.rotation.y = sensornodepose.orientation.y;
	  sensornode_pose.rotation.z = sensornodepose.orientation.z;
	  	  
			  
	  //publish marker poses
//	  for(unsigned int j = 0; j <  fiducialmarkers.size(); j++ ){
//		  			  
//		  transform.setOrigin( tf::Vector3(fiducialmarkers[j].trans.x, fiducialmarkers[j].trans.y, fiducialmarkers[j].trans.z));
//		  transform.setRotation( tf::createQuaternionFromRPY( fiducialmarkers[j].rot.x, fiducialmarkers[j].rot.y, fiducialmarkers[j].rot.z ) );
//
//		  //ROS_INFO("x,y,z,r,p,y [%f] [%f] [%f] [%f] [%f] [%f]",fiducialmarkers[j].trans.x, fiducialmarkers[j].trans.y, fiducialmarkers[j].trans.z, fiducialmarkers[j].rot.x, fiducialmarkers[j].rot.y, fiducialmarkers[j].rot.z);
//
//		  char markerid[50];
//		  sprintf(markerid,"/marker_%u",fiducialmarkers[j].id);
//		  br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/sensornode", markerid));			  
//	  }
	  
	  sensornodeLocalCoordinates();
  }   
}

void setSensornodePose(geometry_msgs::Pose pose){

	tf::Quaternion qt_static;
	qt_static = tf::createQuaternionFromRPY(PI/2,0,-PI/2);
	//qt_static(tf::Scalar(0.0), tf::Scalar(0.707), tf::Scalar(0.707), tf::Scalar(0.0));
	qt_static.normalize();
	
	tf::Quaternion qt;
	tf::quaternionMsgToTF(pose.orientation, qt);	
	
	qt = qt_static * qt;
	
	tf::quaternionTFToMsg(qt, pose.orientation);

	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
	transform.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
	br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/quanjo_body", "/sensornode"));

	sensornodeLocalCoordinates();
}

//calibrate the external camera position in robot system
void senekaCalibrateCamera(geometry_msgs::Pose measured_marker){	
	
	//measured marker fixed pose in robot system
	//base system world_dummy_link
	geometry_msgs::Pose fixed_marker;
	fixed_marker = reference_pose_;			
	
	tf::Quaternion fixed_marker_qt;
	tf::quaternionMsgToTF(fixed_marker.orientation, fixed_marker_qt);
	
	//measured marker in camera system
	tf::Quaternion measured_marker_qt; 
	tf::quaternionMsgToTF(measured_marker.orientation, measured_marker_qt);
	
	//measured marker in robot system
	tf::Matrix3x3 m(measured_marker_qt.inverse());
	tf::Vector3 vec_3(measured_marker.position.x, measured_marker.position.y, measured_marker.position.z);
	vec_3 = m * vec_3;
	
	//camera in robot system
	geometry_msgs::Pose camera_rs;
	camera_rs.position.x = fixed_marker.position.x - vec_3.x();
	camera_rs.position.y = fixed_marker.position.y - vec_3.y();
	camera_rs.position.z = fixed_marker.position.z - vec_3.z();

	tf::Quaternion qt_rs;
	qt_rs = fixed_marker_qt * measured_marker_qt.inverse();
	qt_rs.normalize();	
	tf::quaternionTFToMsg(qt_rs, camera_rs.orientation);
	
	camera_pose_ = camera_rs;
}	

//------------------------------</Callbacks>----------------------------------------------------------
void sensornodeGlobalCoordinates(){
 
  //dummy quanjo_position
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0) );
  transform.setRotation( tf::createQuaternionFromRPY(0,0,0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world_dummy_link", "/quanjo_body"));
  
  //Camera Position
  transform.setOrigin( tf::Vector3(camera_pose_.position.x, 
		  	  	  	  	  	  	   camera_pose_.position.y, 
		  	  	  	  	  	  	   camera_pose_.position.z) );
  transform.setRotation( tf::Quaternion(camera_pose_.orientation.x,
		  	  	  	  	  	  	  	    camera_pose_.orientation.y,
		  	  	  	  	  	  	  	    camera_pose_.orientation.z,
		  	  	  	  	  	  	  	    camera_pose_.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/quanjo_body", "/quanjo_camera"));
  
}

void sensornodeLocalCoordinates(){

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  //Do the coordinate transformations and publish handle position based on sensornode_pose
  for(unsigned int i = 0; i < handles.size(); i++){
            
    cv::Mat tm_hl_sn = eulerToMatrix( handles[i].rot.x, handles[i].rot.y, handles[i].rot.z);//3x3Matrix
    cv::Mat tmp = cv::Mat(tm_hl_sn);
    tm_hl_sn = cv::Mat(3,4, CV_64FC1);  
      
    //create homogenous coordinates
    for (int k=0; k<3; k++)
      for (int l=0; l<3; l++)
	tm_hl_sn.at<double>(k,l) = tmp.at<double>(k,l);
    tm_hl_sn.at<double>(0,3) = handles[i].trans.x;
    tm_hl_sn.at<double>(1,3) = handles[i].trans.y;
    tm_hl_sn.at<double>(2,3) = handles[i].trans.z;

    std::vector<double> q_hl_sn = FrameToVec7(tm_hl_sn);

    //publish handle positions
    char handle_name[50];
    sprintf(handle_name,"/handle%u",i+1);
    transform.setOrigin( tf::Vector3(q_hl_sn[0], q_hl_sn[1], q_hl_sn[2]));
    transform.setRotation( tf::Quaternion(q_hl_sn[4],q_hl_sn[5],q_hl_sn[6],q_hl_sn[3]));
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "/sensornode", handle_name));

    //publish trigger points and initial grabbing pose
    char trigger_name[50];      
    sprintf(trigger_name,"trigger_%u_up",i+1);
    transform.setOrigin( tf::Vector3(trigger_offset.up.x, trigger_offset.up.y, trigger_offset.up.z));
    transform.setRotation( tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), handle_name, trigger_name));

    sprintf(trigger_name,"trigger_%u_down",i+1);
    transform.setOrigin( tf::Vector3(trigger_offset.down.x, trigger_offset.down.y, trigger_offset.down.z));
    transform.setRotation( tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), handle_name, trigger_name));  

    sprintf(trigger_name,"grab_entry%u",i+1);
    transform.setOrigin( tf::Vector3(grab_entry.x, grab_entry.y, grab_entry.z));
    transform.setRotation( tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), handle_name, trigger_name));       
  }
}

//Load Parameters positions of markers,handles and triggers in reference to the sensorsonde using my own SerializeIO class + scaling the values.
bool loadParameters(std::vector<fiducialmarker>* afiducialmarkers, std::vector<handle>* ahandles, trigger_points* atriggers, pose* aentry){
	
  std::string path = ros::package::getPath("seneka_sensornode_detection");
  std::string concatpath = path + "/launch/sensorsonde_coordinates.def";
  
  SerializeIO *ser = new SerializeIO(concatpath.c_str(),'i');
  if(simulation_){
    ser->close();
    concatpath = path + "/launch/sensorsonde_coordinates_sim.def";
    ser = new SerializeIO(concatpath.c_str(),'i');
  }

  fiducialmarker fiducial1, fiducial2, fiducial3, fiducial4, fiducial5, fiducial6;
  handle handle1, handle2, handle3, handle4, handle5, handle6;
  trigger_points triggers;
	

  double scale = 0.001;//from mm to m

  //Warning: Fiducial id must be identical with the id('s) from cob_fiducials!!
  //Warning: You are responsible for that!!
  if(!ser->readVariable("fiducial1_ID",&(fiducial1.id))) return false;
  if(!ser->readVariable("fiducial1_POSX",&(fiducial1.trans.x))) return false;
  if(!ser->readVariable("fiducial1_POSY",&(fiducial1.trans.y))) return false;
  if(!ser->readVariable("fiducial1_POSZ",&(fiducial1.trans.z))) return false;
  if(!ser->readVariable("fiducial1_ROT1",&(fiducial1.rot.x))) return false;
  if(!ser->readVariable("fiducial1_ROT2",&(fiducial1.rot.y))) return false;
  if(!ser->readVariable("fiducial1_ROT3",&(fiducial1.rot.z))) return false;

  if(!ser->readVariable("fiducial2_ID",&(fiducial2.id))) return false;
  if(!ser->readVariable("fiducial2_POSX",&(fiducial2.trans.x))) return false;
  if(!ser->readVariable("fiducial2_POSY",&(fiducial2.trans.y))) return false;
  if(!ser->readVariable("fiducial2_POSZ",&(fiducial2.trans.z))) return false;
  if(!ser->readVariable("fiducial2_ROT1",&(fiducial2.rot.x))) return false;
  if(!ser->readVariable("fiducial2_ROT2",&(fiducial2.rot.y))) return false;
  if(!ser->readVariable("fiducial2_ROT3",&(fiducial2.rot.z))) return false;

  if(!ser->readVariable("fiducial3_ID",&(fiducial3.id))) return false;
  if(!ser->readVariable("fiducial3_POSX",&(fiducial3.trans.x))) return false;
  if(!ser->readVariable("fiducial3_POSY",&(fiducial3.trans.y))) return false;
  if(!ser->readVariable("fiducial3_POSZ",&(fiducial3.trans.z))) return false;
  if(!ser->readVariable("fiducial3_ROT1",&(fiducial3.rot.x))) return false;
  if(!ser->readVariable("fiducial3_ROT2",&(fiducial3.rot.y))) return false;
  if(!ser->readVariable("fiducial3_ROT3",&(fiducial3.rot.z))) return false;

  if(!ser->readVariable("fiducial4_ID",&(fiducial4.id))) return false;
  if(!ser->readVariable("fiducial4_POSX",&(fiducial4.trans.x))) return false;
  if(!ser->readVariable("fiducial4_POSY",&(fiducial4.trans.y))) return false;
  if(!ser->readVariable("fiducial4_POSZ",&(fiducial4.trans.z))) return false;
  if(!ser->readVariable("fiducial4_ROT1",&(fiducial4.rot.x))) return false;
  if(!ser->readVariable("fiducial4_ROT2",&(fiducial4.rot.y))) return false;
  if(!ser->readVariable("fiducial4_ROT3",&(fiducial4.rot.z))) return false;

  if(!ser->readVariable("fiducial5_ID",&(fiducial5.id))) return false;
  if(!ser->readVariable("fiducial5_POSX",&(fiducial5.trans.x))) return false;
  if(!ser->readVariable("fiducial5_POSY",&(fiducial5.trans.y))) return false;
  if(!ser->readVariable("fiducial5_POSZ",&(fiducial5.trans.z))) return false;
  if(!ser->readVariable("fiducial5_ROT1",&(fiducial5.rot.x))) return false;
  if(!ser->readVariable("fiducial5_ROT2",&(fiducial5.rot.y))) return false;
  if(!ser->readVariable("fiducial5_ROT3",&(fiducial5.rot.z))) return false;

  if(!ser->readVariable("fiducial6_ID",&(fiducial6.id))) return false;
  if(!ser->readVariable("fiducial6_POSX",&(fiducial6.trans.x))) return false;
  if(!ser->readVariable("fiducial6_POSY",&(fiducial6.trans.y))) return false;
  if(!ser->readVariable("fiducial6_POSZ",&(fiducial6.trans.z))) return false;
  if(!ser->readVariable("fiducial6_ROT1",&(fiducial6.rot.x))) return false;
  if(!ser->readVariable("fiducial6_ROT2",&(fiducial6.rot.y))) return false;
  if(!ser->readVariable("fiducial6_ROT3",&(fiducial6.rot.z))) return false;	

  //--- Read handle pose ---
  if(!ser->readVariable("handle1_x",&(handle1.trans.x))) return false;
  if(!ser->readVariable("handle1_y",&(handle1.trans.y))) return false;
  if(!ser->readVariable("handle1_z",&(handle1.trans.z))) return false;
  if(!ser->readVariable("handle1_roll",&(handle1.rot.x))) return false;
  if(!ser->readVariable("handle1_pitch",&(handle1.rot.y))) return false;
  if(!ser->readVariable("handle1_yaw",&(handle1.rot.z))) return false;

  if(!ser->readVariable("handle2_x",&(handle2.trans.x))) return false;
  if(!ser->readVariable("handle2_y",&(handle2.trans.y))) return false;
  if(!ser->readVariable("handle2_z",&(handle2.trans.z))) return false;
  if(!ser->readVariable("handle2_roll",&(handle2.rot.x))) return false;
  if(!ser->readVariable("handle2_pitch",&(handle2.rot.y))) return false;
  if(!ser->readVariable("handle2_yaw",&(handle2.rot.z))) return false;

  if(!ser->readVariable("handle3_x",&(handle3.trans.x))) return false;
  if(!ser->readVariable("handle3_y",&(handle3.trans.y))) return false;
  if(!ser->readVariable("handle3_z",&(handle3.trans.z))) return false;
  if(!ser->readVariable("handle3_roll",&(handle3.rot.x))) return false;
  if(!ser->readVariable("handle3_pitch",&(handle3.rot.y))) return false;
  if(!ser->readVariable("handle3_yaw",&(handle3.rot.z))) return false;

  if(!ser->readVariable("handle4_x",&(handle4.trans.x))) return false;
  if(!ser->readVariable("handle4_y",&(handle4.trans.y))) return false;
  if(!ser->readVariable("handle4_z",&(handle4.trans.z))) return false;
  if(!ser->readVariable("handle4_roll",&(handle4.rot.x))) return false;
  if(!ser->readVariable("handle4_pitch",&(handle4.rot.y))) return false;
  if(!ser->readVariable("handle4_yaw",&(handle4.rot.z))) return false;

  if(!ser->readVariable("handle5_x",&(handle5.trans.x))) return false;
  if(!ser->readVariable("handle5_y",&(handle5.trans.y))) return false;
  if(!ser->readVariable("handle5_z",&(handle5.trans.z))) return false;
  if(!ser->readVariable("handle5_roll",&(handle5.rot.x))) return false;
  if(!ser->readVariable("handle5_pitch",&(handle5.rot.y))) return false;
  if(!ser->readVariable("handle5_yaw",&(handle5.rot.z))) return false;

  if(!ser->readVariable("handle6_x",&(handle6.trans.x))) return false;
  if(!ser->readVariable("handle6_y",&(handle6.trans.y))) return false;
  if(!ser->readVariable("handle6_z",&(handle6.trans.z))) return false;
  if(!ser->readVariable("handle6_roll",&(handle6.rot.x))) return false;
  if(!ser->readVariable("handle6_pitch",&(handle6.rot.y))) return false;
  if(!ser->readVariable("handle6_yaw",&(handle6.rot.z))) return false;

  //--- Read trigger offset ---
  if(!ser->readVariable("trigger_up_offset_x",&(triggers.up.x))) return false;
  if(!ser->readVariable("trigger_up_offset_y",&(triggers.up.y))) return false;
  if(!ser->readVariable("trigger_up_offset_z",&(triggers.up.z))) return false;

  if(!ser->readVariable("trigger_down_offset_x",&(triggers.down.x))) return false;
  if(!ser->readVariable("trigger_down_offset_y",&(triggers.down.y))) return false;
  if(!ser->readVariable("trigger_down_offset_z",&(triggers.down.z))) return false;

  if(!ser->readVariable("grab_entry_x",&(grab_entry.x))) return false;
  if(!ser->readVariable("grab_entry_y",&(grab_entry.y))) return false;
  if(!ser->readVariable("grab_entry_z",&(grab_entry.z))) return false;


  afiducialmarkers->push_back(fiducial1);
  afiducialmarkers->push_back(fiducial2);
  afiducialmarkers->push_back(fiducial3);
  afiducialmarkers->push_back(fiducial4);
  afiducialmarkers->push_back(fiducial5);
  afiducialmarkers->push_back(fiducial6);

  ahandles->push_back(handle1);
  ahandles->push_back(handle2);
  ahandles->push_back(handle3);
  ahandles->push_back(handle4);
  ahandles->push_back(handle5);
  ahandles->push_back(handle6);

  *atriggers = triggers;

  //Scale to meter
  for(unsigned int i = 0; i < afiducialmarkers->size(); i++){
	  
    (*afiducialmarkers)[i].trans.x =  (*afiducialmarkers)[i].trans.x * scale;
    (*afiducialmarkers)[i].trans.y =  (*afiducialmarkers)[i].trans.y * scale;
    (*afiducialmarkers)[i].trans.z =  (*afiducialmarkers)[i].trans.z * scale;
  }
  for(unsigned int i = 0; i < ahandles->size(); i++){
	  
    (*ahandles)[i].trans.x =  (*ahandles)[i].trans.x * scale;
    (*ahandles)[i].trans.y =  (*ahandles)[i].trans.y * scale;
    (*ahandles)[i].trans.z =  (*ahandles)[i].trans.z * scale;
  }
  atriggers->up.x = atriggers->up.x * scale + gripper_depth;
  atriggers->up.y = atriggers->up.y * scale;
  atriggers->up.z = atriggers->up.z * scale + gripper_length;
  atriggers->down.x = atriggers->down.x * scale + gripper_depth;
  atriggers->down.y = atriggers->down.y * scale;
  atriggers->down.z = (atriggers->down.z * scale) + gripper_length;
  aentry->x = aentry->x * scale + gripper_depth;
  aentry->y = aentry->y * scale;
  aentry->z = aentry->z * scale + gripper_length;

  ser->close();
  delete ser;
  return true;
}


//------------------------Coordinate Transformations----------------------------------------------------------------------------------
//XYZ (Roll,Pitch,Yaw) Euler System to Matrix representation
cv::Mat eulerToMatrix(double rot1, double rot2, double rot3) {
  
  cv::Mat m(3,3,CV_64FC1);
	
  cv::Mat X_rot(3,3,CV_64F);
  cv::Mat Y_rot(3,3,CV_64F);
  cv::Mat Z_rot(3,3,CV_64F);
	
  double alpha =  rot1;
  double beta  =  rot2;
  double gamma =  rot3;
	
  X_rot.at<double>(0,0) =  1.0;
  X_rot.at<double>(0,1) =  0.0;
  X_rot.at<double>(0,2) =  0.0;
  X_rot.at<double>(1,0) =  0.0;
  X_rot.at<double>(1,1) =  cos(alpha);
  X_rot.at<double>(1,2) =  -sin(alpha);
  X_rot.at<double>(2,0) =  0.0;
  X_rot.at<double>(2,1) =  sin(alpha);
  X_rot.at<double>(2,2) =  cos(alpha);
	
  Y_rot.at<double>(0,0) =  cos(beta);
  Y_rot.at<double>(0,1) =  0.0;
  Y_rot.at<double>(0,2) =  sin(beta);
  Y_rot.at<double>(1,0) =  0.0;
  Y_rot.at<double>(1,1) =  1.0;
  Y_rot.at<double>(1,2) =  0.0;
  Y_rot.at<double>(2,0) =  -sin(beta);
  Y_rot.at<double>(2,1) =  0.0;
  Y_rot.at<double>(2,2) =  cos(beta);
	
  Z_rot.at<double>(0,0) =  cos(gamma);
  Z_rot.at<double>(0,1) =  -sin(gamma);
  Z_rot.at<double>(0,2) =  0.0;
  Z_rot.at<double>(1,0) =  sin(gamma);
  Z_rot.at<double>(1,1) =  cos(gamma);
  Z_rot.at<double>(1,2) =  0.0;
  Z_rot.at<double>(2,0) =  0.0;
  Z_rot.at<double>(2,1) =  0.0;
  Z_rot.at<double>(2,2) =  1.0;
	
  m = Z_rot*Y_rot*X_rot;
	
  return m;
}

// Function copied from cob_vision_ipa_utils/MathUtils.h to avoid dependency
inline float SIGN(float x)
{
  return (x >= 0.0f) ? +1.0f : -1.0f;
}

// Function copied from cob_vision_ipa_utils/MathUtils.h to avoid dependency
std::vector<double> FrameToVec7(const cv::Mat frame){

  // [0]-[2]: translation xyz
  // [3]-[6]: quaternion wxyz
  std::vector<double> pose(7, 0.0);

  double r11 = frame.at<double>(0,0);
  double r12 = frame.at<double>(0,1);
  double r13 = frame.at<double>(0,2);
  double r21 = frame.at<double>(1,0);
  double r22 = frame.at<double>(1,1);
  double r23 = frame.at<double>(1,2);
  double r31 = frame.at<double>(2,0);
  double r32 = frame.at<double>(2,1);
  double r33 = frame.at<double>(2,2);

  double qw = ( r11 + r22 + r33 + 1.0) / 4.0;
  double qx = ( r11 - r22 - r33 + 1.0) / 4.0;
  double qy = (-r11 + r22 - r33 + 1.0) / 4.0;
  double qz = (-r11 - r22 + r33 + 1.0) / 4.0;
  if(qw < 0.0f) qw = 0.0;
  if(qx < 0.0f) qx = 0.0;
  if(qy < 0.0f) qy = 0.0;
  if(qz < 0.0f) qz = 0.0;
  qw = std::sqrt(qw);
  qx = std::sqrt(qx);
  qy = std::sqrt(qy);
  qz = std::sqrt(qz);
  if(qw >= qx && qw >= qy && qw >= qz)
    {
      qw *= +1.0;
      qx *= SIGN(r32 - r23);
      qy *= SIGN(r13 - r31);
      qz *= SIGN(r21 - r12);
    }
  else if(qx >= qw && qx >= qy && qx >= qz)
    {
      qw *= SIGN(r32 - r23);
      qx *= +1.0;
      qy *= SIGN(r21 + r12);
      qz *= SIGN(r13 + r31);
    }
  else if(qy >= qw && qy >= qx && qy >= qz)
    {
      qw *= SIGN(r13 - r31);
      qx *= SIGN(r21 + r12);
      qy *= +1.0;
      qz *= SIGN(r32 + r23);
    }
  else if(qz >= qw && qz >= qx && qz >= qy)
    {
      qw *= SIGN(r21 - r12);
      qx *= SIGN(r31 + r13);
      qy *= SIGN(r32 + r23);
      qz *= +1.0;
    }
  else
    {
      printf("coding error\n");
    }
  double r = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= r;
  qx /= r;
  qy /= r;
  qz /= r;

  pose[3] = qw;
  pose[4] = qx;
  pose[5] = qy;
  pose[6] = qz;

  // Translation
  pose[0] = frame.at<double>(0,3);
  pose[1] = frame.at<double>(1,3);
  pose[2] = frame.at<double>(2,3);
  return pose;
}


/** Quaternion multiplication
 *  It's not used right now and it's not tested!
 */
void multiplyFrame7(std::vector<double> q1,std::vector<double> q2, std::vector<double>* q)
{
  // First quaternion q1 (x1 y1 z1 r1)
  double x1=q1[4];
  double y1=q1[5];
  double z1=q1[6];
  double r1=q1[3];

  // Second quaternion q2 (x2 y2 z2 r2)
  double x2=q2[4];
  double y2=q2[5];
  double z2=q2[6];
  double r2=q2[3];

  //translation
  q->push_back(q1[0]+q2[0]); 
  q->push_back(q1[1]+q2[1]);
  q->push_back(q1[2]+q2[2]);

  //quaternion
  q->push_back(r1*r2 - x1*x2 - y1*y2 - z1*z2);   // r component
  q->push_back(x1*r2 + r1*x2 + y1*z2 - z1*y2);   // x component
  q->push_back(r1*y2 - x1*z2 + y1*r2 + z1*x2);   // y component
  q->push_back(r1*z2 + x1*y2 - y1*x2 + z1*r2);   // z component
}

bool getCameraPose(seneka_sensornode_detection::getCameraPose::Request &req,
		seneka_sensornode_detection::getCameraPose::Response &res){
	
	res.camera_pose = camera_pose_;
	res.success = true;
	
	return true;
}

bool setCameraPose(seneka_sensornode_detection::setCameraPose::Request &req,
		seneka_sensornode_detection::setCameraPose::Response &res){
	
	geometry_msgs::Point rpy_in_rad;
	
	rpy_in_rad.x = req.rpy.x * PI / 180;
	rpy_in_rad.y = req.rpy.y * PI / 180;
	rpy_in_rad.z = req.rpy.z * PI / 180;
	
	tf::Quaternion qt = tf::createQuaternionFromRPY(-PI/2 + rpy_in_rad.x,
													0 + rpy_in_rad.y ,
													-PI/2 + rpy_in_rad.z);
	
	camera_pose_.position = req.position;
	camera_pose_.orientation.w = qt.getW();
	camera_pose_.orientation.x = qt.getX();
	camera_pose_.orientation.y = qt.getY();
	camera_pose_.orientation.z = qt.getZ();	
	
	return true;
}

bool calibrateCamera(seneka_sensornode_detection::calibrateCamera::Request &req,
		seneka_sensornode_detection::calibrateCamera::Response &res){
	
	extrinsic_camera_calib_ = true;
	
	res.success = true;
	return res.success;
}

bool getExtCalibration(seneka_sensornode_detection::getExtCalibration::Request &req,
		seneka_sensornode_detection::getExtCalibration::Response &res){
	
	res.pose = camera_pose_;
	
	res.success = true;
	return res.success;
}

bool compensateInaccuracy (seneka_sensornode_detection::compensateInaccuracy::Request &req,
		seneka_sensornode_detection::compensateInaccuracy::Response &res){

//	for(uint i=0; i < trigger_offset.size(); i++){
//		
//		trigger_offset[i].up.trans.x = trigger_offset[i]..up.x + req.gripper_depth_offset;
//		trigger_offset[i].down.trans.x = trigger_offset[i].down.x + req.gripper_depth_offset;
//	}
	
	trigger_offset.up.x = trigger_offset.up.x + req.gripper_depth_offset;
	trigger_offset.down.x = trigger_offset.down.x + req.gripper_depth_offset;
	
	res.gripper_depth_offset = req.gripper_depth_offset;
	res.success = true;
	
	return res.success;
}

bool setSensornodePose (seneka_sensornode_detection::setSensornodePose::Request &req,
		seneka_sensornode_detection::setSensornodePose::Response &res){
	
	sensornodeSimulatedPose_ = req.pose;
	
	res.success = true;
	return res.success;
}

//-----------------------------------main----------------------------------------------------------------------------
int main( int argc, char** argv )
{  
  ros::init(argc, argv, "sensornode_detection");
  ros::NodeHandle node;
 
  init(node);
  
  ros::Subscriber sub = node.subscribe("/fiducials/fiducial_detection_array", 1, getSensornodeStateCamera);
  service_setCameraPose_ = node.advertiseService("setCameraPose", setCameraPose);
  service_getCameraPose_ = node.advertiseService("getCameraPose", getCameraPose);
  service_calibrateCamera_ = node.advertiseService("calibrateCamera", calibrateCamera);
  service_getExtCalibration_ = node.advertiseService("getExtCalibration", getExtCalibration);
  service_compensateInaccuracy_ = node.advertiseService("compensateInaccuracy", compensateInaccuracy);
  service_setSensornodePose_ = node.advertiseService("setSensornodePose", setSensornodePose);
  
  if(!loadParameters(&fiducialmarkers,&handles,&trigger_offset,&grab_entry)){
    ROS_ERROR("Failed to load parameters. Check the path...");
    return 0;
  } else {
    ROS_INFO("Congratulations! All parameters succesfully loaded :)!!.");
  }

  while(ros::ok() && simulation_){
	  //getSensornodeStateGazebo(node);
	  setSensornodePose(sensornodeSimulatedPose_);
	  ros::spinOnce();
  }
 
  ros::spin();
  return 0;
}

