#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <list>
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

std::string edge_leg_detector_topic;
std::string human_pose_topic;
geometry_msgs::PoseArray human_poses_filters;
// per ripubblicare senza falsi positivi
geometry_msgs::Pose HumanPose;
geometry_msgs::Point HumanPoint;
geometry_msgs::Quaternion HumanQuaternion;
ros::Subscriber array_pose_subscriber;
ros::Publisher array_pose_publisher;


void arrayPoseCallback(const geometry_msgs::PoseArrayConstPtr poseArray){
	human_poses_filters.poses.clear();
	bool processed[poseArray->poses.size()];	
	for(int i = 0; i < poseArray->poses.size(); i++ ){
		processed[i]=false;
	}	
	for(int i = 0; i < poseArray->poses.size(); i++ ){   
		if(processed[i]==false){ 
    			for(int j = i+1; j < poseArray->poses.size(); j++){
  				if((processed[j]==false) && (sqrt(pow(poseArray->poses[i].position.x-poseArray->poses[j].position.x,2) + pow(poseArray->poses[i].position.y-poseArray->poses[j].position.y,2)) < 0.40)){
					HumanPoint.x = (poseArray->poses[i].position.x+poseArray->poses[j].position.x)/2;
					HumanPoint.y = (poseArray->poses[i].position.y+poseArray->poses[j].position.y)/2;
					HumanPoint.z = 0; 
					HumanQuaternion.x = 0;//|-> Orientation is ignored
					HumanQuaternion.y = 0;//|
					HumanQuaternion.z = 0;//|
					HumanQuaternion.w = 1;//|
					HumanPose.position = HumanPoint;
					HumanPose.orientation= HumanQuaternion;
					human_poses_filters.header.frame_id = poseArray->header.frame_id;
					human_poses_filters.poses.push_back( HumanPose );
					processed[j]=true;
					processed[i]=true;
				}
			}			
		}
 	}
		
	array_pose_publisher.publish(human_poses_filters);
	//human_poses_filters.poses.clear();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_to_people_node");
  ros::NodeHandle nh("~");
  nh.param("edge_leg_detector_topic", edge_leg_detector_topic, std::string("/edge_leg_detector"));
  nh.param("human_pose_topic", human_pose_topic, std::string("/human_pose"));
  array_pose_subscriber  = nh.subscribe<geometry_msgs::PoseArray>      	     (edge_leg_detector_topic, 1,    	&arrayPoseCallback);
  array_pose_publisher= nh.advertise<geometry_msgs::PoseArray>(human_pose_topic, 1);
  ros::spin();
  return 0;
}
