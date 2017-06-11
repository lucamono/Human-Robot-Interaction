#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <linux/joystick.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <list>
#include <vector>
#include <math.h>
#include <sound_play/sound_play.h>
#include <iostream>
#include <face_person_detect/msg1.h>

using namespace std;

std::string command_vel_follow_topic;
std::string human_pose_topic;
std::string face_CvPoint_topic;
geometry_msgs::Twist twist_update;
geometry_msgs::PoseArray human_poses_filters;
// per ripubblicare senza falsi positivi
geometry_msgs::Pose HumanPose;
geometry_msgs::Point HumanPoint;
geometry_msgs::Quaternion HumanQuaternion;
ros::Publisher cmd_vel_follow_publisher;
float lin_vel = 0.0;
float ang_vel = 0.0;
bool command=false; 
float damping=0.0;

int pointX=0;

void faceCvPointCallback(const face_person_detect::msg1ConstPtr faceCvPoint){
    pointX=faceCvPoint->x;
    cout << pointX << endl;
}

void arrayPoseCallback(const geometry_msgs::PoseArrayConstPtr poseArray){
	float range = 0.1;
	if(poseArray->poses.size() > 0){
		float human_orient = 0.0;
		geometry_msgs::Pose person_pose = poseArray->poses[0];
		human_orient = atan2(person_pose.position.y,person_pose.position.x);
		if(human_orient > 0.0 + range){
			lin_vel = 0.0;
			ang_vel = 0.6;			
		}
		else if(human_orient < (0.0 - range)){
			lin_vel = 0.0;
			ang_vel = -0.4;			
		}
		else 
		{	if(person_pose.position.x>0.7){
				lin_vel = 0.2;
				ang_vel = 0.0;
			}
			else{
  				sound_play::SoundClient sc;
				sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/interaction.mp3");
    			}
		}
	}
	else{
		lin_vel = 0.0;
		ang_vel = 0.0;	
	}
	twist_update.linear.x=lin_vel;
        twist_update.angular.z=ang_vel;
        cmd_vel_follow_publisher.publish(twist_update);
}


//Sound speech method
void sleepok(int t, ros::NodeHandle &nh){
   if (nh.ok())
       sleep(t);
 }


int main(int argc, char** argv) {
  ros::init(argc, argv, "start_follow");
  ros::NodeHandle nh("~");
  sound_play::SoundClient sc;
  //sound condition
  sleepok(1, nh);

  nh.param("command_vel_follow_topic", command_vel_follow_topic, std::string("/cmd_vel_follow"));
  nh.param("human_pose_topic", human_pose_topic, std::string("/human_pose"));
  nh.param("face_CvPoint_topic", face_CvPoint_topic, std::string("/face_CV_Point"));

  ros::Subscriber face_CvPoint_subscriber = nh.subscribe<face_person_detect::msg1>      	     (face_CvPoint_topic, 1, &faceCvPointCallback);
  ros::Subscriber array_pose_subscriber = nh.subscribe<geometry_msgs::PoseArray>      	     (human_pose_topic, 1, &arrayPoseCallback);

  cmd_vel_follow_publisher = nh.advertise<geometry_msgs::Twist>(command_vel_follow_topic, 1);
  
 // sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/follow_mode.mp3");  
  sleepok(2, nh);
  ros::spin();
  return 0;
}
