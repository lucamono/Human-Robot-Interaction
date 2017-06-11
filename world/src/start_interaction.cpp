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
#include <joystick_teleop/msg.h>


using namespace std;

std::string command_vel_topic;
std::string command_vel_joy_topic;
std::string command_vel_follow_topic;
std::string command_button_topic;
std::string edge_leg_detector_topic;
std::string human_pose_topic;
geometry_msgs::Twist twist_follow;
geometry_msgs::Twist twist_joy;
geometry_msgs::Twist twist_update;
geometry_msgs::PoseArray human_poses_filters;
// per ripubblicare senza falsi positivi
geometry_msgs::Pose HumanPose;
geometry_msgs::Point HumanPoint;
geometry_msgs::Quaternion HumanQuaternion;
ros::Publisher array_pose_publisher;
ros::Publisher cmd_vel_publisher;
//init is the normal Joystick
int mode=0;
float damping=1.0;

void commandVel_follow_Callback(const geometry_msgs::TwistConstPtr twist){
                //mode 1 follow humans		
                twist_follow.linear.x = twist->linear.x;
		twist_follow.angular.z = twist->angular.z;
}

void commandVel_joy_Callback(const geometry_msgs::TwistConstPtr twist2){
		//mode 0 normalJoy - mode 1 follow humans+normalJoy - mode 2 avoidanceJoy			
		twist_joy.linear.x = twist2->linear.x*damping;
		twist_joy.angular.z = twist2->angular.z*damping;
		if(mode == 1){
			twist_update.linear.x = twist_joy.linear.x + twist_follow.linear.x;
			twist_update.angular.z = twist_joy.angular.z + twist_follow.angular.z;
		}
		else{
			twist_update.linear.x = twist_joy.linear.x;
			twist_update.angular.z = twist_joy.angular.z;
		}
		cmd_vel_publisher.publish(twist_update);
}

void commandButton_joy_Callback(const joystick_teleop::msg button){
		sound_play::SoundClient sc;
		if(button.type==1 && button.value==1 && button.number==0){
			mode =1; 			
			sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/follow_mode.mp3");
			cout << "follow_mode" << endl;
		}		
		else if(button.type==1 && button.value==1 && button.number==1){
			mode =0;
			sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/joystick_mode.mp3");
			cout << "joystick mode" << endl;
		}
		else if(button.type==1 && button.value==1 && button.number==2){
			mode =2;
			sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/avoidanceJoy.mp3");	
			cout << "avoidance joy" << endl;
		}	
		else{
			mode =mode;
		}
}

void arrayPoseCallback(const geometry_msgs::PoseArrayConstPtr poseArray){
	if((poseArray->poses.size() > 0) && mode == 2){
		for(int i = 0; i < poseArray->poses.size(); i++ ){   
			if(poseArray->poses[i].position.x > 1.5 && poseArray->poses[i].position.y > 1.5){
				damping = 0.5;
			}
			if((poseArray->poses[i].position.x <= 1.5 && poseArray->poses[i].position.x > 0.7) || (poseArray->poses[i].position.y <= 1.5 && poseArray->poses[i].position.y > 0.7)){
				damping = 0.5;
			}
			else{
				damping = 0.0;
			}
	 	
		}
	}
	else{
		damping = 1.0;
	}
}


//Sound speech method
void sleepok(int t, ros::NodeHandle &nh){
   if (nh.ok())
       sleep(t);
 }

int main(int argc, char** argv) {
  ros::init(argc, argv, "start_interaction");
  ros::NodeHandle nh("~");
  
  //speech params
  sound_play::SoundClient sc;
  sleepok(1, nh);

  nh.param("command_vel_joy_topic", command_vel_joy_topic, std::string("/cmd_vel_joy"));
  nh.param("command_vel_follow_topic", command_vel_follow_topic, std::string("/cmd_vel_follow"));
  nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
  nh.param("command_button_topic", command_button_topic, std::string("/cmd_button"));
  nh.param("human_pose_topic", human_pose_topic, std::string("/human_pose"));
  
ros::Subscriber command_vel_follow_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>      	     (command_vel_follow_topic, 1, &commandVel_follow_Callback);

  ros::Subscriber command_vel_joy_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>      	     (command_vel_joy_topic, 1, &commandVel_joy_Callback);

 ros::Subscriber command_button_subscriber = nh.subscribe<joystick_teleop::msg>      	     (command_button_topic, 1, &commandButton_joy_Callback);

  ros::Subscriber array_pose_subscriber = nh.subscribe<geometry_msgs::PoseArray>      	     (human_pose_topic, 1, &arrayPoseCallback);

  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(command_vel_topic, 1);
  array_pose_publisher = nh.advertise<geometry_msgs::PoseArray>(human_pose_topic, 1);
 // ros::Rate r(50);
	
 sc.playWave("/home/marrtino/catkin_ws/src/hri/audio_mp3/start.mp3");
 sleepok(2, nh);		
  


  ros::spin();
  return 0;  
}
