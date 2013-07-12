#include <ros/ros.h>
#include "ros/console.h"
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "map_server/image_loader.h"
#include "yaml-cpp/yaml.h"
#include <gazebo/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_datatypes.h>
#include <SDL/SDL_image.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <iostream>
#include <cmath>
#define stage_1 1
#define stage_2 2
#define stage_3 3
#define ROTATE 0
#define MOVING 1 
using namespace std;
bool start=false;

int x[8] = {49, 98, 202, 312, 96, 160, 232, 314};
int y[8] = {90, 74, 62, 74, 227, 259, 189, 222};
int goal_number;
double goal_x, goal_y;
int stage_number, state_;

double x_val;
double y_val;
double gaz_ori_x, gaz_ori_y, gaz_ori_z, gaz_ori_w;
double rv_ori_x, rv_ori_y, rv_ori_z, rv_ori_w;
double x_map;
double y_map;
double res = 0.044875;
int image_height = 300;
bool is_map_1_;

actionlib_msgs::GoalID goal_id_;
move_base_msgs::MoveBaseGoal goal_;
geometry_msgs::Twist cmd;

geometry_msgs::PoseWithCovarianceStamped map_pose;
nav_msgs::MapMetaData meta_data_message_;
nav_msgs::GetMap::Response map_resp_;
ros::Publisher map_pose_pub_;
ros::Publisher map_pub_;
ros::Publisher metadata_pub_;
ros::Publisher cancel_goal_;
ros::Publisher vel_pub_;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void map_statesCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void sendGoal1();
void sendGoal2();
void sendMap();
void cancelGoal();

void goalCallBack(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
  ROS_INFO("Got goal");
  goal_id_ = msg->goal_id;
  goal_ = msg->goal;
  start = true;
  state_ = ROTATE;
}

void map_statesCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  if (!start) return;
  x_map = msg->pose.pose.position.x;
  y_map = msg->pose.pose.position.y;
  rv_ori_x = msg->pose.pose.orientation.x;
  rv_ori_y = msg->pose.pose.orientation.y;
	rv_ori_z = msg->pose.pose.orientation.z;
	rv_ori_w = msg->pose.pose.orientation.w;
	double destination_yaw = atan2((goal_.target_pose.pose.position.y-y_map),(goal_.target_pose.pose.position.x-x_map));
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(goal_.target_pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	double yaw_val = yaw;
	cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
	if (stage_number == stage_1){
	  if (pow((x_map - goal_.target_pose.pose.position.x), 2) + pow((y_map - goal_.target_pose.pose.position.y), 2) < pow (1, 2) ){
      cancelGoal();
      stage_number = stage_2;
    }
	}
	else if (stage_number == stage_2){
	  ROS_INFO("Got stage 2");
	  switch(state_){
		  case ROTATE:
			  ROS_INFO("rotating");
			  if (destination_yaw - yaw_val < 0.05 && destination_yaw - yaw_val > -0.05){
				  state_ = MOVING;
				    ROS_INFO("Go to moving");}
			  else
				  if (destination_yaw > yaw_val){
					  cmd.angular.z = 0.5;
				  }
				  else {
					  cmd.angular.z = -0.5;
				  }
			  break;
		  case MOVING:
		    ROS_INFO("moving");
			  if (goal_.target_pose.pose.position.x - x_val < 5 && goal_.target_pose.pose.position.x - x_val > -5 && goal_.target_pose.pose.position.y - y_val < 5 && goal_.target_pose.pose.position.y - y_val > -5 ){
				  stage_number = stage_3;
				    ROS_INFO("finfished");
        }
			  else{
					  cmd.linear.x = 0.5;
					  if (destination_yaw > yaw_val + 0.05){
						  cmd.angular.z = 0.5;
					  }
					  else if (destination_yaw - yaw_val < -0.05){
						  cmd.angular.z = -0.5;
					  }
			  }
			  break;
	  }
	  vel_pub_.publish(cmd);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constant_goal_publisher");
  ros::NodeHandle n;
  ros::Subscriber map_pose_sub_;
  ros::Subscriber world_pose_sub_;
  
  world_pose_sub_ = n.subscribe("/move_base/goal", 1, goalCallBack);
  map_pose_sub_ = n.subscribe("amcl_pose", 1, map_statesCallBack);
  map_pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  metadata_pub_ = n.advertise<nav_msgs::MapMetaData>("mymap_metadata", 1, true);
  map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("mymap", 1, true);
  cancel_goal_ = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1, true);
  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  stage_number = stage_1;
  start = false;
  is_map_1_ = true;

  ros::spin();
  return 0;
}


void sendGoal1(){
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.orientation.x = rv_ori_x;
  goal.target_pose.pose.orientation.y = rv_ori_y;
  goal.target_pose.pose.orientation.z = rv_ori_z;
  goal.target_pose.pose.orientation.w = rv_ori_w;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  return;
}

void sendGoal2(){
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  return;
}

void sendMap(){
  std::string mapfname;
  double origin[3];
  int negate;
  double occ_th, free_th;
  
  if (is_map_1_)
    mapfname = "aptemptyhousemap_1.pgm";
  else
    mapfname = "apthousemap_1.pgm";
  std::string frame_id;
  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", frame_id, std::string("map"));
  private_nh.param("negate", negate, 0);
  private_nh.param("occupied_thresh", occ_th, 0.65);
  private_nh.param("free_thresh", free_th, 0.196);
  origin[0] = origin[1] = origin[2] = 0.0;

  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin);
  map_resp_.map.info.map_load_time = ros::Time::now();
  map_resp_.map.header.frame_id = frame_id;
  map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp_.map.info.width,
           map_resp_.map.info.height,
           map_resp_.map.info.resolution);
  meta_data_message_ = map_resp_.map.info;

  // Latched publisher for metadata
  metadata_pub_.publish( meta_data_message_ );
  
  // Latched publisher for data
  map_pub_.publish( map_resp_.map );
}

void cancelGoal(){
ROS_INFO("cancel goal");
  cancel_goal_.publish(goal_id_);
}
