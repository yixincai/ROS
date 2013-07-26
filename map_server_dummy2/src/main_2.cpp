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
#include <robot_sim_nlp/JPArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <iostream>
#include <cmath>
#define stage_1 1
#define stage_2 2
using namespace std;

double res = 0.044875;
int image_height = 300;
bool is_map_1_;
nav_msgs::GetMap::Response map_resp_;
ros::Publisher map_pub_;

void sendMap();

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void mapCallBack(const robot_sim_nlp::JPArray::ConstPtr& msg){
  int index = 0;

  for(unsigned int y=0; y<map_resp_.map.info.height; y++){
	for(unsigned int x=0; x<map_resp_.map.info.width; x++){
		map_resp_.map.data[index] = msg->data[index];
		index++;					
	}				
  }
  map_resp_.map.info.map_load_time = ros::Time::now();
  map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp_.map.info.width,
           map_resp_.map.info.height,
           map_resp_.map.info.resolution);

  // Latched publisher for data
  map_pub_.publish( map_resp_.map );
}

void goalCallBack(const geometry_msgs::Point::ConstPtr& msg){
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = msg->x*res;
  goal.target_pose.pose.position.y = (image_height-msg->y) * res;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg->z);
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constant_goal_publisher");
  ros::NodeHandle n;
  ros::Subscriber goal_sub_;
  ros::Subscriber map_sub_;
  
  map_sub_ = n.subscribe("sim_costmap", 1, mapCallBack);
  goal_sub_ = n.subscribe("sim_goal", 1, goalCallBack);

  map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("mymap", 1, true);
  sendMap();
  ros::spin();
  return 0;
}

void sendMap(){
  std::string mapfname;
  double origin[3];
  int negate;
  double occ_th, free_th;
  
  if (is_map_1_)
    mapfname = "/home/jfasola/fuerte_workspace/sandbox/map_server_dummy2/aptemptyhousemap_1.pgm";
  else
    mapfname = "/home/jfasola/fuerte_workspace/sandbox/map_server_dummy2/apthousemap_1.pgm";
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
  
  // Latched publisher for data
  map_pub_.publish( map_resp_.map );
}

