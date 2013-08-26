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
#include "std_msgs/String.h"

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <iostream>
using namespace std;

int x[8] = {54, 98, 202, 312, 96, 160, 232, 314};
int y[8] = {90, 74, 62, 74, 227, 259, 189, 222};
int image_height;
int goal_number;
double x_val = 0;
double y_val = 0;
double gaz_ori_x, gaz_ori_y, gaz_ori_z, gaz_ori_w;
double rv_ori_x, rv_ori_y, rv_ori_z, rv_ori_w;
double x_map = 0;
double y_map = 0;
double res;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void model_statesCallBack(const gazebo::ModelStates::ConstPtr& msg){
	printf("Got world\n");
	unsigned int i=0;
	for (;i<msg->name.size();i++){
		if (strcmp(msg->name[i].c_str(),"pr2")==0)
			break;
	}
	x_val = msg->pose[i].position.x;
	y_val = msg->pose[i].position.y;
	gaz_ori_x = msg->pose[i].orientation.x;
  gaz_ori_y = msg->pose[i].orientation.y;
	gaz_ori_z = msg->pose[i].orientation.z;
	gaz_ori_w = msg->pose[i].orientation.w;
	printf("The x in the world is %f \n", x_val);
	printf("The y in the world is %f \n", y_val);
}

void map_statesCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	printf("Got map\n");
  x_map = msg->pose.pose.position.x;
  y_map = msg->pose.pose.position.y;
  rv_ori_x = msg->pose.pose.orientation.x;
  rv_ori_y = msg->pose.pose.orientation.y;
	rv_ori_z = msg->pose.pose.orientation.z;
	rv_ori_w = msg->pose.pose.orientation.w;
  printf("The x in the map is %f \n", x_map);
  printf("The y in the map is %f \n", y_map);
}

void resCallBack(const nav_msgs::MapMetaDataConstPtr& mapinfo){
  res=mapinfo->resolution;
  image_height = mapinfo->height;
}

class TeleopPR2Keyboard
{
  private:
  gazebo_msgs::ModelState gazebo_pose;
  geometry_msgs::PoseWithCovarianceStamped map_pose;
  nav_msgs::MapMetaData meta_data_message_;
  nav_msgs::GetMap::Response map_resp_;

  ros::NodeHandle n;
  ros::Publisher gazebo_pose_pub_;
  ros::Publisher map_pose_pub_;
  ros::Publisher map_pub_;
  ros::Publisher metadata_pub_;
  ros::Publisher pick_place_pub_;
  
  ros::Subscriber res_sub_;
  ros::Subscriber map_pose_sub_;
  ros::Subscriber world_pose_sub_;
  ros::NodeHandle n_;

  public:
  void init()
  { 
    goal_number = 0;
    res_sub_=n.subscribe<nav_msgs::MapMetaData>("map_metadata", 1, resCallBack);
    world_pose_sub_ = n_.subscribe("/gazebo/model_states", 1, model_statesCallBack);
    map_pose_sub_ = n_.subscribe("amcl_pose", 1, map_statesCallBack);

    map_pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    gazebo_pose_pub_ = n_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    metadata_pub_ = n.advertise<nav_msgs::MapMetaData>("mymap_metadata", 1, true);
    map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("mymap", 1, true);
    pick_place_pub_ = n.advertise<std_msgs::String>("pick_place", 1);
    ros::NodeHandle n_private("~");
  }

  public:
  ~TeleopPR2Keyboard();
  void loop();
  void synRviz();
  void synWorld();
  void sendMap();
  void sendPose();
  void sendGoal();
  void pick();
  void drop();
  void open_pick();
  void open_drop();
  void close_pick();
  void close_drop();
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  TeleopPR2Keyboard tpk;
  tpk.init();
  tpk.loop();
  return 0;
}

void TeleopPR2Keyboard::sendPose(){
  ros::spinOnce();
  double worldx, worldy, ox, oy, oz, ow;  
  int mapx, mapy;
  char option;
  cout<<"Enter y for pose in world coordinate, n for pose in map coordinate:"<<endl;
  cin >>option;
  if(option=='y'){
     cout<<"Enter x, y position( in meter ):"<<endl;
     cin>>worldx>>worldy;
  }
  else if (option=='n'){
     cout<<"Enter x,y position( in pixel ):"<<endl;
     cin>>mapx>>mapy;
     worldx =  mapx  * res;
     worldy =  mapy  * res;
  };
  cout<<"Enter orientation in x, y, z, w direction: "<<endl;
  cin>>ox>>oy>>oz>>ow;
  cin.ignore(10,'\n');
  map_pose.header.frame_id = "/map";
  map_pose.pose.pose.position.x = worldx;
  map_pose.pose.pose.position.y = worldy;
  map_pose.pose.pose.orientation.x = ox;
  map_pose.pose.pose.orientation.y = oy;
  map_pose.pose.pose.orientation.z = oz;
  map_pose.pose.pose.orientation.w = ow;
  map_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  map_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  map_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  map_pose_pub_.publish(map_pose);
  return;
}

void TeleopPR2Keyboard::sendGoal(){
  ros::spinOnce();
  double worldx,worldy;  
  int mapx,mapy;
  char option;
  
  cout<<"Enter y for goal in world coordinate, n for goal in map coordinate, g for goal we defined:"<<endl;
  cin >>option;
  if(option=='y'){
    cout<<"Enter x, y position( in meter ):"<<endl;
    cin>>worldx>>worldy;
  }
  else if (option=='n'){
    cout<<"Enter x, y position( in pixel ):"<<endl;
    cin>>mapx>>mapy;
    worldx =  mapx  * res;
    worldy =  mapy  * res;
  }
  else if (option == 'g'){
    worldx = x[goal_number] * res;
    worldy = (image_height-y[goal_number]) * res;
    cout<<x[goal_number]<<" "<<worldx<<endl;
    cout<<y[goal_number]<<" "<<worldy<<endl;
    goal_number++;
    if (goal_number == 8)
      goal_number = 0;
  }
  cin.ignore(10,'\n');

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = worldx;
  goal.target_pose.pose.position.y = worldy;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  return;
}

void TeleopPR2Keyboard::synRviz(){
  ros::spinOnce();
  map_pose.header.frame_id = "/map";
  map_pose.pose.pose.position.x = x_val;
  map_pose.pose.pose.position.y = y_val;
  map_pose.pose.pose.orientation.x = gaz_ori_x;
  map_pose.pose.pose.orientation.y = gaz_ori_y;
  map_pose.pose.pose.orientation.w = gaz_ori_w;
  map_pose.pose.pose.orientation.z = gaz_ori_z;
  map_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  map_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  map_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  map_pose_pub_.publish(map_pose);
  return;
}

void TeleopPR2Keyboard::synWorld(){
  ros::spinOnce();
  gazebo_pose.model_name = "pr2";
  gazebo_pose.pose.position.x = x_map;
  gazebo_pose.pose.position.y = y_map;
  gazebo_pose.pose.position.z = 0.0;
  gazebo_pose.pose.orientation.x = rv_ori_x;
  gazebo_pose.pose.orientation.y = rv_ori_y;
  gazebo_pose.pose.orientation.z = rv_ori_z;
  gazebo_pose.pose.orientation.w = rv_ori_w;
  gazebo_pose.twist.linear.x = 0;
  gazebo_pose.twist.linear.y = 0;
  gazebo_pose.twist.linear.z = 0;
  gazebo_pose.twist.angular.x = 0;
  gazebo_pose.twist.angular.y = 0;
  gazebo_pose.twist.angular.z = 0;
  gazebo_pose.reference_frame = "world";
  gazebo_pose_pub_.publish(gazebo_pose);
  
  sleep(1);
  
  map_pose.header.frame_id = "/map";
  map_pose.pose.pose.position.x = x_map;
  map_pose.pose.pose.position.y = y_map;
  map_pose.pose.pose.orientation.x = rv_ori_x;
  map_pose.pose.pose.orientation.y = rv_ori_y;
  map_pose.pose.pose.orientation.w = rv_ori_w;
  map_pose.pose.pose.orientation.z = rv_ori_z;
  map_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  map_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  map_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  map_pose_pub_.publish(map_pose);
  return;
}

void TeleopPR2Keyboard::sendMap(){
  ros::spinOnce();
  std::string mapfname;
  double res;
  double origin[3];
  int negate;
  double occ_th, free_th;
      
  cout<<"Enter file name (q or quit to exit): "<<endl;
  getline(cin, mapfname);
  if (mapfname.compare("q") == 0 || mapfname.compare("quit") == 0)
    return;
  cout<<"Enter resolution: "<<endl;
  cin>>res;
  cin.ignore(10,'\n');
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

void TeleopPR2Keyboard::loop(){
  char option;
  while (ros::ok()){
    ros::spinOnce();
    cout<<"Enter i to set initial pose, g to send goal, "<<endl;
    cout<<"r to synchronize rviz, w to synchronize world, "<<endl;
    cout<<"p to give bottle, d to give medicine, 1 to pick bottle, "<<endl;
    cout<<"2 to drop bottle, 3 to pick medicine, 4 to drop medicine, "<<endl;
    cout<<"m to send new map, or q to quit: "<<endl;
    cin>>option;
    cin.ignore(10,'\n');    
    switch (option){
      case 'i':
        sendPose();
        break;
      case 'g':
        sendGoal();
        break;
      case 'r':
        synRviz();
        break;
      case 'w':
        synWorld();
        break;
      case 'm':
        sendMap();
        break;
      case 'q':
        return;
      case 'p':
        pick();
        break;
      case 'd':
        drop();
        break;
      case '1':
        open_pick();
        break;
      case '2':
        open_drop();
        break;
      case '3':
        close_pick();
        break;
      case '4':
        close_drop();
        break;
      default:
        break;
    }
  }
}

TeleopPR2Keyboard::~TeleopPR2Keyboard(){}

void TeleopPR2Keyboard::pick(){
  std_msgs::String msg;
  msg.data = "give bottle";
  pick_place_pub_.publish(msg);
}

void TeleopPR2Keyboard::drop(){
  std_msgs::String msg;
  msg.data = "give medicine";
  pick_place_pub_.publish(msg);
}

void TeleopPR2Keyboard::open_pick(){
  std_msgs::String msg;
  msg.data = "pick bottle";
  pick_place_pub_.publish(msg);
}

void TeleopPR2Keyboard::open_drop(){
  std_msgs::String msg;
  msg.data = "drop bottle";
  pick_place_pub_.publish(msg);
}

void TeleopPR2Keyboard::close_pick(){
  std_msgs::String msg;
  msg.data = "pick medicine";
  pick_place_pub_.publish(msg);
}

void TeleopPR2Keyboard::close_drop(){
  std_msgs::String msg;
  msg.data = "drop medicine";
  pick_place_pub_.publish(msg);
}
