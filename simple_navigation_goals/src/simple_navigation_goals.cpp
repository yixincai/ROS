#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/time.h>
#include "nav_msgs/MapMetaData.h"


using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double res;
void callback(const nav_msgs::MapMetaDataConstPtr& mapinfo){
  res=mapinfo->resolution;
}

class TeleopPR2Keyboard
{
  private:
  ros::NodeHandle n;
  ros::Publisher map_pose_pub;
  geometry_msgs::PoseWithCovarianceStamped pose;
  ros::Subscriber subres;

  public:
  void init()
  { 
    subres=n.subscribe<nav_msgs::MapMetaData>("map_metadata", 1, callback);
    map_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    ros::NodeHandle n_private("~");
  }

  public:
  ~TeleopPR2Keyboard();
  void loop();
  void sendPose();
  void sendGoal();
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

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
  cout<<"Enter y for goal in world coordinate, n for goal in map coordinate:"<<endl;
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
  pose.header.frame_id = "/map";
  pose.pose.pose.position.x = 3;
  pose.pose.pose.position.y = 3;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;
  pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  map_pose_pub.publish(pose);
  return;
}

void TeleopPR2Keyboard::sendGoal(){
  ros::spinOnce();
  double worldx,worldy;  
  int mapx,mapy;
  char option;
  cout<<"Enter y for goal in world coordinate, n for goal in map coordinate:"<<endl;
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
  };
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
  
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    return;
}


void TeleopPR2Keyboard::loop(){
  char option;
  while (ros::ok()){
    ros::spinOnce();
    cout<<"Enter i to set initial pose, g to send goal, or q to quit: "<<endl;
    cin>>option;
    switch (option){
      case 'i':
        sendPose();
        break;
      case 'g':
        sendGoal();
        break;
      case 'q':
        return;
      default:
        break;
    }
    cin.ignore(10,'\n');
  }

}
TeleopPR2Keyboard::~TeleopPR2Keyboard(){}
