/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/time.h>
#include "nav_msgs/MapMetaData.h"

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_G 0x67
#define KEYCODE_R 0x72

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double x_val = 0;
double y_val = 0;
double gaz_ori_x, gaz_ori_y, gaz_ori_z, gaz_ori_w;
double rv_ori_x, rv_ori_y, rv_ori_z, rv_ori_w;
double x_map = 0;
double y_map = 0;

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
	printf("The y in the world is %f \n\n", y_val);
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
  printf("The y in the map is %f \n\n", y_map);
}

class TeleopPR2Keyboard
{
  private:
  geometry_msgs::PoseWithCovarianceStamped pose;
  gazebo_msgs::ModelState gazebo_pose;
  
  ros::Publisher gazebo_pose_pub_;
  ros::NodeHandle n_;
  ros::Publisher amcl_pose_pub_;
  ros::Subscriber map_pose_sub_;
  ros::Subscriber sub_;
  
  public:
  void init()
  { 
    gazebo_pose_pub_ = n_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    amcl_pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    sub_ = n_.subscribe("/gazebo/model_states", 1, model_statesCallBack);
    map_pose_sub_ = n_.subscribe("amcl_pose", 1, map_statesCallBack);
    ros::NodeHandle n_private("~");
  }
  
  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  while(ros::ok())
  {
	  
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    ros::spinOnce();
    switch (c){
      case KEYCODE_G:
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
        
        pose.header.frame_id = "/map";
        pose.pose.pose.position.x = x_map;
        pose.pose.pose.position.y = y_map;
        pose.pose.pose.orientation.x = rv_ori_x;
        pose.pose.pose.orientation.y = rv_ori_y;
        pose.pose.pose.orientation.w = rv_ori_w;
        pose.pose.pose.orientation.z = rv_ori_z;
        pose.pose.covariance[6*0+0] = 0.5 * 0.5;
        pose.pose.covariance[6*1+1] = 0.5 * 0.5;
        pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
        amcl_pose_pub_.publish(pose);
        break;
      case KEYCODE_R:
        pose.header.frame_id = "/map";
        pose.pose.pose.position.x = x_val;
        pose.pose.pose.position.y = y_val;
        pose.pose.pose.orientation.x = gaz_ori_x;
        pose.pose.pose.orientation.y = gaz_ori_y;
        pose.pose.pose.orientation.w = gaz_ori_w;
        pose.pose.pose.orientation.z = gaz_ori_z;
        pose.pose.covariance[6*0+0] = 0.5 * 0.5;
        pose.pose.covariance[6*1+1] = 0.5 * 0.5;
        pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
        amcl_pose_pub_.publish(pose);
        break;
    }
  }
}
