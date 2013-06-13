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
#include <gazebo/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_datatypes.h>

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

#define ROTATE 0
#define MOVING 1 
using namespace std;

double x_val = 0;
double y_val = 0;
double z_val = 0;
double yaw_val = 0;
double linear_margin = 0.5;
double yaw_margin = 0.01;
double x_destination = 5;
double y_destination = 5;
int state_ = ROTATE;
bool flag = false;
bool update = false;

void model_statesCallBack(const gazebo::ModelStates::ConstPtr& msg){
	printf("foo\n");
	update = true;
	unsigned int i=0;
	for (;i<msg->name.size();i++){
		if (strcmp(msg->name[i].c_str(),"pr2")==0)
			break;
	}
	tf::Quaternion q;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(msg->pose[i].orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	yaw_val = yaw;

	x_val = msg->pose[i].position.x;
	y_val = msg->pose[i].position.y;
	z_val = msg->pose[i].position.z;
}

class TeleopPR2Keyboard
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  geometry_msgs::PoseWithCovarianceStamped pose;
  gazebo_msgs::ModelState gazebo_pose;
  
  ros::Publisher gazebo_pose_pub_;
  ros::NodeHandle n_;
  ros::Publisher amcl_pose_pub_;
  ros::Subscriber sub_;
  public:
  void init()
  { 
    gazebo_pose_pub_ = n_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    amcl_pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    sub_ = n_.subscribe("/gazebo/model_states", 1, model_statesCallBack);
    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 0.5);
    n_private.param("run_vel", run_vel, 1.0);
    n_private.param("yaw_rate", yaw_rate, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 1.5);

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
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

	printf("tester\n");

  while(ros::ok())
  {
	ros::spinOnce();
    // get the next event from the keyboard
   if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    switch (c){
      case KEYCODE_G:
        gazebo_pose.model_name = "pr2";
        gazebo_pose.pose.position.x = 10.0;
        gazebo_pose.pose.position.y = 10.0;
        gazebo_pose.pose.position.z = 0.0;
        gazebo_pose.pose.orientation.x = 0.0;
        gazebo_pose.pose.orientation.y = 0.0;
        gazebo_pose.pose.orientation.z = 0.0;
        gazebo_pose.pose.orientation.w = 1.0;
        gazebo_pose.twist.linear.x = 0;
        gazebo_pose.twist.linear.y = 0;
        gazebo_pose.twist.linear.z = 0;
        gazebo_pose.twist.angular.x = 0;
        gazebo_pose.twist.angular.y = 0;
        gazebo_pose.twist.angular.z = 0;
        gazebo_pose.reference_frame = "world";
        gazebo_pose_pub_.publish(gazebo_pose);
        break;
      case KEYCODE_R:
        pose.header.frame_id = "/map";
        pose.pose.pose.position.x = 15;
        pose.pose.pose.position.y = 15;
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0);
        tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
        pose.pose.covariance[6*0+0] = 0.5 * 0.5;
        pose.pose.covariance[6*1+1] = 0.5 * 0.5;
        pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
        amcl_pose_pub_.publish(pose);
        break;
  }
  }
}
