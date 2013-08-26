#include <ros/ros.h>
#include "ros/console.h"
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
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
#include <tf/transform_listener.h>
#include <SDL/SDL_image.h>
#include <robot_spatial_nlp/JPArray.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <iostream>
#include <cmath>

#define RAD(x) ((x)*(M_PI/180.0))

using namespace std;

// List of Robot States
namespace RobotState {
  enum Item {NONE, NAVIGATION, APPROACH, ROTATE, BACK_AWAY, FINISHED};
};
typedef RobotState::Item RobotState_t; 

RobotState_t state = RobotState::NONE;
RobotState_t trans_state = RobotState::NONE;

// Robot positions
double x_map, y_map, yaw_val;
double approach_x, approach_y, approach_yaw;
double back_x, back_y, back_yaw;
double rotate_yaw;
double approach_distance;
double x_start, y_start;

// ROS message handling
nav_msgs::GetMap::Response map_resp_;
ros::Publisher map_pub_;
ros::Publisher vel_pub_;
ros::Publisher end_pub_;
move_base_msgs::MoveBaseGoal goal;
geometry_msgs::Twist cmd;

// Map constants
double res = 0.05;
int image_height = 300;

// Publish SLAM map
void sendMap();

// Movebase action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* ac;

// Compute angle difference between desired and current yaw
double anglediff(double yaw_desired, double yaw){
	double ang1,ang2,diff;
	ang1 = yaw_desired;
	ang2 = yaw;
	diff = ang1 - ang2;
	if(diff > M_PI)
		diff -= 2*M_PI;
	else if(diff < -M_PI)
		diff += 2*M_PI;
		
	return diff;
}

// Compute angle difference between vector and robot yaw
double anglediff(double vx, double vy, double yaw, double* yaw_desired){
	// Calculate desired yaw
	*yaw_desired = atan2(vy,vx);
	
	// Return angle difference
	return anglediff(*yaw_desired, yaw);
}

// Distance between two points
double distance(double x, double y, double x2, double y2)
{
	return sqrt(pow((x-x2),2) + pow((y-y2),2));
}

// Return sign of value
double sign(double value){
	return ((value < 0.0) ? -1.0 : 1.0);
}

// Send finished message
void sendFinishedMessage(){
	// Publish goal reached message to simgl_ros
	printf("Finished goal request!*****\n");
	std_msgs::String str;
	std::stringstream ss;
	ss << "goal_reached";
	str.data = ss.str();
	end_pub_.publish(std_msgs::String(str));
}

// Navigation stack goal completed
void doneCb(const actionlib::SimpleClientGoalState& astate, const move_base_msgs::MoveBaseResultConstPtr& result)
{
	// Nav stack goal finished
	ROS_INFO("Nav action client finished in state [%s]", astate.toString().c_str());
	state = RobotState::FINISHED;
}

// Simgl_ros costmap message
void mapCallBack(const robot_spatial_nlp::JPArray::ConstPtr& msg){
	int index = 0;

	// Receive new costmap from simgl_ros
	for(unsigned int y=0; y<map_resp_.map.info.height; y++){
		for(unsigned int x=0; x<map_resp_.map.info.width; x++){
			map_resp_.map.data[index] = msg->data[index];
			index++;					
		}
	}
	map_resp_.map.info.map_load_time = ros::Time::now();
	map_resp_.map.header.stamp = ros::Time::now();
	ROS_INFO("Costmap sent! (%d X %d) map @ %.3lf m/cell",
	map_resp_.map.info.width,
	map_resp_.map.info.height,
	map_resp_.map.info.resolution);

	// Publish new costmap to move_base
	map_pub_.publish( map_resp_.map );
}

// Simgl_ros robot goal position message
void goalCallBack(const geometry_msgs::Point::ConstPtr& msg){
	
	// Navigation stack goal received
	if(msg->z < -5.0){
		// Change to navigation state
		printf("Navigating to goal...\n");
		state = RobotState::NAVIGATION;
		
		// Send goal to nav stack
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = msg->x*res;
		goal.target_pose.pose.position.y = (image_height-msg->y) * res;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw((msg->z + 10.0));
		ac->sendGoal(goal, boost::bind(doneCb, _1, _2), MoveBaseClient::SimpleActiveCallback(), MoveBaseClient::SimpleFeedbackCallback());
	}
	// Approach goal received
	else if(msg->z < 5.0){
		// Change to approach state
		printf("Approaching goal...\n");
		state = RobotState::APPROACH;
		
		// Set goal position
		approach_x = msg->x*res;
		approach_y = (image_height-msg->y) * res;
		approach_yaw = msg->z;
		
		// Set start position
		x_start = x_map;
		y_start = y_map;
		
		// Set approach distance
		approach_distance = distance(approach_x, approach_y, x_start, y_start);
	}
	// Back away goal received
	else{
		// Change to back away state
		printf("Backing away to goal...\n");
		state = RobotState::BACK_AWAY;
		
		// Set goal position
		back_x = msg->x*res;
		back_y = (image_height-msg->y) * res;
		back_yaw = (msg->z - 10.0);
	}
}

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "constant_goal_publisher");
	ros::NodeHandle n;
	
	// ROS subscribers
	ros::Subscriber goal_sub_ = n.subscribe("sim_goal", 1, goalCallBack);
	ros::Subscriber map_sub_  = n.subscribe("sim_costmap", 1, mapCallBack);
	
	// ROS publishers
	map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("mymap", 1, true);
	vel_pub_ = n.advertise<geometry_msgs::Twist>("base_controller/command", 1);
	end_pub_ = n.advertise<std_msgs::String>("goal_request", 1, true);
	
	// Start move_base action client
	MoveBaseClient ac2("move_base", true);
	ac = &ac2;
	
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
  
  	// Send initial map
  	sendMap();
  	
  	// Initialize transform listener
    tf::TransformListener listener_;
    listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
  
  	// Loop variables
  	double ang_diff = 0.0, fdiff;
  	double vx, vy, dist, dist2, yaw_desired, dist_traveled;
  	
  	// Constants
  	double ang_thresh = RAD(30.0);      // Radians
  	double goal_ang_thresh = RAD(10.0); // Radians
  	double goal_dist_thresh = 0.2;      // Meters
  	double back_dist_thresh = 0.4;      // Meters
  	double min_vx = 0.1;  				// Meters/second
  	double max_vx = 0.5;  				// Meters/second
  	double min_va = RAD(5.0);           // Radians/second
  	double min_rotate = RAD(10.0);      // Radians/second
  	double max_rotate = RAD(60.0);      // Radians/second
  	
  	// Main loop
	while(ros::ok()){
		ros::spinOnce();
		
		// Get current position of robot in map	
		tf::StampedTransform transform;
		try{
		  listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
		}
    catch (tf::TransformException ex){
      ROS_ERROR("Tf has an error to look up transform: %s",ex.what());
      continue;
    }
		x_map = transform.getOrigin().x();
		y_map = transform.getOrigin().y();
		yaw_val = tf::getYaw(transform.getRotation());
	  
	  	// State machine loop
	  	bool done = false;
	  	while(!done){
			switch(state){
			case RobotState::NAVIGATION:
				// Wait for action client done callback..
				// TODO: track robot position and send new goal/overtake final positioning
				break;
			
			case RobotState::APPROACH:
				// Calculate distance to goal
				dist = distance(approach_x, approach_y, x_map, y_map);
				dist_traveled = distance(x_map, y_map, x_start, y_start);
			
				// Check if goal reached
				if(/*dist < goal_dist_thresh ||*/ dist_traveled > approach_distance){
					// Approach success!
					//printf("APPROACH: Reached goal, now facing target... (dist = %g, yaw = %g, ryaw = %g)\n",dist, yaw_val, approach_yaw);
					state = RobotState::ROTATE;
					rotate_yaw = approach_yaw;
					ang_diff = anglediff(rotate_yaw,yaw_val);
					printf("APPROACH: Reached goal, now facing target... (dist = %g, yaw = %g, ryaw = %g, diff = %g)\n",dist, yaw_val, rotate_yaw, ang_diff);
					trans_state = RobotState::FINISHED;
					continue;
				}
			
				// Compute angle difference to goal point
				vx = approach_x - x_map;
				vy = approach_y - y_map;
				ang_diff = anglediff(vx,vy,yaw_val,&yaw_desired);
			
				// Angle difference is large
				if((dist > goal_dist_thresh) && (fabs(ang_diff) > ang_thresh)){
					// Transition to rotate state
					printf("APPROACH: Rotating to point... (diff = %g)\n",ang_diff);
					state = RobotState::ROTATE;
					rotate_yaw = yaw_desired;
					trans_state = RobotState::APPROACH;
					continue;
				}
				
				// Move to point directly with min/max speed
				if(vx < min_vx){
					vx = min_vx;
				}
				else if(vx > max_vx){
					vx = max_vx;
				}
				if(fabs(ang_diff) < min_va){
					ang_diff = 0.0;
				}
				
				// Set command velocities
				cmd.linear.x = vx; 
				cmd.linear.y = 0.0;
				cmd.angular.z = 0.0;//ang_diff;
			
				// Publish velocities
				vel_pub_.publish(cmd);
				break;
			
			case RobotState::ROTATE:
				// Compute angle difference to yaw desired
				ang_diff = anglediff(rotate_yaw,yaw_val);
				fdiff = fabs(ang_diff);
				
				// Check if desired yaw reached
				if(fdiff < goal_ang_thresh){
					// Transition to next state
					printf("ROTATE: Done rotating... (diff = %g)\n",ang_diff);
					state = trans_state;
					continue;
				}
				
				// Rotate to desired yaw with min/max speed
				if(fdiff < min_rotate){
					ang_diff = sign(ang_diff)*min_rotate;
				}
				else if(fdiff > max_rotate){
					ang_diff = sign(ang_diff)*max_rotate;
				}
				
				// Set command velocities
				cmd.linear.x = 0.0; 
				cmd.linear.y = 0.0;
				cmd.angular.z = ang_diff;
				
				printf("ROTATE: (yaw = %g, ryaw = %g, diff = %g)\n", yaw_val, rotate_yaw, ang_diff);
			
				// Publish velocities
				vel_pub_.publish(cmd);
				break;
			
			case RobotState::BACK_AWAY:
				// Calculate distance to goal
				// TODO: Change back pt and robot pt to odom frame
				dist2 = distance(approach_x, approach_y, x_map, y_map);
				dist = distance(back_x, back_y, x_map, y_map);
				
				// Check if back up distance reached
				if(dist2 > back_dist_thresh){
					// Back away success!
					printf("BACK AWAY: Reached goal (dist = %g)\n",dist2);
					state = RobotState::FINISHED;
					continue;
				}
				
				// Back up directly with min/max speed
				if(dist < min_vx){
					dist = min_vx;
				}
				else if(dist > max_vx){
					dist = max_vx;
				}
				
				// Set command velocities
				cmd.linear.x = -dist; 
				cmd.linear.y = 0.0;
				cmd.angular.z = 0.0;
			
				// Publish velocities
				vel_pub_.publish(cmd);
				break;
			
			case RobotState::FINISHED:
				// Send finished message
				sendFinishedMessage();
				state = RobotState::NONE;
				break;
			
			default:
				break;
			};
			
			// Done processing state machine
			done = true;
		}
  	}
  	
	return 0;
}

// Publish SLAM map
void sendMap(){
	std::string mapfname;
	double origin[3];
	int negate;
	double occ_th, free_th;

	mapfname = "/home/jfasola/fuerte_workspace/sandbox/jpmaster77/robot_spatial_nlp/pics/map_2_slam.pgm";
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

