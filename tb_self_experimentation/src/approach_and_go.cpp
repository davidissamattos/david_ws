#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream> 
#include <string>
#include <boost/thread/thread.hpp> //ros does not support c++11!!!
#include "tb_self_experimentation/robot_feedback_service.h"
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Some global ros variables
geometry_msgs::Pose2D apriltag_position;
ros::Publisher pub;
ros::ServiceClient client;

// Parameters
float hri_distance = 1.0;
float feedbackDelay = 3;
float distance_x = 10;
float distance_y = 10;

//Action client for sending goals
MoveBaseClient* ac;

//Function headers
void getInput();
void approachApriltag();
void goAway();

//This function is a simple keyboard input
//If we press 'a' we approach the tag -> function approachAptriltag
//If we press 's' we go to the point (distance_x, distance_y) -> function goAway
//This function is suppose to run on a separate thread
void getInput()
{
	while(ros::ok)
	{
		ros::Rate loop_rate(1);
		std::cout<<"Press 'a' to approach and 's' to go away"<<endl;
		std::string result;    		
		getline(cin,result);
            	cout<<"You entered '"<<result<<"'' "<< endl;
		if(result=="a")
			approachApriltag();
		if(result=="s")
			goAway();
		loop_rate.sleep();
		ros::spinOnce();
	}
    
}

//This function just collect the current apriltag position and save it to the global variable apriltag_position
void apriltagPositionCallback(const geometry_msgs::Pose2D &msg)
{
	apriltag_position.x = msg.x;
	apriltag_position.y = msg.y;
	apriltag_position.theta = msg.theta;
}

//This function calls the action to approach the tag
//We send our goal on the /map frame
//Our position is calculated so we approach the tag from the front with a distance of hri_position
//The action is the MoveBaseClient. It is declared globally as the pointer ac
void approachApriltag()
{
	//Changing the face to flat
	tb_self_experimentation::robot_feedback_service srv;
	srv.request.face_type = "flat";
	if (client.call(srv))
	{
		ROS_INFO("Flat face");
	}	

	ROS_INFO("Approaching tag");
	move_base_msgs::MoveBaseGoal goal;
	//Setting the header
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	//setting the distance
	goal.target_pose.pose.position.x = apriltag_position.x + hri_distance*cos(M_PI + apriltag_position.theta);
	goal.target_pose.pose.position.y = apriltag_position.y + hri_distance*sin(M_PI+apriltag_position.theta);
	//setting orientation
	double angle = apriltag_position.theta;
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,angle);
	goal.target_pose.pose.orientation.x = qt.x();
	goal.target_pose.pose.orientation.y = qt.y();
	goal.target_pose.pose.orientation.z = qt.z();
	goal.target_pose.pose.orientation.w = qt.w();

	//Debug messages.	
	ROS_INFO("Apriltag position");
	cout<<"x: "<<apriltag_position.x<<" y: "<<apriltag_position.y<<" Theta: "<<apriltag_position.theta<<endl;
	ROS_INFO("Goal position");
	cout<<"x: "<<goal.target_pose.pose.position.x<<" y: "<<goal.target_pose.pose.position.y<<endl;
	
	//Sending the goal and waiting for the result of the goal
	ROS_INFO("Sending goal");
	ac->sendGoal(goal);
	ac->waitForResult();

	//If we actually got the the goal we publish true to the topic hri_distance/conclude_approach
	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Goal reached");
		std_msgs::Bool signal;
		//The base will delay a few seconds before doing anything else
		//This will give us some time to give the feedback
		ros::Duration(feedbackDelay).sleep();
		signal.data = true;
		pub.publish(signal);

		
	}	
	else
	{	
		ROS_INFO("The base failed to move reach the goal");
		std_msgs::Bool signal;
		signal.data = false;
		pub.publish(signal);
	}
}


//This function is very similar to the approach tag. But instead of going to the tag we go to a predefined point distance_x, distance_y)
//However this function always publish false to the hri_distance/conclude_approach topic
void goAway()
{
	ROS_INFO("Going away");
	move_base_msgs::MoveBaseGoal goal;
	//setting the header
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	//setting orientation
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,0);
	goal.target_pose.pose.orientation.x = qt.x();
	goal.target_pose.pose.orientation.y = qt.y();
	goal.target_pose.pose.orientation.z = qt.z();
	goal.target_pose.pose.orientation.w = qt.w();
	//setting the distance
	goal.target_pose.pose.position.x = distance_x;
	goal.target_pose.pose.position.y = distance_y;

	//Sending the goal and waiting for result	
	ROS_INFO("Sending goal");
	cout<<"x: "<<goal.target_pose.pose.position.x<<" y: "<<goal.target_pose.pose.position.y<<endl;
	ac->sendGoal(goal);
	ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Goal reached");
		std_msgs::Bool signal;
		signal.data = false;
		pub.publish(signal);
	}	
	else
	{
		ROS_INFO("The base failed to move reach the goal");
		std_msgs::Bool signal;
		signal.data = false;
		pub.publish(signal);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "approach_and_go");	
	ros::NodeHandle nh;
	//tell the action client that we want to spin a thread by default
	// ac is a global pointer
	ac = new MoveBaseClient("move_base", true);
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
        	ROS_INFO("See if the turtlebot base is on");
	}
	
	ros::Subscriber sub_pos = nh.subscribe("apriltag/global_position", 1, &apriltagPositionCallback);
	pub = nh.advertise<std_msgs::Bool>("hri_distance/conclude_approach", 1); 
	//service for changing the robot face	
	ros::ServiceClient client = nh.serviceClient<tb_self_experimentation::robot_feedback_service>("hri_distance/robot/face_feedback");

	//Input thread -> the keyboard input does not hold the ROS node
	boost::thread t1(getInput);
	//Always update parameters before calling the callback function
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		//Getting parameters for experimenting
		//Updating these parameters every loop
		if (nh.hasParam("hri_distance"))
	 	{
			nh.getParam("hri_distance", hri_distance);
		}	
		if (nh.hasParam("distance_x"))
	 	{
			nh.getParam("distance_x", distance_x);
		}
		if (nh.hasParam("distance_y"))
	 	{
			nh.getParam("distance_y", distance_y);
		}
		if (nh.hasParam("feedbackDelay"))
	 	{
			nh.getParam("feedbackDelay", feedbackDelay);
		}
	loop_rate.sleep();
	ros::spinOnce();
	}
	t1.join();
	return 0;
}

