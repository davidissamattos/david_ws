#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float hri_distance = 2.0;
//tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true);


void approachCallback(const std_msgs::Bool &msg)
{
	if(msg.data)//If we can start approaching
	{	
		move_base_msgs::MoveBaseGoal goal;
		//we'll send a goal to the robot to move a distance to the apriltag
		//position.x = distance
		//position.y =0 // We want the robot to approach perpendicular to the tag
		//orientation.w = 0 we want the robot to be facing the tag
		goal.target_pose.header.frame_id = "apriltag";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = hri_distance;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.orientation.w = 0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Getting close to the human");
		}
		else
		{
			ROS_INFO("The base failed to move close");
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "approach_human");
	//Public node	
	ros::NodeHandle nh;
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	//Declaring and setting the subscriber
	ros::Subscriber sub = nh.subscribe("hri_distance/approach", 1, &approachCallback);	

	//Always update parameters before calling the callback function
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		//Inialization
		//Getting parameter name id for the april tag	
		if (nh.hasParam("hri_distance"))
	 	{
	 		// Found parameter, can now query it using param_name
			nh.getParam("hri_distance", hri_distance);
			std::cout<<"hri_distance  "<<hri_distance<<std::endl;
		}
		else
		{
			hri_distance=0;
			ROS_INFO("No parameter 'hri_distance' found. Using hri_distance 2.0");
		}
		
	loop_rate.sleep();
	ros::spinOnce();
	}
}
