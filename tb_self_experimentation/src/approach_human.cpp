#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float hri_distance = 2.0;
geometry_msgs::Pose2D apriltag_position;
ros::Publisher pub_goal;
float something_x = 10;
float something_y = 10;
MoveBaseClient* ac;



void apriltagPositionCallback(const geometry_msgs::Pose2D &msg)
{
	apriltag_position.x = msg.x;
	apriltag_position.y = msg.y;
	apriltag_position.theta = msg.theta;
	//std::cout<<position<<std::endl;
}

void approachApriltag()
{
	move_base_msgs::MoveBaseGoal goal;
	//setting the header
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	//setting orientation
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,apriltag_position.theta);
	goal.target_pose.pose.orientation.x = qt.x();
	goal.target_pose.pose.orientation.y = qt.y();
	goal.target_pose.pose.orientation.z = qt.z();
	goal.target_pose.pose.orientation.w = qt.w();
	//setting the distance
	goal.target_pose.pose.position.x = apriltag_position.x + hri_distance*cos(-apriltag_position.theta);
	goal.target_pose.pose.position.y = apriltag_position.y + hri_distance*sin(-apriltag_position.theta);

	//Sending the goal and waiting for result	
	ROS_INFO("Sending goal");
	ac->sendGoal(goal);

	ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Goal reached");
	else
		ROS_INFO("The base failed to move reach the goal");
}

void goAway()
{
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
	goal.target_pose.pose.position.x = something_x;
	goal.target_pose.pose.position.y = something_y;

	//Sending the goal and waiting for result	
	ROS_INFO("Sending goal");
	ac->sendGoal(goal);

	ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Goal reached");
	else
		ROS_INFO("The base failed to move reach the goal");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "approach_human");
	//Public node	
	ros::NodeHandle nh;
	//tell the action client that we want to spin a thread by default
	ac = new MoveBaseClient("move_base", true);
	
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	ros::Subscriber sub_pos = nh.subscribe("apriltag/global_position", 1, &apriltagPositionCallback);
	
	
	//Always update parameters before calling the callback function
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		//Getting parameters for experimenting
		//Updating these parameters every loop
		if (nh.hasParam("hri_distance"))
	 	{
			nh.getParam("hri_distance", hri_distance);
		}	
		if (nh.hasParam("something_x"))
	 	{
			nh.getParam("something_x", something_x);
		}
		if (nh.hasParam("something_y"))
	 	{
			nh.getParam("something_y", something_y);
		}
		
		
	loop_rate.sleep();
	ros::spinOnce();
	}
	return 0;




}























/*

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseActionResult>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

float hri_distance = 2.0;
geometry_msgs::Pose2D position;
ros::Publisher pub_goal;
float something_x = 10;
float something_y = 10;

void approachCallback()
{
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "/map";
	goal.header.stamp = ros::Time::now();
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,position.theta);
	goal.pose.orientation.x = qt.x();
	goal.pose.orientation.y = qt.y();
	goal.pose.orientation.z = qt.z();
	goal.pose.orientation.w = qt.w();
	//setting the distance
	goal.pose.position.x = position.x + hri_distance*cos(-position.theta);
	goal.pose.position.y = position.y + hri_distance*sin(-position.theta);
	//std::cout<<"hri_distance  "<<hri_distance<<std::endl;
	std::cout<<goal<<std::endl;
	//Publishing goal to move_base_simple	
	pub_goal.publish(goal);


void somethingCallback()
{
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "/map";
	goal.header.stamp = ros::Time::now();
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,1);
	goal.pose.orientation.x = qt.x();
	goal.pose.orientation.y = qt.y();
	goal.pose.orientation.z = qt.z();
	goal.pose.orientation.w = qt.w();
	//setting the distance
	goal.pose.position.x = something_x;
	goal.pose.position.y = something_y;
	//Publishing goal to move_base_simple	
	pub_goal.publish(goal);

}

//Saving the current position of the apriltag
void apriltagPositionCallback(const geometry_msgs::Pose2D &msg)
{
	position.x = msg.x;
	position.y = msg.y;
	position.theta = msg.theta;
	std::cout<<position<<std::endl;
}


void goalStatusCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
	//Se ja chegouy perto e self-exp
	//chama o something
	//Se ja foi o something
	//self-experiment
	int status = msg.status.status
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "approach_human");
	//Public node	
	ros::NodeHandle nh;
	//Declaring and setting the subscriber
	ros::Subscriber sub = nh.subscribe("move_base/result", 1, &goalStatusCallback);
	
	ros::Subscriber sub_pos = nh.subscribe("apriltag/global_position", 1, &apriltagPositionCallback);
	//Declaring the publisher
	pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal/", 1);
	//Always update parameters before calling the callback function
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		//Getting parameters for experimenting
		//Updating these parameters every loop
		if (nh.hasParam("hri_distance"))
	 	{
			nh.getParam("hri_distance", hri_distance);
		}	
		if (nh.hasParam("something_x"))
	 	{
			nh.getParam("something_x", something_x);
		}
		if (nh.hasParam("something_y"))
	 	{
			nh.getParam("something_y", something_y);
		}
		
		
	loop_rate.sleep();
	ros::spinOnce();
	}
	return 0;
}
*/
