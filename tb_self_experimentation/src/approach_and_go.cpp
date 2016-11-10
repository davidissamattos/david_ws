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
#include <boost/thread/thread.hpp> //ros does not support c++11
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float hri_distance = 1.0;
geometry_msgs::Pose2D apriltag_position;
ros::Publisher pub;
float something_x = 10;
float something_y = 10;
MoveBaseClient* ac;

void getInput();
void approachApriltag();
void goAway();


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

void apriltagPositionCallback(const geometry_msgs::Pose2D &msg)
{
	apriltag_position.x = msg.x;
	apriltag_position.y = msg.y;
	apriltag_position.theta = msg.theta;
	//cout<<"x: "<<apriltag_position.x<<" y: "<<apriltag_position.y<<"Theta: "<<apriltag_position.theta<<endl;
}

void approachApriltag()
{
	ROS_INFO("Approaching tag");
	move_base_msgs::MoveBaseGoal goal;
	//Setting the header
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	//setting the distance
	goal.target_pose.pose.position.x = apriltag_position.x + hri_distance*cos(apriltag_position.theta);
	goal.target_pose.pose.position.y = apriltag_position.y -hri_distance*sin(apriltag_position.theta);

		//setting orientation
	double angle = apriltag_position.theta;
	tf::Quaternion qt = tf::Quaternion();
	qt.setRPY(0,0,angle);
	goal.target_pose.pose.orientation.x = qt.x();
	goal.target_pose.pose.orientation.y = qt.y();
	goal.target_pose.pose.orientation.z = qt.z();
	goal.target_pose.pose.orientation.w = qt.w();

	//Sending the goal and waiting for result	
	ROS_INFO("Apriltag position");
	cout<<"x: "<<apriltag_position.x<<" y: "<<apriltag_position.y<<" Theta: "<<apriltag_position.theta<<endl;
	//ROS_INFO("Distance x sin and cos");
	//cout<<"sin: "<<hri_distance*sin(-apriltag_position.theta)<<" cos: "<<hri_distance*cos(-apriltag_position.theta)<<endl;
	ROS_INFO("Goal position");
	cout<<"x: "<<goal.target_pose.pose.position.x<<" y: "<<goal.target_pose.pose.position.y<<endl;
	
	ROS_INFO("Sending goal");
	ac->sendGoal(goal);
	ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Goal reached");
		std_msgs::Bool signal;
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
	goal.target_pose.pose.position.x = something_x;
	goal.target_pose.pose.position.y = something_y;

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
	ac = new MoveBaseClient("move_base", true);
	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
        ROS_INFO("See if the tb base is on");
	}
	
	ros::Subscriber sub_pos = nh.subscribe("apriltag/global_position", 1, &apriltagPositionCallback);
	pub = nh.advertise<std_msgs::Bool>("hri_distance/robot/concluded_approach", 1); 
	//Input thread
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
	t1.join();
	return 0;
}

