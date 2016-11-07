#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

float hri_distance = 2.0;
geometry_msgs::Pose2D position;
ros::Publisher pub_goal;
bool going_to_goal = false;

void approachCallback(const std_msgs::Bool &msg)
{
	//This flags are just to make sure we publish the goal once
	//very simplistic case
	
	//Define the goal
	if(msg.data==true && going_to_goal==false)//If we can start approaching
	{	
		//Sending the tb to the distance goal
		going_to_goal = true;
		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "/map";
		goal.header.stamp = ros::Time::now();
		
		tf::Quaternion qt = tf::Quaternion();
		qt.setRPY(0,0,position.theta);
		goal.pose.orientation.x = qt.x();
		goal.pose.orientation.y = qt.y();
		goal.pose.orientation.z = qt.z();
		goal.pose.orientation.w = qt.w();

		goal.pose.position.x = position.x + hri_distance*cos(-position.theta);
		goal.pose.position.y = position.y + hri_distance*sin(-position.theta);
		std::cout<<"hri_distance  "<<hri_distance<<std::endl;
		std::cout<<goal<<std::endl;
		
		pub_goal.publish(goal);
		
	}
	if(msg.data==false && going_to_goal==true)//waiting to finish the goal
	{
		going_to_goal = false;
	}
}

//Saving the current position of the apriltag
void apriltagPositionCallback(const geometry_msgs::Pose2D &msg)
{
	position.x = msg.x;
	position.y = msg.y;
	position.theta = msg.theta;
	std::cout<<position<<std::endl;
}


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "approach_human");
	//Public node	
	ros::NodeHandle nh;

	//Declaring and setting the subscriber
	ros::Subscriber sub = nh.subscribe("hri_distance/approach/start", 1, &approachCallback);
	
	ros::Subscriber sub_pos = nh.subscribe("apriltag/global_position", 1, &apriltagPositionCallback);
	//Declaring the publisher
	pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal/", 1);

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
			//std::cout<<"hri_distance  "<<hri_distance<<std::endl;
		}
		else
		{
			hri_distance=0;
			ROS_INFO("No parameter 'hri_distance' found. Using hri_distance 2.0");
		}
		
	loop_rate.sleep();
	ros::spinOnce();
	}
	return 0;
}
