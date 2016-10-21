#include <ros/ros.h>

//ROS Messages
#include "wasp_custom_msgs/object_loc.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
using namespace std;

//Simple Global variables
ros::Publisher pub_x;
ros::Publisher pub_y;
ros::Publisher pub_z;

int id_ref = 0;

void filterTag_object_location(const wasp_custom_msgs::object_loc &msg)
{

	int id = msg.ID;
	if(id == id_ref)
	{
		
		//distance
		std_msgs::Float64 dist_x;
		std_msgs::Float64 dist_y;
		std_msgs::Float64 dist_z;
		
		//Retrieving info from message
		dist_x.data = msg.point.x;
		dist_y.data = msg.point.y;
		dist_z.data = msg.point.z;

		//Publishing data
		pub_x.publish(dist_x);
		pub_y.publish(dist_y);
        	pub_z.publish(dist_z);

	}
}


int main(int argc, char **argv) 
{
	// Initialize the ROS system.
	ros::init(argc, argv, "filter_tag");
	// Establish this program as a ROS node. 
	//Public node	
	ros::NodeHandle nh;
	
	//Getting parameter name id for the april tag	
	if (nh.hasParam("id_ref"))
 	{
 		// Found parameter, can now query it using param_name
		nh.getParam("id_ref", id_ref);
		std::cout<<"Using id_ref  "<<id_ref<<std::endl;
	}
	else
	{
		id_ref=0;
		ROS_INFO("No parameter 'id_ref' found. Using id 0 for the april tag");
	}


	//Declaring and setting the subscriber
	ros::Subscriber sub = nh.subscribe("object_location", 1, &filterTag_object_location);
	//Setting the publisher
	pub_x = nh.advertise<std_msgs::Float64>("distance_x/", 1);
	pub_y = nh.advertise<std_msgs::Float64>("distance_y/", 1);
	pub_z = nh.advertise<std_msgs::Float64>("distance_z/", 1);

	
	ros::spin();
}

