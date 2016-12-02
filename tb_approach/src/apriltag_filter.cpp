//THis is a very basic node that filters the content of the identified april tags for a single id number. THis way we can select only the tags we are interested

#include <ros/ros.h>
//ROS Messages
#include "tb_self_experimentation/object_loc.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
using namespace std;

//Simple Global variables
ros::Publisher pub;
//If no parameters are found we use tag id=0
int id_ref = 0;

void filterTag_object_location(const tb_self_experimentation::object_loc &msg)
{
	//Filtering the data
	int id = msg.ID;
	if(id == id_ref)
	{
		//Publishing data
		pub.publish(msg);
		cout<< msg <<endl;
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
	if (nh.hasParam("apriltag/id_ref"))
 	{
 		// Found parameter, can now query it using param_name
		nh.getParam("apriltag/id_ref", id_ref);
		std::cout<<"Using id_ref  "<<id_ref<<std::endl;
	}
	else
	{
		id_ref=0;
		ROS_INFO("No parameter 'id_ref' found. Using id 0 for the april tag");
	}
	//Declaring and setting the subscriber
	ros::Subscriber sub = nh.subscribe("apriltag/object_location", 1, &filterTag_object_location);
	//Setting the publisher
	pub = nh.advertise<tb_self_experimentation::object_loc>("apriltag/distance/", 1);
	ros::spin();
}

