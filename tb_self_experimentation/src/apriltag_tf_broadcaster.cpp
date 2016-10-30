//This node is responsible for converting the apriltag location in respect to the turtlebot.
// This way we can set up a listener that looks at the apriltag tf and put it into the world coordinate system

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "wasp_custom_msgs/object_loc.h"

//Some global transformation variables
tf::TransformBroadcaster br;
tf::Transform transform;

//This callback function is responsible for getting the camera-apriltag distance and add a mpde tp tje 
void transform_callback(const wasp_custom_msgs::object_loc &msg)
{
	transform.setOrigin( tf::Vector3(msg.point.x, msg.point.y, msg.point.z) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_frame", "apriltag"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "apriltag_tf_broadcaster");
	ros::NodeHandle nh;
	ros::Subscriber sub =  nh.subscribe("apriltag/distance", 1, &transform_callback);
	ros::spin();
};
