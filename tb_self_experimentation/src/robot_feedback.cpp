//THis is a very basic node that filters the content of the identified april tags for a single id number. THis way we can select only the tags we are interested

#include <ros/ros.h>
//ROS Messages
#include "tb_self_experimentation/object_loc.h"
#include "tb_self_experimentation/robot_feedback_service.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
std::string happy_image = "/home/chalmers/david_ws/src/tb_self_experimentation/misc/images/happy.png";
std::string sad_image = "/home/chalmers/david_ws/src/tb_self_experimentation/misc/images/sad.png";
std::string flat_image = "/home/chalmers/david_ws/src/tb_self_experimentation/misc/images/flat.png";

bool robotFeedback(tb_self_experimentation::robot_feedback_service::Request  &req, tb_self_experimentation::robot_feedback_service::Response &res)
{
	if(req.face_type == "happy")//Positive feedback
	{
		Mat image;
		image = imread(happy_image, CV_LOAD_IMAGE_COLOR);
		if(!image.data )                              // Check for invalid input
    		{
        		cout <<  "Could not open or find the image" << std::endl ;
    		}
		else
		{
			imshow( "robot", image );                   // Show our image inside it.
		}
	}
	if(req.face_type == "sad")//negative feedback
	{
		cv::Mat image;
		image = imread(sad_image, CV_LOAD_IMAGE_COLOR);
		if(!image.data )                              // Check for invalid input
    		{
        		cout <<  "Could not open or find the image" << std::endl ;
    		}
		else
		{
			imshow( "robot", image );                   // Show our image inside it.
		}
	}
	if(req.face_type == "flat")//negative feedback
	{
		cv::Mat image;
		image = imread(flat_image, CV_LOAD_IMAGE_COLOR);
		if(!image.data )                              // Check for invalid input
    		{
        		cout <<  "Could not open or find the image" << std::endl ;
    		}
		else
		{
			imshow( "robot", image );                   // Show our image inside it.
		}
	}
	res.response = true;
	return true;
}


int main(int argc, char **argv) 
{
	// Initialize the ROS system.
	cout <<  "Initializing" << std::endl ;
	ros::init(argc, argv, "robot_feedback");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("hri_distance/robot/face_feedback", robotFeedback);
	
	//Getting image parameters
	if (nh.hasParam("happy_image"))
 	{
 		// Found parameter, can now query it using param_name
		nh.getParam("happy_image", happy_image);
	}
	if (nh.hasParam("sad_image"))
 	{
 		// Found parameter, can now query it using param_name
		nh.getParam("sad_image", sad_image);
	}
	if (nh.hasParam("flat_image"))
 	{
 		// Found parameter, can now query it using param_name
		nh.getParam("flat_image", flat_image);
	}
	
	//show initial image
	cv::startWindowThread();
	namedWindow( "robot", WINDOW_AUTOSIZE );// Create a window for display.
	Mat image;
	image = imread(flat_image, CV_LOAD_IMAGE_COLOR);
	if(! image.data )                              // Check for invalid input
	{
		cout <<  "Could not open or find the image" << std::endl ;
	}
	else
	{
		cout <<  "Image was read with success" << std::endl ;
		imshow( "robot", image);                   // Show our image inside it.
	}
	ros::spin();
}

