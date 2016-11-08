//THis is a very basic node that filters the content of the identified april tags for a single id number. THis way we can select only the tags we are interested

#include <ros/ros.h>
//ROS Messages
#include "tb_self_experimentation/object_loc.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
std::string happy_image = "/home/davidis/david_ws/happy.jpg";
std::string sad_image = "/home/davidis/david_ws/sad.jpg";


void robotFeedback(const std_msgs::Bool &msg)
{
	if(msg.data)//Positive feedback
	{
		Mat image;
		image = imread(happy_image, CV_LOAD_IMAGE_COLOR);
		if(! image.data )                              // Check for invalid input
    		{
        		cout <<  "Could not open or find the image" << std::endl ;
    		}
		else
		{
			imshow( "robot", image );                   // Show our image inside it.
		}
	}
	else//negative feedback
	{
		cv::Mat image;
		image = imread(sad_image, CV_LOAD_IMAGE_COLOR);
		if(! image.data )                              // Check for invalid input
    		{
        		cout <<  "Could not open or find the image" << std::endl ;
    		}
		else
		{
			imshow( "robot", image );                   // Show our image inside it.
		}
	}
}


int main(int argc, char **argv) 
{
	// Initialize the ROS system.
	cout <<  "Initializing" << std::endl ;
	ros::init(argc, argv, "robot_feedback");
	ros::NodeHandle nh;
	ros::Subscriber sub_positive = nh.subscribe("hri_distance/robot/feedback", 1, &robotFeedback);
	
	//show image
	cv::startWindowThread();
	namedWindow( "robot", WINDOW_AUTOSIZE );// Create a window for display.
	Mat image;
	image = imread(happy_image, CV_LOAD_IMAGE_COLOR);
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

