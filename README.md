# ROS Workspace for self-experimenting in robots
This repository contains some ROS packages downloaded from the internet and created by myself to allow self-experimenting in robotics.

I am testing only with ROS Indigo using Ubuntu 14.04 LTS on Parallels 11 Mac and on Ubuntu 14.04 LTS in a full linux Lenovo computer.
Some issues with this setup:
* The gazebo simulator for the virtual machine does not work properly
* The lenovo computer internal microphone is really bad

Some considerations
* I am self-experimenting in Human-Robot Interaction in physical proxemic distance.
* The robotic platform that I am using is the Turtlebot 2
* To allow this self-experimenting I am rellying on several packages download over the internet and some of those packages are not uploaded here because the robot is using another workspace for other project. I will try to list everything here

Dependencies
* Turtlebot complete installation
* Pocketsphinx package
    * http://jokla.me/robotics/speech-recognition-ros/
    * http://wiki.ros.org/pocketsphinx
    * http://www.speech.cs.cmu.edu/tools/lmtool.html
    * Remember to run sudo apt-get install gstreamer0.10-gconf if there are problems in launching the recognizer
* I changed the dependencies so I don't need the following packages anymore. But I will list them here anyway because they are the references for part of what I have modified
    * WASP Summer school packages and autonomous system course repo
    	* https://github.com/WASP2016
	* https://github.com/pierg/wasp_cht2_catkin
    	* There might be some conflicts between pierg repository and the WASP Summer School repository
    * April tag package
	* This package is contained inside the WASP Summer School and pierg repo. In pierg package there are some modifications in the source files for the april tag recognition to include some information of the april tags in the image (what are the pixels, center, image size...)
	* I modified this package extensively

Description of the packages
* Pocketsphinx
    * 
* tb_simulation
    * launch
* tb_self_experimentation
    * launch
	*
    * misc
	*
    * msg
	*
    * pynodes
	*
    * src
	*

