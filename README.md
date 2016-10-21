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
* April tag package
* Pocketsphinx package
    * http://jokla.me/robotics/speech-recognition-ros/
    * http://wiki.ros.org/pocketsphinx
    * http://www.speech.cs.cmu.edu/tools/lmtool.html
* WASP message package and maybe some other stuff developed in the WASP Autonomous system course
    * https://github.com/pierg/wasp_cht2_catkin
* WASP Summer school packages (not totally sure if I am really depending on something from there)
    * https://github.com/WASP2016
    * There might be some conflicts between pierg repository and the WASP Summer School repository
