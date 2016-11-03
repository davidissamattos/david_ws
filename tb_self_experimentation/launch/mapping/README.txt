1 - Configure the turtlebot ip and the host ip in the .bashrc files
2- Launch the mapping_tb using ssh in the turtlebot computer
3- Launch the mapping_basestation in the basestation computer
4- Run the turtlebot around to map the environment
5- After mapping, save the file using the command
	rosrun map_server map_saver -f ~/david_ws/src/tb_self_experimentation/misc/maps/map_name

Some references for this
http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM

