# fake_laser_catkin
use ros catkin to build a project to pulish fake laser data and tranfrom it for rviz and mapping
##source code
source code locates in the src/beginner_tutorials  
it is a simple publisher of ros to generate fake laser data  
##how to use
###build catkin workspace
[Create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment/)
###command line
cd ~/catkin_ws  

catkin_make  

source ./devel/setup.bash   

roscore  

rosrun beginner_tutorials talker  

rosrun gmapping slam_gmapping scan:=laser  

rosrun rviz rviz
