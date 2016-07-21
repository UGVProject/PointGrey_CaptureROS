# ROS_publisher
This is a program for capturing stereo Pointgrey images with Opencv capture and then convert cv::mat into ROS publisher.
Works under catkin (ROS workspace)! So just put it under catkin workspace -> src folder and catkin_make it~~


usage:
roslaunch stereo_publisher capture.launch

default:type:=2 --- with trigger

type:=0 --- without trigger


roslaunch stereo_publisher capture.launch type:=0
