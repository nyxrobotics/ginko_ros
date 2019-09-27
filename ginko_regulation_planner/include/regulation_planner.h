#ifndef REGULATION_PLANNER_H
#define REGULATION_PLANNER_H

#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <iostream>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <boost/bind.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

class RegulationPlanner {
private:
	//動作:
	//ros::NodeHandle node_handle_;
	ros::Subscriber regulation_command_sub_;
	int regulation_command_ready_ = 0;
	int regulation_command_update_ = 0;
	ros::Publisher motion_command_pub_;
	std_msgs::String motion_command_;

	std_msgs::String regulation_command_;
	std::string regulation_state_;

	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;

	std::string robot_tf_ = "body_imu_base_link";	//ロボットのTF
	std::string human_r_tf_ = "human_r";
	std::string human_l_tf_ = "human_l";
	std::string robot_standing_r_tf_ = "robot_standing_r";
	std::string robot_standing_l_tf_ = "robot_standing_l";
	std::string robot_laying_r_tf_ = "robot_laying_r";
	std::string robot_laying_l_tf_ = "robot_laying_l";

public:
	RegulationPlanner(ros::NodeHandle main_nh);
	~RegulationPlanner();
	void readParams(ros::NodeHandle node_handle_);
	int mainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);
	void initTF2();
	bool checkTfAlive(std::string tf_name);
	void getRegulationCommandCallback(const std_msgs::String::ConstPtr& msg);
	int regulationMotionSelect();
};














#endif //REGULATION_PLANNER_H

