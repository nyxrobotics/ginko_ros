
#ifndef GINKO_CONTROLLER_H
#define GINKO_CONTROLLER_H

#include <ros/ros.h>

#include <vector>
#include <string>

//#include <sstream>
//#include <cmath>
//#include <cstdlib>
//#include <boost/bind.hpp>
//#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

//GinkoTimer
#include <std_msgs/Int8.h>
#include <time.h>

//GinkoSerial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> // 受信バッファに届いているデータの数を所得するために使用
//#include "serial.hpp"
#include <linux/serial.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <ginko_params.h>
#include "ginko_serial.h"



class GinkoController {
private:
	GinkoSerial ginko_serial_;
	GinkoTimer ginko_timer_;
	GinkoParams ginko_params_;
	unsigned char torque_enable_[GinkoParams::_com_count] = {}
			, torque_request_[GinkoParams::_com_count] = {}
			, pose_request_[GinkoParams::_com_count] = {}
			, ofs_reconf_request[GinkoParams::_com_count] = {};
	double init_pose_[SERVO_NUM]={};
	double target_pose_[SERVO_NUM]={};
	double state_pose_[SERVO_NUM]={};
	unsigned int timestamp_ms_ = 0;
	const unsigned int startup_ms_ = 2000;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
//	ros::NodeHandle priv_node_handle_;
	// ROS Parameters
	// ROS Topic Publisher
	ros::Publisher joint_states_pub_;
	// ROS Topic Subscriber
	ros::Subscriber goal_joint_states_sub_;
	ros::Subscriber torque_enable_sub_;
	// ROS Service Server
	// ROS Service Client
	// Ginko Parameters
	std::string robot_name_;
//	float protocol_version_;

//	DynamixelWorkbench *joint_controller_;
//	DynamixelWorkbench *gripper_controller_;

	std::vector<uint8_t> joint_id_;
	std::vector<uint8_t> gripper_id_;

	std::string joint_mode_;
	std::string gripper_mode_;

	//for using open mp-----
//	sensor_msgs::JointState joint_state;
	float joint_states_pos[SERVO_NUM] = {};
	float joint_states_vel[SERVO_NUM] = {};
	float joint_states_eff[SERVO_NUM] = {};

	double get_joint_position[SERVO_NUM] = {};
	double get_joint_velocity[SERVO_NUM] = {};
	double get_joint_effort[SERVO_NUM] = {};
	//-----

public:
	GinkoController();
	~GinkoController();
	void control_loop_com(unsigned char comnum);
	void control_loop_main();

private:
//	void initMsg();
	void initPublisher();
	void initSubscriber();
	void requestJointStates(unsigned char comnum);
	void updateJointStates();
	void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void torqueEnableCallback(const std_msgs::Int8 &msg);
};


#endif //GINKO_CONTROLLER_H
