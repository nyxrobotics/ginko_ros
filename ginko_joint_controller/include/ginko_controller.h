/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Taehun Lim (Darby) */

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
#include <ginko_joint_controller/servo_offsetsConfig.h> //(project)/cfg/servo_offsets.cfgから自動生成されるらしい
#include <ginko_params.h>
#include "ginko_serial.h"



class GinkoController {
private:
	GinkoSerial ginko_serial_;
	GinkoTimer ginko_timer_;
	GinkoParams ginko_params_;
	unsigned char torque_enable_ = 0 , torque_request_ = 0 , pose_request_ = 0, ofs_reconf_request = 0;
	double init_pose_[SERVO_NUM]={};
	double target_pose_[SERVO_NUM]={};
	double state_pose_[SERVO_NUM]={};
	double servo_offsets_[SERVO_NUM]={};
//	double servo_offsets_[SERVO_NUM]={
//			0,		0,		0.052,	0,		0.09,
//			0,		0.104,	0,		0,		0.104,
//			0.02,	0.1,	0,		0.104,	0,
//			0,		0,		0.12,	0,		0,
//			0,		0,		0.1,	0,		0};
	unsigned int timestamp_ms_ = 0;
	const unsigned int startup_ms_ = 2000;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	ros::NodeHandle priv_node_handle_;
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

	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;

public:
	GinkoController();
	~GinkoController();
	void control_loop();

private:
//	void initMsg();
	void initPublisher();
	void initSubscriber();
	void initOffsetsReconfigure();
	void offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level);
	void updateJointStates();
	void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void torqueEnableCallback(const std_msgs::Int8 &msg);
};


#endif //GINKO_CONTROLLER_H
