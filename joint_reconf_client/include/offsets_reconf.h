#ifndef GINKO_OFFSETS_H
#define GINKO_OFFSETS_H

#include <ros/ros.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <ginko_params.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#define JOINT_BUFFER_NUM 20


class GinkoOffsets {
private:
	int init_flag = 1;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;

	// ROS Topic Subscriber
	ros::Subscriber joint_states_sub_;
	ros::Subscriber init_flag_sub_;

	double servo_states_[SERVO_NUM]={};
	double servo_states_buffer_[SERVO_NUM][JOINT_BUFFER_NUM]={};

public:
	GinkoOffsets();
	~GinkoOffsets();
	double servo_offsets_[SERVO_NUM]={};
	double servo_offsets_calib_[SERVO_NUM]={};
private:
	void initPublisher();
	void initSubscriber();
	void initOffsetsReconfigure();
	void getJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void getGoalJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg);
	void calcJointStatesMedian();
	void calcJointOffsets();
	void reconfigureOffsetsClient();
};


#endif //GINKO_OFFSETS_H
