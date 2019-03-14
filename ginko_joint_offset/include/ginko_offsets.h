#ifndef GINKO_OFFSETS_H
#define GINKO_OFFSETS_H

#include <ros/ros.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <ginko_joint_offset/servo_offsetsConfig.h> //(project)/cfg/servo_offsets.cfgから自動生成されるらしい
#include <ginko_params.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>

class GinkoOffsets {
private:
	unsigned char ofs_reconf_request = 0;
	int init_flag = 1;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	// ROS Topic Publisher
	ros::Publisher joint_states_ofs_pub_;
	ros::Publisher goal_joint_position_ofs_pub_;
	// ROS Topic Subscriber
	ros::Subscriber joint_states_sub_;
	ros::Subscriber goal_joint_position_sub_;
	ros::Subscriber init_flag_sub_;
	double joint_goal_[SERVO_NUM]={};
	double joint_states_[SERVO_NUM]={};
	double servo_goal_[SERVO_NUM]={};
	double servo_states_[SERVO_NUM]={};
public:
	GinkoOffsets();
	~GinkoOffsets();
	double servo_offsets_[SERVO_NUM]={};
	double servo_offsets_calib_[SERVO_NUM]={};
private:
	void initPublisher();
	void initSubscriber();
	void initOffsetsReconfigure();
	void offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level);
	void getJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void getGoalJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg);
	void reconfigureOffsetsClient();
};


#endif //GINKO_OFFSETS_H
