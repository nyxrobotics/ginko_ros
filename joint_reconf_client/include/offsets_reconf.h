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

//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


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
	double servo_offsets_[SERVO_NUM]={};
	double servo_offsets_calib_[SERVO_NUM]={};

	//股関節の補正のためTF2を使う
	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	std::string crotch_tf_ = "body_imu_yaw";
	std::string r_toe_center_tf_ = "leg_r_toe_center";
	std::string l_toe_center_tf_ = "leg_l_toe_center";
	double toe_width = 0.08;//本来0.08でちょうどのはずだが合わないので調整した
	double crotch_offsets_calib_[2]={};
	double crotch_offsets_calib_buffer_[2][JOINT_BUFFER_NUM]={};
public:
	GinkoOffsets();
	~GinkoOffsets();


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

	void initTF2();
	void calcCrotchOffsets();
	void calcCrotchOffsetsMedian();
};


#endif //GINKO_OFFSETS_H
