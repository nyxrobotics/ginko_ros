
#ifndef GINKO_CONTROLLER_H
#define GINKO_CONTROLLER_H

#include <ros/ros.h>

#include <vector>
#include <string>

#include <sensor_msgs/JointState.h>
#include <ginko_params.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
class GinkoController {
private:
	GinkoParams ginko_params_;
	double target_pose_servo_[SERVO_NUM]={};
	double target_pose_gazeo_[GAZEBO_JOINT_NUM]={};
	unsigned int timestamp_ms_ = 0;
	const unsigned int startup_ms_ = 2000;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
//	ros::NodeHandle priv_node_handle_;
	// ROS Parameters
	// ROS Topic Publisher
	ros::Publisher joint_states_pub_;
	ros::Publisher gazebo_joints_pub_[SERVO_NUM];
//	ros::Publisher body_joint1_pub_,
//	arm_r_joint0_pub_,arm_r_joint1_pub_,arm_r_joint2_pub_,arm_r_joint3_pub_,
//	arm_l_joint0_pub_,arm_l_joint1_pub_,arm_l_joint2_pub_,arm_l_joint3_pub_,
//	leg_r_joint0_pub_,leg_r_joint1_pub_,leg_r_joint4_pub_,leg_r_joint7_pub_,leg_r_joint8_pub_,
//	leg_l_joint0_pub_,leg_l_joint1_pub_,leg_l_joint4_pub_,leg_l_joint7_pub_,leg_l_joint8_pub_;
	// ROS Topic Subscriber
	ros::Subscriber goal_joint_states_sub_;
	ros::Subscriber torque_enable_sub_;

	ros::Subscriber body_joint1_sub_,
	arm_r_joint0_sub_,arm_r_joint1_sub_,arm_r_joint2_sub_,arm_r_joint3_sub_,
	arm_l_joint0_sub_,arm_l_joint1_sub_,arm_l_joint2_sub_,arm_l_joint3_sub_,
	leg_r_joint0_sub_,leg_r_joint1_sub_,leg_r_joint4_sub_,leg_r_joint7_sub_,leg_r_joint8_sub_,
	leg_l_joint0_sub_,leg_l_joint1_sub_,leg_l_joint4_sub_,leg_l_joint7_sub_,leg_l_joint8_sub_;
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

private:
	void initPublisher();
	void initSubscriber();
	void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void torqueEnableCallback(const std_msgs::Int8 &msg);
};


#endif //GINKO_CONTROLLER_H
