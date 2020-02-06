#ifndef BATTLE_PLANNER_H
#define BATTLE_PLANNER_H

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

class BattlePlanner {
private:
	//動作:


//	ros::NodeHandle node_handle_;
	ros::Subscriber imu_quaternion_sub_;
	ros::Subscriber battle_command_sub_;
	int imu_ready_ = 0;
	int imu_fall_direction_ = 0;//0:直立、1:前転倒、2:後転倒
	int battle_command_ready_ = 0;
	int battle_command_update_ = 0;
	ros::Publisher motion_command_pub_;
	std_msgs::String motion_command_;

	sensor_msgs::Imu imu_quaternion_;
	std_msgs::String battle_command_;
	std::string battle_state_;

	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;

	std::string robot_tf_ = "body_imu_base_link";	//"ground_foot_center_link";	//ジャイロのTF
	std::string ring_tf_ = "ring_center";	//"ground_foot_center_link";	//ジャイロのTF
	std::string target_tf_ = "target_slow";	//ジャイロのTF

	double area_distance_threth_ = 0.45;
	double area_angle_threth_ = 0.7;//1.0472;
	double ring_radious_, center_radious_, outer_radious;


public:
	BattlePlanner(ros::NodeHandle main_nh);
	~BattlePlanner();
	void readParams(ros::NodeHandle node_handle_);
	int mainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);
	void initTF2();
	void getBattleCommandCallback(const std_msgs::String::ConstPtr& msg);
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
	int battleMotionSelect();
	int approachTarget(const std::string target_tf_name,
			const double distance_margin,
			const double angle_margin,
			const double tf_timeout,
			const double motion_timeout,
			bool init_flag);
	int attackTarget(const std::string target_tf_name,
			const double distance_margin,
			const double angle_margin,
			const double tf_timeout,
			const double motion_timeout,
			bool init_flag);
	int avoidTarget(const std::string target_tf_name,
			const double distance_margin,
			const double angle_margin,
			const double tf_timeout,
			const double motion_timeout,
			bool init_flag);
	int searchTarget(const std::string target_tf_name,
			const double tf_timeout,
			bool init_flag);
	int checkArea(const std::string target_tf_name);
};

enum MOTION_STATUS {
	CONTINUE,
	SUCCEED,
	FAILED,
	TF_TIMEOUT,
	MOTION_TIMEOUT,
	INITIALIZED,
	ERROR
};

enum RING_AREAS {
	RING_CENTER,
	RING_MIDDLE,
	RING_OUTER,
	RING_OUTSIDE,
	RING_ERROR
};

#endif //BATTLE_PLANNER
