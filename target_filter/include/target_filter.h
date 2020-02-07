#ifndef GINKO_OFFSETS_H
#define GINKO_OFFSETS_H

#include <ros/ros.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
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
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseStamped.h"

//detect robot falling
#include <sensor_msgs/Imu.h>

class TargetFilter {
private:
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	// ROS Topic Subscriber
	ros::Subscriber r_target_sub_;
	ros::Subscriber l_target_sub_;
	ros::Publisher target_pub_;

	geometry_msgs::PoseStamped r_target_pose_;
	geometry_msgs::PoseStamped l_target_pose_;
	geometry_msgs::PoseStamped target_pose_;
//	geometry_msgs::PoseStamped target_pose_tmp_;
	geometry_msgs::PoseStamped target_pose_slow_;
	//TFもパブリッシュする
	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster staticBroadcaster;

//	//入力
	std::string odom_tf_in_name_ = "odom";
	bool tf_initialized_ = 0;
//	//出力
	std::string target_tf_out_name_ = "target";

	double speed_limit_ = 5.0;
	bool r_updated_ = 0;
	bool l_updated_ = 0;
	ros::Time r_latest_time_;
	ros::Time l_latest_time_;
	//転倒検知
	ros::Subscriber imu_quaternion_sub_;
	int imu_ready_ = 0;
	int imu_fall_direction_ = 0;//0:直立、1:前転倒、2:後転倒
	sensor_msgs::Imu imu_quaternion_;

public:
	TargetFilter(ros::NodeHandle main_nh);
	~TargetFilter();
	int mainLoop();

private:
	void initPublisher();
	void initSubscriber();
	void initTF2();
	void readParams(ros::NodeHandle main_nh);
	void getRightTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void getLeftTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
};


#endif //GINKO_OFFSETS_H
