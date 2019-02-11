#ifndef ANGLE_CORRECTION_H
#define ANGLE_CORRECTION_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>


class DriftCorrection {
private:
	//NodeHandler,Publisher,Subscriber
	ros::NodeHandle node_handle_;
	ros::Subscriber imu_raw_sub_;
	ros::Publisher imu_base_pub_;
	ros::Publisher imu_drift_correct_pub_;
	//ROS Params
	//座標変換に使用(ジャイロの向き→ロボットの向き)
	geometry_msgs::Vector3 fixture_euler_;
	geometry_msgs::Vector3 calib_euler_;
	//ロボットの向きに合わせたジャイロの位置のリンク
	std::string parent_link;

	double gyro_z_drift_ = 0.;		//算出する現在のドリフト値
	double gyro_z_stopping_ = 1.;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double joint_target_stopping_ = 0.;
	double joint_sens_stopping_ = 0.;
	double imu_stopping_ = 0.;

	// ROS Topic Subscriber
	ros::Subscriber joint_target_sub_;	//関節角度の目標値。停止中の判定に使用。
	ros::Subscriber joint_sens_sub_;	//関節角度の計測値。停止中の判定に使用。


public:
	DriftCorrection(ros::NodeHandle main_nh);
	~DriftCorrection();

	double drift_thresh = 0.99;
	bool publish_debug_topic = true;
	void readParams(ros::NodeHandle main_nh);

private:
	void initPublisher();
	void initSubscriber();
	void getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);

};
















#endif //ANGLE_CORRECTION_H
