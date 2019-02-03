#ifndef DRIFT_CORRECTION_H
#define DRIFT_CORRECTION_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


class DriftCorrection {
private:
	LaunchParams launch_params;
	double gyro_z_drift_ = 0.;		//算出する現在のドリフト値
	double gyro_z_stopping_ = 1.;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double joint_target_stopping_ = 0.;
	double joint_sens_stopping_ = 0.;
	double imu_stopping_ = 0.;

	// ROS Topic Subscriber
	ros::Subscriber joint_target_sub_;	//関節角度の目標値。停止中の判定に使用。
	ros::Subscriber joint_sens_sub_;	//関節角度の計測値。停止中の判定に使用。
	ros::Subscriber imu_raw_sub_;		//ジャイロの生データ
	ros::Publisher  imu_drift_correction_pub_; //ドリフト補正後のデータ
public:
	DriftCorrection();
	~DriftCorrection();

	double drift_thresh = 0.99;
	bool publish_debug_topic = true;
	void readParams();

private:
//	void initPublisher();
//	void initSubscriber();

};
















#endif //DRIFT_CORRECTION_H
