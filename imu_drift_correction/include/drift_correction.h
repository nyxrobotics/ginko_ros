#ifndef DRIFT_CORRECTION_H
#define DRIFT_CORRECTION_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <iostream>

class DriftCorrection {
private:
	//NodeHandler,Publisher,Subscriber
	ros::NodeHandle node_handle_;
	ros::Subscriber imu_base_sub_;//ジャイロの生データ。これにドリフト補正をかける。
	ros::Subscriber imu_quaternion_sub_;//ドリフト補正を重力周りにかけるため、現在姿勢も受け取る必要がある。
	ros::Subscriber joint_goals_sub_;	//関節角度の目標値。停止中の判定に使用。
	ros::Subscriber joint_states_sub_;	//関節角度の計測値。停止中の判定に使用。

	ros::Publisher imu_drift_correct_pub_;
	//rosparam
	bool publish_debug_topics_ ;
	int imu_drift_waiting_counter_ ;

	ros::Publisher debug_stopping_pub_;
	ros::Publisher debug_gyro_drift_pub_;
	ros::Publisher debug_accel_drift_pub_;
	ros::Publisher debug_joint_states_drift_pub_;
	ros::Publisher debug_joint_goals_drift_pub_;

	//内部処理用変数
	sensor_msgs::Imu imu_raw_;
	sensor_msgs::Imu imu_quaternion_;

	double stopping_ = 0;		//算出する現在のドリフト値
	double gyro_stopping_ = 0;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double accel_stopping_ = 0;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double joint_target_stopping_ = 0;
	double joint_sens_stopping_ = 0;

	//TODO:パラメータにする
	//各種センサの静止判定のスレッショルド
	double thresh_gyro_stopping_ = 0.002;
	double thresh_accel_stopping_= 0.2;
	double thresh_joint_target_stopping_ = 0.01;
	double thresh_joint_sens_stopping_ = 0.0001;
	//ドリフト補正の1ステップで反映する割合
	double drift_correction_speed_norm_ = 0.01;

	tf2::Quaternion quaternion_;	//ドリフト補正を重力周りにかけるため、現在姿勢も受け取る必要がある。
	tf2::Vector3 drifting_;			//現在姿勢に対して重力周りの回転だけを取り出してドリフトとして引く。

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
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void getJointGoalsCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

	double gyroNormal(const sensor_msgs::Imu imu_in);
	double accelNormal(const sensor_msgs::Imu imu_in);
	double jointStatesNormal(const sensor_msgs::JointState joints_in);
	double jointGoalsNormal(const sensor_msgs::JointState joints_in);
};














#endif //DRIFT_CORRECTION_H
