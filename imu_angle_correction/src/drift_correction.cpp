#include "drift_correction.h"

//DriftCorrection here
DriftCorrection::DriftCorrection(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber();
	initPublisher();
}
DriftCorrection::~DriftCorrection() {
	ros::shutdown();
}
void DriftCorrection::readParams(ros::NodeHandle main_nh){
	main_nh.param<double>("fixture_euler_x", fixture_euler_.x, 0.0);
	main_nh.param<double>("fixture_euler_y", fixture_euler_.y, 0.0);
	main_nh.param<double>("fixture_euler_z", fixture_euler_.z, 0.0);

	main_nh.param<double>("calib_euler_x", calib_euler_.x, 0.0);
	main_nh.param<double>("calib_euler_y", calib_euler_.y, 0.0);
	main_nh.param<double>("calib_euler_z", calib_euler_.z, 0.0);

	main_nh.param<std::string>("parent_link", parent_link, "base");
//	ROS_FATAL("rotation:[x,y,z]=[%f,%f,%f]",fixture_euler_.x,fixture_euler_.y,fixture_euler_.z);
}

void DriftCorrection::initSubscriber(){
	imu_raw_sub_ = node_handle_.subscribe("imu_raw_in", 1,&DriftCorrection::getImuRawCallback, this);
//	ROS_FATAL("ImuRpy:Subscriber Initialized");
}
void DriftCorrection::initPublisher(){
	imu_base_pub_  = node_handle_.advertise<sensor_msgs::Imu>("imu_base_out", 1);
	imu_drift_correct_pub_ = node_handle_.advertise<sensor_msgs::Imu>("imu_drift_correction_out", 1);
//	ROS_FATAL("ImuRpy:Publisher Initialized");
}
void DriftCorrection::getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg){
//	ROS_FATAL("ImuRpy:callback start");
	sensor_msgs::Imu imu_out = *msg; //中身をコピー
//	geometry_msgs::Vector3 angular_velocity_euler = msg->angular_velocity;
	tf2::Vector3 angular_veloc_raw( msg->angular_velocity.x,  msg->angular_velocity.y,  msg->angular_velocity.z);
	tf2::Vector3 linear_accel_raw( msg->linear_acceleration.x,  msg->linear_acceleration.y,  msg->linear_acceleration.z);
//	tf2::Quaternion angular_velocity_quaternion;
//	angular_velocity_quaternion.setEulerZYX(angular_velocity_euler.z, angular_velocity_euler.y, angular_velocity_euler.x);

	tf2::Matrix3x3 rotation_matrix;
	rotation_matrix.setRPY(fixture_euler_.x, fixture_euler_.y, fixture_euler_.z);
	tf2::Matrix3x3 calib_matrix;
	calib_matrix.setRPY(calib_euler_.x, calib_euler_.y, calib_euler_.z);
	tf2::Vector3 angular_veloc_base  = calib_matrix * rotation_matrix * angular_veloc_raw; //この辺の計算はEigenの使い方と同じなのでそっちを参考にしたほうが良い。
	tf2::Vector3 linear_accel_base  = calib_matrix * rotation_matrix * linear_accel_raw;
	imu_out.angular_velocity.x = angular_veloc_base.getX();
	imu_out.angular_velocity.y = angular_veloc_base.getY();
	imu_out.angular_velocity.z = angular_veloc_base.getZ();
	imu_out.linear_acceleration.x = linear_accel_base.getX();
	imu_out.linear_acceleration.y = linear_accel_base.getY();
	imu_out.linear_acceleration.z = linear_accel_base.getZ();
	imu_out.header.frame_id = parent_link;
	imu_base_pub_.publish(imu_out);
	imu_out.angular_velocity.z = 0.0;
	imu_drift_correct_pub_.publish(imu_out);
}



