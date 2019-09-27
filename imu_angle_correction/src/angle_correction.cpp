#include <angle_correction.h>

//DriftCorrection here
DriftCorrection::DriftCorrection(ros::NodeHandle main_nh){
	readParams(main_nh);
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(2);
	transformDiff = tfBuffer_ptr->lookupTransform(target_link ,attached_link, ros::Time::now(), ros::Duration(10.0));
//	transformDiff = tfBuffer_ptr->lookupTransform(attached_link, target_link, ros::Time::now(), ros::Duration(10.0));
	initSubscriber();
	initPublisher();
}
DriftCorrection::~DriftCorrection() {
	ros::shutdown();
}
void DriftCorrection::readParams(ros::NodeHandle main_nh){
	main_nh.param<double>("calib_euler_x", calib_euler_.x, 0.0);
	main_nh.param<double>("calib_euler_y", calib_euler_.y, 0.0);
	main_nh.param<double>("calib_euler_z", calib_euler_.z, 0.0);

	main_nh.param<std::string>("attached_link", attached_link, "imu_attached");
	main_nh.param<std::string>("target_link", target_link, "imu_base");
//	ROS_FATAL("rotation:[x,y,z]=[%f,%f,%f]",fixture_euler_.x,fixture_euler_.y,fixture_euler_.z);
}

void DriftCorrection::initSubscriber(){
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	imu_raw_sub_ = node_handle_.subscribe("imu_raw_in", 10,&DriftCorrection::getImuRawCallback, this, transport_hints);
}
void DriftCorrection::initPublisher(){
	imu_base_pub_  = node_handle_.advertise<sensor_msgs::Imu>("imu_base_out", 10);
}
void DriftCorrection::getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg){
//	ROS_FATAL("ImuRpy:callback start");
	sensor_msgs::Imu imu_in = *msg; //中身をコピー
	sensor_msgs::Imu imu_out = imu_in; //中身をコピー

	tf2::Vector3 angular_veloc_raw(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
	tf2::Vector3 linear_accel_raw(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
	tf2::Quaternion orientation_diff(transformDiff.transform.rotation.x,
			transformDiff.transform.rotation.y,
			transformDiff.transform.rotation.z,
			transformDiff.transform.rotation.w
			);
	tf2::Matrix3x3 rotation_matrix(orientation_diff);
	tf2::Matrix3x3 calib_matrix;
	calib_matrix.setRPY(calib_euler_.x, calib_euler_.y, calib_euler_.z);
	tf2::Vector3 angular_veloc_base  = calib_matrix* rotation_matrix * angular_veloc_raw; //この辺の計算はEigenの使い方と同じなのでそっちを参考にしたほうが良い。
	tf2::Vector3 linear_accel_base  = calib_matrix * rotation_matrix * linear_accel_raw;
	imu_out.angular_velocity.x = angular_veloc_base.getX();
	imu_out.angular_velocity.y = angular_veloc_base.getY();
	imu_out.angular_velocity.z = angular_veloc_base.getZ();
	imu_out.linear_acceleration.x = linear_accel_base.getX();
	imu_out.linear_acceleration.y = linear_accel_base.getY();
	imu_out.linear_acceleration.z = linear_accel_base.getZ();
	imu_out.header.frame_id = target_link;
	imu_base_pub_.publish(imu_out);
//	imu_out.angular_velocity.z = 0.0;
//	imu_drift_correct_pub_.publish(imu_out);
}



