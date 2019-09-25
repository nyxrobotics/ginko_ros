#include <gazebo_imu_accel.h>

//AccelOverwrite here
AccelOverwrite::AccelOverwrite(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber();
	initPublisher();
}
AccelOverwrite::~AccelOverwrite() {
	ros::shutdown();
}
void AccelOverwrite::readParams(ros::NodeHandle main_nh){
}
void AccelOverwrite::initSubscriber(){
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	imu_raw_sub_ = node_handle_.subscribe("imu_in", 10, &AccelOverwrite::getImuRawCallback, this, transport_hints);
}
void AccelOverwrite::initPublisher(){
	imu_base_pub_  = node_handle_.advertise<sensor_msgs::Imu>("imu_out", 10);
}
void AccelOverwrite::getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg){
//	ROS_FATAL("ImuRpy:callback start");
	sensor_msgs::Imu imu_out = *msg; //中身をコピー
	tf2::Quaternion imu_quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf2::Vector3 gravity(0.0, 0.0, 9.8);
	tf2::Matrix3x3 rotation_matrix(imu_quaternion);
	tf2::Vector3 gravity_rotated = rotation_matrix.inverse() * gravity;
	imu_out.linear_acceleration.x = gravity_rotated.getX();
	imu_out.linear_acceleration.y = gravity_rotated.getY();
	imu_out.linear_acceleration.z = gravity_rotated.getZ();
	imu_base_pub_.publish(imu_out);
}



