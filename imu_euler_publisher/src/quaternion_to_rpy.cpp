#include <quaternion_to_rpy.h>

//DriftCorrection here
ImuRpy::ImuRpy(){
	initSubscriber();
	initPublisher();
}
ImuRpy::~ImuRpy() {
	ros::shutdown();
}

void ImuRpy::initSubscriber(){
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 1,&ImuRpy::getQuaternionCallback, this, transport_hints);
//	ROS_FATAL("ImuRpy:Subscriber Initialized");
}
void ImuRpy::initPublisher(){
	imu_euler_pub_ = node_handle_.advertise<geometry_msgs::Vector3>("euler_out", 1);
//	ROS_FATAL("ImuRpy:Publisher Initialized");
}
void ImuRpy::getQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
//	ROS_FATAL("ImuRpy:callback start");
	geometry_msgs::Quaternion quat = msg->orientation;
	tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
	geometry_msgs::Vector3 euler;
	tf2::Matrix3x3(tf2_quat).getEulerYPR(euler.z, euler.y, euler.x);
	imu_euler_pub_.publish(euler);
//	ROS_FATAL("ImuRpy:callback end");
}
