#include <rollpitch_tf.h>

//RollPitchTF here
RollPitchTF::RollPitchTF(){
//	if(publish_debug_topic_){
		initPublisher(main_nh);
//	}
}

RollPitchTF::~RollPitchTF() {

}
void RollPitchTF::initPublisher(ros::NodeHandle main_nh){
	yaw_angle_pub_ = main_nh.advertise<std_msgs::Float32>("imu_yaw_angle", 1);
}

tf2::Quaternion RollPitchTF::calcRollPitchQuaternion(const sensor_msgs::Imu imu_in){
//	sensor_msgs::Imu imu_out = imu_in;
	tf2::Quaternion imu_in_quaternion;
	imu_in_quaternion.setX(imu_in.orientation.x);
	imu_in_quaternion.setY(imu_in.orientation.y);
	imu_in_quaternion.setZ(imu_in.orientation.z);
	imu_in_quaternion.setW(imu_in.orientation.w);
	tf2::Matrix3x3 rotation_matrix_in(imu_in_quaternion);
	double euler_z, euler_y, euler_x;
	rotation_matrix_in.getEulerZYX(euler_z, euler_y, euler_x);
	tf2::Matrix3x3 rotation_matrix_out;
	rotation_matrix_in.setEulerZYX(0.0, euler_y, euler_x);
	tf2::Quaternion imu_out_quaternion;
	rotation_matrix_in.getRotation(imu_out_quaternion);
	return imu_out_quaternion;
}

tf2::Quaternion RollPitchTF::calcYawQuaternion(const sensor_msgs::Imu imu_in){
	tf2::Quaternion imu_in_quaternion;
	imu_in_quaternion.setX(imu_in.orientation.x);
	imu_in_quaternion.setY(imu_in.orientation.y);
	imu_in_quaternion.setZ(imu_in.orientation.z);
	imu_in_quaternion.setW(imu_in.orientation.w);
	tf2::Matrix3x3 rotation_matrix_in(imu_in_quaternion);
	tf2::Vector3 x_axis_vector(1.0, 0.0, 0.0);
	tf2::Vector3 face_vector  = rotation_matrix_in * x_axis_vector;
	double theta_z = std::atan2(face_vector.getY(),face_vector.getX());
	if(publish_debug_topic_){
		std_msgs::Float32 tmp;
		tmp.data = theta_z;
		yaw_angle_pub_.publish(tmp);
	}
	tf2::Matrix3x3 rotation_matrix_out;
	rotation_matrix_in.setEulerZYX(theta_z, 0.0, 0.0);
	tf2::Quaternion imu_out_quaternion;
	rotation_matrix_in.getRotation(imu_out_quaternion);
	return imu_out_quaternion;
}

void RollPitchTF::broadcastRotatedTf(std::string parent_name,std::string tf_name,tf2::Quaternion rotation){
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = parent_name;
	transformStamped.child_frame_id = tf_name;
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x = rotation.x();
	transformStamped.transform.rotation.y = rotation.y();
	transformStamped.transform.rotation.z = rotation.z();
	transformStamped.transform.rotation.w = rotation.w();
	tfBroadcaster.sendTransform(transformStamped);
}

void RollPitchTF::broadcastRotatedTfReverse(std::string parent_name,std::string tf_name,tf2::Quaternion rotation){
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = parent_name;
	transformStamped.child_frame_id = tf_name;
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x = rotation.inverse().x();
	transformStamped.transform.rotation.y = rotation.inverse().y();
	transformStamped.transform.rotation.z = rotation.inverse().z();
	transformStamped.transform.rotation.w = rotation.inverse().w();
	tfBroadcaster.sendTransform(transformStamped);
}
