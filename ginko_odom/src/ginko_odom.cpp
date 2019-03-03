#include <ginko_odom.h>

//GinkoOdometry here
GinkoOdometry::GinkoOdometry(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber(main_nh);
	initPublisher(main_nh);
	usleep(100000);
}

GinkoOdometry::~GinkoOdometry() {
	ros::shutdown();
}

void GinkoOdometry::readParams(ros::NodeHandle node_handle_){
//	node_handle_.param<bool>("publish_debug_topics", publish_debug_topics_, true);
//	node_handle_.param<int>("drift_waiting_conter", imu_drift_waiting_counter_, 100);
}

void GinkoOdometry::initSubscriber(ros::NodeHandle node_handle_){
	imu_height_sub_ = node_handle_.subscribe("imu_height_in", 1,&GinkoOdometry::getImuHeightCallback, this);
	imu_height_vel_sub_ = node_handle_.subscribe("imu_height_vel_in", 1,&GinkoOdometry::getImuHeightVelCallback, this);
	imu_height_acc_sub_ = node_handle_.subscribe("imu_height_acc_in", 1,&GinkoOdometry::getImuHeightAccCallback, this);
	r_pose_sub_ = node_handle_.subscribe("r_pose_in", 1,&GinkoOdometry::getFootRightCallback, this);
	l_pose_sub_ = node_handle_.subscribe("l_pose_in", 1,&GinkoOdometry::getFootLeftCallback, this);
	r_ratio_sub_ = node_handle_.subscribe("r_ratio_in", 1,&GinkoOdometry::getFoorRightRatioCallback, this);
	l_ratio_sub_ = node_handle_.subscribe("l_ratio_in", 1,&GinkoOdometry::getFoorLeftRatioCallback, this);
	imu_sub_ = node_handle_.subscribe("imu_in", 1,&GinkoOdometry::getImuCallback, this);
	ground_pose_sub_ = node_handle_.subscribe("ground_pose_in", 1,&GinkoOdometry::getGroundPoseCallback, this);

//	//Init TF Listener
//	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
//	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
//	sleep(2);//TFが安定するまで待つ(ないと落ちる。良くわからない)
//	tfBuffer_ptr->lookupTransform(odom_parent_name_ ,odom_angle_base_, ros::Time::now(), ros::Duration(1.0));
//	sleep(1);//TFが安定するまで待つ(ないとたまに起動時からずっと更新周期が低くなる。良くわからない)
}


void GinkoOdometry::initPublisher(ros::NodeHandle node_handle_){
	odom_pub_     = node_handle_.advertise<nav_msgs::Odometry>("odom_out", 1);
//	odom_data_.header.frame_id = odom_parent_name_;
	odom_data_.header.frame_id = "body_imu_base_link";
	odom_data_.child_frame_id = odom_name_;
	odom_data_.pose.pose.position.x = 0.;
	odom_data_.pose.pose.position.y = 0.;
	odom_data_.pose.pose.position.z = 0.;
	odom_data_.pose.pose.orientation.x = 0.;
	odom_data_.pose.pose.orientation.y = 0.;
	odom_data_.pose.pose.orientation.z = 0.;
	odom_data_.pose.pose.orientation.w = 1.;

	imu_height_data_.data = 0.;
	imu_data.header.frame_id = "body_imu_base_link";
	imu_data.orientation.x = 0.;
	imu_data.orientation.y = 0.;
	imu_data.orientation.z = 0.;
	imu_data.orientation.w = 1.;
}

int GinkoOdometry::odomLoop(){
	odom_data_.header.frame_id = imu_data.header.frame_id;
	tf2::Quaternion quaternion;
	quaternion.setX(imu_data.orientation.x);
	quaternion.setY(imu_data.orientation.y);
	quaternion.setZ(imu_data.orientation.z);
	quaternion.setW(imu_data.orientation.w);
	tf2::Quaternion quaternion_inv = quaternion.inverse();
	odom_data_.pose.pose.orientation.x = quaternion_inv.getX();
	odom_data_.pose.pose.orientation.y = quaternion_inv.getY();
	odom_data_.pose.pose.orientation.z = quaternion_inv.getZ();
	odom_data_.pose.pose.orientation.w = quaternion_inv.getW();

	tf2::Vector3 height_vector(0.,0.,-imu_height_data_.data);
	tf2::Matrix3x3 rotation_matrix(quaternion_inv);
	tf2::Vector3 height_vector_rotate = rotation_matrix * height_vector;
	odom_data_.pose.pose.position.x = height_vector_rotate.getX();
	odom_data_.pose.pose.position.y = height_vector_rotate.getY();
	odom_data_.pose.pose.position.z = height_vector_rotate.getZ();
	odom_x = 0.1;
	tf2::Vector3 odom_vector(odom_x,odom_y,0.0);
	tf2::Vector3 odom_vector_rotate = rotation_matrix * odom_vector;
	odom_data_.pose.pose.position.x += odom_vector_rotate.getX();
	odom_data_.pose.pose.position.y += odom_vector_rotate.getY();
	odom_data_.pose.pose.position.z += odom_vector_rotate.getZ();
	odomTfPublish(odom_data_);
	return 0;
}
void GinkoOdometry::odomTfPublish(const nav_msgs::Odometry odom){
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = odom_data_.header.frame_id;
	transformStamped.child_frame_id  = odom_data_.child_frame_id;
	transformStamped.transform.translation.x = odom_data_.pose.pose.position.x;
	transformStamped.transform.translation.y = odom_data_.pose.pose.position.y;
	transformStamped.transform.translation.z = odom_data_.pose.pose.position.z;
	transformStamped.transform.rotation.x	= odom_data_.pose.pose.orientation.x;
	transformStamped.transform.rotation.y	= odom_data_.pose.pose.orientation.y;
	transformStamped.transform.rotation.z	= odom_data_.pose.pose.orientation.z;
	transformStamped.transform.rotation.w	= odom_data_.pose.pose.orientation.w;
	tfBroadcaster.sendTransform(transformStamped);

}

void GinkoOdometry::getImuHeightCallback(const std_msgs::Float32::ConstPtr& msg){
	imu_height_data_ = *msg; //中身をコピー
}
void GinkoOdometry::getImuHeightVelCallback(const std_msgs::Float32::ConstPtr& msg){
	imu_height_vel_data_ = *msg;
}
void GinkoOdometry::getImuHeightAccCallback(const std_msgs::Float32::ConstPtr& msg){
	imu_height_acc_data_ = *msg;
}
void GinkoOdometry::getFootRightCallback(const nav_msgs::Odometry::ConstPtr& msg){
	r_pose_data_ = *msg;
}
void GinkoOdometry::getFootLeftCallback(const nav_msgs::Odometry::ConstPtr& msg){
	l_pose_data_ = *msg;
}
void GinkoOdometry::getFoorRightRatioCallback(const std_msgs::Float32::ConstPtr& msg){
	r_ratio_data_ = *msg;
}
void GinkoOdometry::getFoorLeftRatioCallback(const std_msgs::Float32::ConstPtr& msg){
	l_ratio_data_ = *msg;
}
void GinkoOdometry::getImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	imu_data = *msg;
}
void GinkoOdometry::getGroundPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	ground_pose_data_ = *msg;
}
