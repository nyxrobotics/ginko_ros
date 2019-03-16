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
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	imu_height_sub_ = node_handle_.subscribe("imu_height_in", 10,&GinkoOdometry::getImuHeightCallback, this, transport_hints);
	imu_height_vel_sub_ = node_handle_.subscribe("imu_height_vel_in", 10,&GinkoOdometry::getImuHeightVelCallback, this, transport_hints);
	imu_height_acc_sub_ = node_handle_.subscribe("imu_height_acc_in", 10,&GinkoOdometry::getImuHeightAccCallback, this, transport_hints);
	r_pose_sub_ = node_handle_.subscribe("r_pose_in", 10,&GinkoOdometry::getFootRightCallback, this, transport_hints);
	l_pose_sub_ = node_handle_.subscribe("l_pose_in", 10,&GinkoOdometry::getFootLeftCallback, this, transport_hints);
	r_ratio_sub_ = node_handle_.subscribe("r_ratio_in", 10,&GinkoOdometry::getFoorRightRatioCallback, this, transport_hints);
	l_ratio_sub_ = node_handle_.subscribe("l_ratio_in", 10,&GinkoOdometry::getFoorLeftRatioCallback, this, transport_hints);
	imu_sub_ = node_handle_.subscribe("imu_in", 10,&GinkoOdometry::getImuCallback, this, transport_hints);
	ground_pose_sub_ = node_handle_.subscribe("ground_pose_in", 10,&GinkoOdometry::getGroundPoseCallback, this, transport_hints);

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
	static int init_flag = 0;
	tf2::Quaternion quaternion;
	static tf2::Quaternion quaternion_prev;
	tf2::Vector3 ground_vector_rotate(ground_pose_data_.pose.position.x, ground_pose_data_.pose.position.y, ground_pose_data_.pose.position.z);
	static tf2::Vector3 ground_vector_rotate_prev;
	tf2::Vector3 right_pose(r_pose_data_.pose.pose.position.x,r_pose_data_.pose.pose.position.y,r_pose_data_.pose.pose.position.z);
	static tf2::Vector3 right_pose_prev;
	tf2::Vector3 left_pose(l_pose_data_.pose.pose.position.x,l_pose_data_.pose.pose.position.y,l_pose_data_.pose.pose.position.z);
	static tf2::Vector3 left_pose_prev;

	//高さ・向きを反映
	odom_data_.header.frame_id = imu_data.header.frame_id;
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
//	tf2::Matrix3x3 rotation_matrix_inv(quaternion);
	tf2::Vector3 height_vector_rotate = rotation_matrix * height_vector;
	odom_data_.pose.pose.position.x = height_vector_rotate.getX();
	odom_data_.pose.pose.position.y = height_vector_rotate.getY();
	odom_data_.pose.pose.position.z = height_vector_rotate.getZ();


	//回転による前後・左右の移動を反映
	double x_diff = 0., y_diff = 0.,z_diff = 0.;

//	tf2::Vector3 odom_vector(odom_x,odom_y,0.0);
//	tf2::Vector3 odom_vector_rotate = rotation_matrix_inv * odom_vector;
//	odom_data_.pose.pose.position.x += odom_vector_rotate.getX();
//	odom_data_.pose.pose.position.y += odom_vector_rotate.getY();
//	odom_data_.pose.pose.position.z += odom_vector_rotate.getZ();
//	odomTfPublish(odom_data_);


	if(init_flag < 1){
		//最初の一回は過去の値がないので加算をしない。
		init_flag ++;
		odom_x = 0.;
		odom_y = 0.;
	}else{
		//足の動きに合わせてodomに加算
		tf2::Vector3 right_pose_diff_rotate = right_pose - right_pose_prev;
		tf2::Vector3 left_pose_diff_rotate = left_pose - left_pose_prev;
		tf2::Vector3 foots_pose_diff_rotate = r_ratio_data_.data*right_pose_diff_rotate + l_ratio_data_.data*left_pose_diff_rotate;
		tf2::Vector3 foots_pose_diff = rotation_matrix.inverse() * foots_pose_diff_rotate;
		odom_x += foots_pose_diff.getX();
		odom_y += foots_pose_diff.getY();
		//姿勢変化による前後・左右移動量分
		tf2::Matrix3x3 rotation_prev(quaternion_prev.inverse());
		tf2::Vector3 ground_vector_prev =   rotation_prev.inverse() * ground_vector_rotate;
		tf2::Vector3 ground_vector 		= rotation_matrix.inverse() * ground_vector_rotate;
		tf2::Vector3 ground_vector_diff = ground_vector - ground_vector_prev;

		tf2::Vector3 ground_vector_prev2 =   rotation_prev.inverse() * ground_vector_rotate_prev;
		tf2::Vector3 ground_vector2 	 = rotation_matrix.inverse() * ground_vector_rotate_prev;
		tf2::Vector3 ground_vector_diff2 = ground_vector2 - ground_vector_prev2;

		x_diff = 0.5*(ground_vector_diff.getX() + ground_vector_diff2.getX());
		y_diff = 0.5*(ground_vector_diff.getY() + ground_vector_diff2.getY());
		z_diff = 0.5*(ground_vector_diff.getY() + ground_vector_diff2.getZ());
		tf2::Vector3 odom_diff_vector(x_diff,y_diff,z_diff);
		odom_x += odom_diff_vector.getX();
		odom_y += odom_diff_vector.getY();
		//ROS_FATAL("odom pose,speed(angle):%f,%f,%f,%f",odom_x,odom_y,odom_diff_vector.getX(),odom_diff_vector.getY());

	}

	tf2::Vector3 odom_vector(x_diff + odom_x,y_diff + odom_y,0.0);
	tf2::Vector3 odom_vector_rotate = rotation_matrix * odom_vector;
	odom_data_.pose.pose.position.x += odom_vector_rotate.getX();
	odom_data_.pose.pose.position.y += odom_vector_rotate.getY();
	odom_data_.pose.pose.position.z += odom_vector_rotate.getZ();
	odomTfPublish(odom_data_);
	quaternion_prev = quaternion;
	ground_vector_rotate_prev = ground_vector_rotate;
	right_pose_prev = right_pose;
	left_pose_prev = left_pose;
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
