#include <foot_grounding.h>

//FootGrounding here
FootGrounding::FootGrounding(ros::NodeHandle main_nh){

	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(2.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	usleep(1000000);
	readParams(main_nh);
//	initSubscriber(main_nh);
	initPublisher(main_nh);
}

FootGrounding::~FootGrounding() {
	ros::shutdown();
}

void FootGrounding::readParams(ros::NodeHandle node_handle_){
//	node_handle_.param<bool>("publish_debug_topics", publish_debug_topics_, true);
//	node_handle_.param<int>("drift_waiting_conter", imu_drift_waiting_counter_, 100);
}

void FootGrounding::initSubscriber(ros::NodeHandle node_handle_){
//	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 1,&FootGrounding::getImuQuaternionCallback, this);
//	joint_states_sub_ = node_handle_.subscribe("joint_states_in", 1,&FootGrounding::getJointStatesCallback, this);
}

void FootGrounding::initPublisher(ros::NodeHandle node_handle_){
	imu_height_pub_     = node_handle_.advertise<std_msgs::Float32>("imu_height_out", 1);
	imu_height_vel_pub_	= node_handle_.advertise<std_msgs::Float32>("imu_height_vel_out", 1);
	imu_height_acc_pub_	= node_handle_.advertise<std_msgs::Float32>("imu_height_acc_out", 1);
//	imu_velocity_pub_		= node_handle_.advertise<geometry_msgs::Vector3>("imu_velocity_out", 1);
//	imu_ground_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("imu_ground_pose_out", 1);
	r_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("r_pose_out", 1);
	l_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("l_pose_out", 1);
	r_ratio_pub_= node_handle_.advertise<std_msgs::Float32>("r_ratio_out", 1);
	l_ratio_pub_= node_handle_.advertise<std_msgs::Float32>("l_ratio_out", 1);
}

int FootGrounding::groundingMainLoop(){
	//新しいデータが来るまで待機
	geometry_msgs::TransformStamped transformStamped;
	transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(0.1));

	geometry_msgs::TransformStamped transformStampedRight;
	geometry_msgs::TransformStamped transformStampedLight;
	geometry_msgs::TransformStamped transformStampedCenter;
	geometry_msgs::TransformStamped transformStampedFootsGround;
	geometry_msgs::TransformStamped transformStampedImuGround;
	transformStampedRight = calcRightGroundpoint();
	transformStampedLight = calcLeftGroundpoint();
	transformStampedCenter =  calcFootsCenter();
	transformStampedFootsGround = calcGroundpoint(transformStampedCenter,transformStampedRight,transformStampedLight);
	transformStampedImuGround = calcImuGround(transformStampedFootsGround);
	imu_height_data_.data = -transformStampedImuGround.transform.translation.z;

	//速度の計算をするため、一回前の値を保持
	ros::Time time_now  = ros::Time::now();
	static ros::Time time_last  = ros::Time::now();
	static std_msgs::Float32 imu_height_prev_ = imu_height_data_;
	static std_msgs::Float32 imu_height_vel_prev_ = imu_height_vel_data_;
	static nav_msgs::Odometry r_pose_prev_;
	static nav_msgs::Odometry l_pose_prev_;


	tf2::Quaternion r_quat(r_pose_data_.pose.pose.orientation.x, r_pose_data_.pose.pose.orientation.y, r_pose_data_.pose.pose.orientation.z, r_pose_data_.pose.pose.orientation.w);
	geometry_msgs::Vector3 r_euler;
	tf2::Matrix3x3(r_quat).getEulerYPR(r_euler.z, r_euler.y, r_euler.x);
	tf2::Quaternion l_quat(l_pose_data_.pose.pose.orientation.x, l_pose_data_.pose.pose.orientation.y, l_pose_data_.pose.pose.orientation.z, l_pose_data_.pose.pose.orientation.w);
	geometry_msgs::Vector3 l_euler;
	tf2::Matrix3x3(l_quat).getEulerYPR(l_euler.z, l_euler.y, l_euler.x);
	static geometry_msgs::Vector3 r_euler_prev;
	static geometry_msgs::Vector3 l_euler_prev;
	static int init_flag = 0;
	if(init_flag < 1){
		//最初の2回は速度・加速度が計算できないので何もしない
		init_flag++;
	}else{
		ros::Duration ros_duration  =  time_now -  time_last;
		double dt = ros_duration.toSec();
		imu_height_vel_data_.data = (imu_height_data_.data - imu_height_prev_.data)/dt;
		imu_height_acc_data_.data = (imu_height_vel_data_.data - imu_height_vel_prev_.data)/dt;
		r_pose_data_.twist.twist.linear.x =  (r_pose_data_.pose.pose.position.x - r_pose_prev_.pose.pose.position.x)/dt;
		r_pose_data_.twist.twist.linear.y =  (r_pose_data_.pose.pose.position.y - r_pose_prev_.pose.pose.position.y)/dt;
		r_pose_data_.twist.twist.linear.z =  (r_pose_data_.pose.pose.position.z - r_pose_prev_.pose.pose.position.z)/dt;
		r_pose_data_.twist.twist.angular.x = (r_euler.x - r_euler_prev.x)/dt;
		r_pose_data_.twist.twist.angular.y = (r_euler.y - r_euler_prev.y)/dt;
		r_pose_data_.twist.twist.angular.z = (r_euler.z - r_euler_prev.z)/dt;

		l_pose_data_.twist.twist.linear.x =  (l_pose_data_.pose.pose.position.x - l_pose_prev_.pose.pose.position.x)/dt;
		l_pose_data_.twist.twist.linear.y =  (l_pose_data_.pose.pose.position.y - l_pose_prev_.pose.pose.position.y)/dt;
		l_pose_data_.twist.twist.linear.z =  (l_pose_data_.pose.pose.position.z - l_pose_prev_.pose.pose.position.z)/dt;
		l_pose_data_.twist.twist.angular.x = (l_euler.x - l_euler_prev.x)/dt;
		l_pose_data_.twist.twist.angular.y = (l_euler.y - l_euler_prev.y)/dt;
		l_pose_data_.twist.twist.angular.z = (l_euler.z - l_euler_prev.z)/dt;

		r_ratio_pub_.publish(r_ratio_data_);
		l_ratio_pub_.publish(l_ratio_data_);
		imu_height_pub_.publish(imu_height_data_);
		imu_height_vel_pub_.publish(imu_height_vel_data_);
		imu_height_acc_pub_.publish(imu_height_acc_data_);
		r_pose_pub_.publish(r_pose_data_);
		l_pose_pub_.publish(l_pose_data_);
	}



	//一回前の値を更新
	imu_height_prev_ = imu_height_data_;
	imu_height_vel_prev_ = imu_height_vel_data_;
	r_pose_prev_ = r_pose_data_;
	l_pose_prev_ = l_pose_data_;
	r_euler_prev = r_euler;
	l_euler_prev = l_euler;
	time_last = time_now;

	return 0;
}

geometry_msgs::TransformStamped FootGrounding::calcRightGroundpoint(){
	geometry_msgs::TransformStamped toe_tf[4]	={
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,r_toe_tf_in_[0] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,r_toe_tf_in_[1] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,r_toe_tf_in_[2] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,r_toe_tf_in_[3] ,ros::Time(0))
		};
	geometry_msgs::TransformStamped toe_shape[2]	={
			tfBuffer_ptr->lookupTransform(r_toe_tf_in_[2] ,r_toe_tf_in_[3] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(r_toe_tf_in_[2] ,r_toe_tf_in_[1] ,ros::Time(0))
		};
	double toe_length = toe_shape[0].transform.translation.x;
	double toe_width  = toe_shape[1].transform.translation.y;

	double z2_z3_diff = toe_tf[3].transform.translation.z - toe_tf[2].transform.translation.z;
	double z2_z1_diff = toe_tf[1].transform.translation.z - toe_tf[2].transform.translation.z;

	double gravityPointX = toe_length * 0.5 * (1.0  - (z2_z3_diff / toe_edg_thresh_max_));
	if (gravityPointX > toe_length){
		gravityPointX = toe_length;
	}else if(gravityPointX < 0){
		gravityPointX = 0;
	}

	double gravityPointY = toe_width * 0.5 * (1.0 - (z2_z1_diff / toe_edg_thresh_max_));
	if (gravityPointY > toe_width){
		gravityPointY = toe_width;
	}else if(gravityPointY < 0){
		gravityPointY = 0;
	}

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = r_toe_tf_in_[2];
	transformStamped.child_frame_id  = "r_grounding";
	transformStamped.transform.translation.x = gravityPointX;
	transformStamped.transform.translation.y = gravityPointY;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x		= 0.0;
	transformStamped.transform.rotation.y		= 0.0;
	transformStamped.transform.rotation.z		= 0.0;
	transformStamped.transform.rotation.w		= 1.0;
//	tfBroadcaster.sendTransform(transformStamped);

	//向きをジャイロに合わせる
	transformStamped.header.frame_id = imu_tf_yaw_in_name_;
	transformStamped.child_frame_id  = ground_r_tf_out_name_;
	tf2::Quaternion quaternion;
	quaternion.setX(toe_tf[2].transform.rotation.x);
	quaternion.setY(toe_tf[2].transform.rotation.y);
	quaternion.setZ(toe_tf[2].transform.rotation.z);
	quaternion.setW(toe_tf[2].transform.rotation.w);
	tf2::Matrix3x3 rotationalMatrix(quaternion);
	tf2::Vector3 translation_vector3;
	translation_vector3.setX(gravityPointX);
	translation_vector3.setY(gravityPointY);
	translation_vector3.setZ(0);
	tf2::Vector3 translation_rotate = rotationalMatrix * translation_vector3;
	transformStamped.transform.translation.x = toe_tf[2].transform.translation.x + translation_rotate.x();
	transformStamped.transform.translation.y = toe_tf[2].transform.translation.y + translation_rotate.y();
	transformStamped.transform.translation.z = toe_tf[2].transform.translation.z + translation_rotate.z();
	tfBroadcaster.sendTransform(transformStamped);
//	transformStamped.child_frame_id  = "r_grounding";
	return transformStamped;

}

geometry_msgs::TransformStamped FootGrounding::calcLeftGroundpoint(){
	geometry_msgs::TransformStamped toe_tf[4]	={
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,l_toe_tf_in_[0] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,l_toe_tf_in_[1] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,l_toe_tf_in_[2] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,l_toe_tf_in_[3] ,ros::Time(0))
		};
	geometry_msgs::TransformStamped toe_shape[2]	={
			tfBuffer_ptr->lookupTransform(l_toe_tf_in_[2] ,l_toe_tf_in_[3] ,ros::Time(0)),
			tfBuffer_ptr->lookupTransform(l_toe_tf_in_[2] ,l_toe_tf_in_[1] ,ros::Time(0))
		};
	double toe_length = toe_shape[0].transform.translation.x;
	double toe_width  = toe_shape[1].transform.translation.y;

	double z2_z3_diff = toe_tf[3].transform.translation.z - toe_tf[2].transform.translation.z;
	double z2_z1_diff = toe_tf[1].transform.translation.z - toe_tf[2].transform.translation.z;

	double gravityPointX = toe_length * 0.5 * (1.0  - (z2_z3_diff / toe_edg_thresh_max_));
	if (gravityPointX > toe_length){
		gravityPointX = toe_length;
	}else if(gravityPointX < 0){
		gravityPointX = 0;
	}

	double gravityPointY = toe_width * 0.5 * (1.0 - (z2_z1_diff / toe_edg_thresh_max_));
	if (gravityPointY > toe_width){
		gravityPointY = toe_width;
	}else if(gravityPointY < 0){
		gravityPointY = 0;
	}

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = l_toe_tf_in_[2];
	transformStamped.child_frame_id  = "l_grounding";
	transformStamped.transform.translation.x = gravityPointX;
	transformStamped.transform.translation.y = gravityPointY;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x		= 0.0;
	transformStamped.transform.rotation.y		= 0.0;
	transformStamped.transform.rotation.z		= 0.0;
	transformStamped.transform.rotation.w		= 1.0;
//	tfBroadcaster.sendTransform(transformStamped);

	//向きをジャイロに合わせる
	transformStamped.header.frame_id = imu_tf_yaw_in_name_;
	transformStamped.child_frame_id  = ground_l_tf_out_name_;
	tf2::Quaternion quaternion;
	quaternion.setX(toe_tf[2].transform.rotation.x);
	quaternion.setY(toe_tf[2].transform.rotation.y);
	quaternion.setZ(toe_tf[2].transform.rotation.z);
	quaternion.setW(toe_tf[2].transform.rotation.w);
	tf2::Matrix3x3 rotationalMatrix(quaternion);
	tf2::Vector3 translation_vector3;
	translation_vector3.setX(gravityPointX);
	translation_vector3.setY(gravityPointY);
	translation_vector3.setZ(0);
	tf2::Vector3 translation_rotate = rotationalMatrix * translation_vector3;
	transformStamped.transform.translation.x = toe_tf[2].transform.translation.x + translation_rotate.x();
	transformStamped.transform.translation.y = toe_tf[2].transform.translation.y + translation_rotate.y();
	transformStamped.transform.translation.z = toe_tf[2].transform.translation.z + translation_rotate.z();
	tfBroadcaster.sendTransform(transformStamped);
//	transformStamped.child_frame_id  = "l_grounding";
	return transformStamped;

}

geometry_msgs::TransformStamped FootGrounding::calcFootsCenter(){

	geometry_msgs::TransformStamped foot_right = tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,r_toe_center_tf_ ,ros::Time(0));
	geometry_msgs::TransformStamped foot_left  = tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,l_toe_center_tf_ ,ros::Time(0));
	geometry_msgs::TransformStamped foot_center;
	foot_center.header.stamp    = ros::Time::now();
	foot_center.header.frame_id = imu_tf_yaw_in_name_;
	foot_center.child_frame_id  = ground_center_tf_out_name_;
	foot_center.transform.translation.x = ( foot_right.transform.translation.x + foot_left.transform.translation.x ) * 0.5;
	foot_center.transform.translation.y = ( foot_right.transform.translation.y + foot_left.transform.translation.y ) * 0.5;
	foot_center.transform.translation.z = ( foot_right.transform.translation.z + foot_left.transform.translation.z ) * 0.5;
	foot_center.transform.rotation.x	= 0.0;
	foot_center.transform.rotation.y	= 0.0;
	foot_center.transform.rotation.z	= 0.0;
	foot_center.transform.rotation.w	= 1.0;

	tfBroadcaster.sendTransform(foot_center);
	//以下パブリッシュ用データ代入
	r_pose_data_.header = foot_right.header;
	r_pose_data_.child_frame_id = foot_right.child_frame_id;
	r_pose_data_.pose.pose.position.x = foot_right.transform.translation.x;
	r_pose_data_.pose.pose.position.y = foot_right.transform.translation.y;
	r_pose_data_.pose.pose.position.z = foot_right.transform.translation.z;
	r_pose_data_.pose.pose.orientation.x = foot_right.transform.rotation.x;
	r_pose_data_.pose.pose.orientation.y = foot_right.transform.rotation.y;
	r_pose_data_.pose.pose.orientation.z = foot_right.transform.rotation.z;
	r_pose_data_.pose.pose.orientation.w = foot_right.transform.rotation.w;

	l_pose_data_.header = foot_left.header;
	l_pose_data_.child_frame_id = foot_left.child_frame_id;
	l_pose_data_.pose.pose.position.x = foot_left.transform.translation.x;
	l_pose_data_.pose.pose.position.y = foot_left.transform.translation.y;
	l_pose_data_.pose.pose.position.z = foot_left.transform.translation.z;
	l_pose_data_.pose.pose.orientation.x = foot_left.transform.rotation.x;
	l_pose_data_.pose.pose.orientation.y = foot_left.transform.rotation.y;
	l_pose_data_.pose.pose.orientation.z = foot_left.transform.rotation.z;
	l_pose_data_.pose.pose.orientation.w = foot_left.transform.rotation.w;

	return foot_center;
}

geometry_msgs::TransformStamped FootGrounding::calcGroundpoint(geometry_msgs::TransformStamped center,geometry_msgs::TransformStamped right_ground,geometry_msgs::TransformStamped left_ground){

	geometry_msgs::TransformStamped foots_diff;
	foots_diff.header.stamp = ros::Time::now();
	foots_diff.header.frame_id = imu_tf_yaw_in_name_;
	foots_diff.child_frame_id  = "ground_point_diff";
	foots_diff.transform.translation.x = left_ground.transform.translation.x - right_ground.transform.translation.x;
	foots_diff.transform.translation.y = left_ground.transform.translation.y - right_ground.transform.translation.y;
	foots_diff.transform.translation.z = left_ground.transform.translation.z - right_ground.transform.translation.z;
	foots_diff.transform.rotation.x		= 0.0;
	foots_diff.transform.rotation.y		= 0.0;
	foots_diff.transform.rotation.z		= 0.0;
	foots_diff.transform.rotation.w		= 1.0;

	double foots_diff_ratio = 0.5* (foots_diff.transform.translation.z  / footup_thresh_max_);
	if (foots_diff_ratio<-0.5){
		foots_diff_ratio = -0.5;
	}else if(foots_diff_ratio>0.5){
		foots_diff_ratio = 0.5;
	}
	double gravityPointX = left_ground.transform.translation.x * (0.5 - foots_diff_ratio) + right_ground.transform.translation.x * (0.5 + foots_diff_ratio);
	double gravityPointY = left_ground.transform.translation.y * (0.5 - foots_diff_ratio) + right_ground.transform.translation.y * (0.5 + foots_diff_ratio);
	double gravityPointZ = left_ground.transform.translation.z * (0.5 - foots_diff_ratio) + right_ground.transform.translation.z * (0.5 + foots_diff_ratio);
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = imu_tf_yaw_in_name_;
	transformStamped.child_frame_id  = ground_point_tf_out_name_;
	transformStamped.transform.translation.x = gravityPointX;
	transformStamped.transform.translation.y = gravityPointY;
	transformStamped.transform.translation.z = gravityPointZ;
	transformStamped.transform.rotation.x		= 0.0;
	transformStamped.transform.rotation.y		= 0.0;
	transformStamped.transform.rotation.z		= 0.0;
	transformStamped.transform.rotation.w		= 1.0;
	tfBroadcaster.sendTransform(transformStamped);
	//以下パブリッシュ用データ代入
	r_ratio_data_.data = 0.5 + foots_diff_ratio;
	l_ratio_data_.data = 0.5 - foots_diff_ratio;
	return transformStamped;
}

geometry_msgs::TransformStamped FootGrounding::calcImuGround(geometry_msgs::TransformStamped ground){

	geometry_msgs::TransformStamped imu_ground_tf;
	imu_ground_tf.header.stamp = ros::Time::now();
	imu_ground_tf.header.frame_id = imu_tf_yaw_in_name_;
	imu_ground_tf.child_frame_id  = ground_imu_tf_out_name_;
	imu_ground_tf.transform.translation.x = 0.0;
	imu_ground_tf.transform.translation.y = 0.0;
	imu_ground_tf.transform.translation.z = ground.transform.translation.z;
	imu_ground_tf.transform.rotation.x		= 0.0;
	imu_ground_tf.transform.rotation.y		= 0.0;
	imu_ground_tf.transform.rotation.z		= 0.0;
	imu_ground_tf.transform.rotation.w		= 1.0;

	tfBroadcaster.sendTransform(imu_ground_tf);
	return imu_ground_tf;
}
