#include <foot_grounding.h>

//FootGrounding here
FootGrounding::FootGrounding(ros::NodeHandle main_nh){
	readParams(main_nh);
//	initSubscriber(main_nh);
	initPublisher(main_nh);
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(2);//TFが安定するまで待つ(ないと落ちる。良くわからない)
	tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_ ,imu_tf_reverse_in_name_, ros::Time::now(), ros::Duration(1.0));
	tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0] ,l_toe_tf_in_[0],ros::Time::now(), ros::Duration(1.0));
	sleep(1);//TFが安定するまで待つ(ないとたまに起動時からずっと更新周期が低くなる。良くわからない)

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
	imu_height_pub_     = node_handle_.advertise<std_msgs::Float32>("imu_height_out", 0);
	imu_height_vel_pub_	= node_handle_.advertise<std_msgs::Float32>("imu_height_vel_out", 0);
	imu_height_acc_pub_	= node_handle_.advertise<std_msgs::Float32>("imu_height_acc_out", 0);
//	imu_velocity_pub_		= node_handle_.advertise<geometry_msgs::Vector3>("imu_velocity_out", 1);
//	imu_ground_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("imu_ground_pose_out", 1);
	r_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("r_pose_out", 0);
	l_pose_pub_	= node_handle_.advertise<nav_msgs::Odometry>("l_pose_out", 0);
	r_ratio_pub_= node_handle_.advertise<std_msgs::Float32>("r_ratio_out", 0);
	l_ratio_pub_= node_handle_.advertise<std_msgs::Float32>("l_ratio_out", 0);
	ground_pose_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("ground_pose_out", 0);
}

int FootGrounding::groundingMainLoop(){
	//新しいデータが来るまで待機

	geometry_msgs::TransformStamped transformStamped;
	transformStamped = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0] ,l_toe_tf_in_[0], ros::Time::now(), ros::Duration(0.2));
	//	usleep(100);//不要なsleep
//	transformStamped = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0] ,l_toe_tf_in_[0], ros::Time(0));

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
	tf2::Quaternion l_quat(l_pose_data_.pose.pose.orientation.x, l_pose_data_.pose.pose.orientation.y, l_pose_data_.pose.pose.orientation.z, l_pose_data_.pose.pose.orientation.w);
	static tf2::Quaternion r_quat_prev;
	static tf2::Quaternion l_quat_prev;
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
		tf2::Quaternion r_quat_diff = r_quat * r_quat_prev.inverse();
		geometry_msgs::Vector3 r_euler_diff;
		tf2::Matrix3x3(r_quat_diff).getEulerYPR(r_euler_diff.z, r_euler_diff.y, r_euler_diff.x);
		r_pose_data_.twist.twist.angular.x = r_euler_diff.x/dt;
		r_pose_data_.twist.twist.angular.y = r_euler_diff.y/dt;
		r_pose_data_.twist.twist.angular.z = r_euler_diff.z/dt;

		l_pose_data_.twist.twist.linear.x =  (l_pose_data_.pose.pose.position.x - l_pose_prev_.pose.pose.position.x)/dt;
		l_pose_data_.twist.twist.linear.y =  (l_pose_data_.pose.pose.position.y - l_pose_prev_.pose.pose.position.y)/dt;
		l_pose_data_.twist.twist.linear.z =  (l_pose_data_.pose.pose.position.z - l_pose_prev_.pose.pose.position.z)/dt;
		tf2::Quaternion l_quat_diff = l_quat * l_quat_prev.inverse();
		geometry_msgs::Vector3 l_euler_diff;
		tf2::Matrix3x3(l_quat_diff).getEulerYPR(l_euler_diff.z, l_euler_diff.y, l_euler_diff.x);
		l_pose_data_.twist.twist.angular.x = l_euler_diff.x/dt;
		l_pose_data_.twist.twist.angular.y = l_euler_diff.y/dt;
		l_pose_data_.twist.twist.angular.z = l_euler_diff.z/dt;

		r_ratio_pub_.publish(r_ratio_data_);
		l_ratio_pub_.publish(l_ratio_data_);
		imu_height_pub_.publish(imu_height_data_);
		imu_height_vel_pub_.publish(imu_height_vel_data_);
		imu_height_acc_pub_.publish(imu_height_acc_data_);
		r_pose_pub_.publish(r_pose_data_);
		l_pose_pub_.publish(l_pose_data_);
		ground_pose_pub_.publish(ground_pose_data_);
	}

	//一回前の値を更新
	imu_height_prev_ = imu_height_data_;
	imu_height_vel_prev_ = imu_height_vel_data_;
	r_pose_prev_ = r_pose_data_;
	l_pose_prev_ = l_pose_data_;
	r_quat_prev = r_quat;
	l_quat_prev = l_quat;
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
	//z軸回転について、ジャイロの回転に足裏の回転が一致する動きの時は足裏中心に寄せる
	ros::Time time_now  = ros::Time::now();
	static ros::Time time_last;
	static int init_flag = 0;
	geometry_msgs::TransformStamped imu_to_center_tf = tfBuffer_ptr->lookupTransform(imu_tf_reverse_in_name_,r_toe_center_tf_,ros::Time(0));
	static geometry_msgs::TransformStamped imu_to_center_tf_prev;
	if(init_flag < 1){
		init_flag++;
	}else{
		tf2::Quaternion quaternion;
		quaternion.setX(imu_to_center_tf.transform.rotation.x);
		quaternion.setY(imu_to_center_tf.transform.rotation.y);
		quaternion.setZ(imu_to_center_tf.transform.rotation.z);
		quaternion.setW(imu_to_center_tf.transform.rotation.w);
		tf2::Quaternion quaternion_prev;
		quaternion_prev.setX(imu_to_center_tf_prev.transform.rotation.x);
		quaternion_prev.setY(imu_to_center_tf_prev.transform.rotation.y);
		quaternion_prev.setZ(imu_to_center_tf_prev.transform.rotation.z);
		quaternion_prev.setW(imu_to_center_tf_prev.transform.rotation.w);
		tf2::Quaternion quaternion_diff = quaternion * quaternion_prev.inverse();
		geometry_msgs::Vector3 euler;
		tf2::Matrix3x3(quaternion_diff).getEulerYPR(euler.z, euler.y, euler.x);
		ros::Duration ros_duration  =  time_now -  time_last;
		double dt = ros_duration.toSec();
		double euler_z_vel = euler.z/dt;
		double edge_ratio = fabs(z2_z3_diff / foot_center_edg_thresh_);
		double euler_ratio = fabs(euler_z_vel / foot_center_rotation_thresh_);
		if(edge_ratio<1.0){
			geometry_msgs::TransformStamped center_tf = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[2],r_toe_center_tf_,ros::Time(0));
			if(edge_ratio>1.0){
				edge_ratio = 1.0;
			}
			if(euler_ratio>1.0){
				euler_ratio = 1.0;
			}
			double center_ratio = (1.0 - edge_ratio) * (1.0 - euler_ratio) * foot_center_multiple_;
			if(center_ratio>1.0){
				center_ratio = 1.0;
			}
			gravityPointX = gravityPointX * (1. - center_ratio) + (0.5 * toe_length)*(center_ratio);
			gravityPointY = gravityPointY * (1. - center_ratio) + (0.5 * toe_width )*(center_ratio);
		}
	}
	imu_to_center_tf_prev = imu_to_center_tf;
	time_last = time_now;


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
	//z軸回転について、ジャイロの回転に足裏の回転が一致する動きの時は足裏中心に寄せる
	ros::Time time_now  = ros::Time::now();
	static ros::Time time_last;
	static int init_flag = 0;
	geometry_msgs::TransformStamped imu_to_center_tf = tfBuffer_ptr->lookupTransform(imu_tf_reverse_in_name_,l_toe_center_tf_,ros::Time(0));
	static geometry_msgs::TransformStamped imu_to_center_tf_prev;
	if(init_flag < 1){
		init_flag++;
	}else{
		tf2::Quaternion quaternion;
		quaternion.setX(imu_to_center_tf.transform.rotation.x);
		quaternion.setY(imu_to_center_tf.transform.rotation.y);
		quaternion.setZ(imu_to_center_tf.transform.rotation.z);
		quaternion.setW(imu_to_center_tf.transform.rotation.w);
		tf2::Quaternion quaternion_prev;
		quaternion_prev.setX(imu_to_center_tf_prev.transform.rotation.x);
		quaternion_prev.setY(imu_to_center_tf_prev.transform.rotation.y);
		quaternion_prev.setZ(imu_to_center_tf_prev.transform.rotation.z);
		quaternion_prev.setW(imu_to_center_tf_prev.transform.rotation.w);
		tf2::Quaternion quaternion_diff = quaternion * quaternion_prev.inverse();
		geometry_msgs::Vector3 euler;
		tf2::Matrix3x3(quaternion_diff).getEulerYPR(euler.z, euler.y, euler.x);
		ros::Duration ros_duration  =  time_now -  time_last;
		double dt = ros_duration.toSec();
		double euler_z_vel = euler.z/dt;
		double edge_ratio = fabs(z2_z3_diff / foot_center_edg_thresh_);
		double euler_ratio = fabs(euler_z_vel / foot_center_rotation_thresh_);
		if(edge_ratio<1.0){
			geometry_msgs::TransformStamped center_tf = tfBuffer_ptr->lookupTransform(l_toe_tf_in_[2],l_toe_center_tf_,ros::Time(0));
			if(edge_ratio>1.0){
				edge_ratio = 1.0;
			}
			if(edge_ratio < foot_center_edg_min_){
				edge_ratio = 0.;
			}else{
				edge_ratio -= foot_center_edg_min_;
				edge_ratio *= (1.0/(1.0-foot_center_edg_min_));
			}
			if(euler_ratio>1.0){
				euler_ratio = 1.0;
			}
			if(euler_ratio < foot_center_rotation_min_){
				euler_ratio = 0.;
			}else{
				euler_ratio -= foot_center_rotation_min_;
				euler_ratio *= (1.0/(1.0-foot_center_rotation_min_));
			}
			double center_ratio = (1.0 - edge_ratio) * (1.0 - euler_ratio) * foot_center_multiple_;
			if(center_ratio>1.0){
				center_ratio = 1.0;
			}
			gravityPointX = gravityPointX * (1. - center_ratio) + (0.5 * toe_length)*(center_ratio);
			gravityPointY = gravityPointY * (1. - center_ratio) + (0.5 * toe_width )*(center_ratio);
		}
	}
	imu_to_center_tf_prev = imu_to_center_tf;
	time_last = time_now;



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
	//imu_tf_in_name_を親にして生やす
	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(imu_tf_in_name_, imu_tf_yaw_in_name_, ros::Time(0));
	tf2::Quaternion quaternion_diff;
	quaternion_diff.setX(tf_diff.transform.rotation.x);
	quaternion_diff.setY(tf_diff.transform.rotation.y);
	quaternion_diff.setZ(tf_diff.transform.rotation.z);
	quaternion_diff.setW(tf_diff.transform.rotation.w);
	tf2::Matrix3x3 rotation_diff(quaternion_diff);
	tf2::Vector3 r_vector_base(foot_right.transform.translation.x,foot_right.transform.translation.y,foot_right.transform.translation.z);
	tf2::Vector3 r_vector_rotate = rotation_diff * r_vector_base;
	tf2::Quaternion r_quaternion_base(foot_right.transform.rotation.x,foot_right.transform.rotation.y,foot_right.transform.rotation.z,foot_right.transform.rotation.w);
	tf2::Quaternion r_quaternion_rotate = quaternion_diff * r_quaternion_base;
	r_pose_data_.header = foot_right.header;
	r_pose_data_.header.frame_id = imu_tf_in_name_;
	r_pose_data_.child_frame_id = foot_right.child_frame_id;
	r_pose_data_.pose.pose.position.x = r_vector_rotate.getX();
	r_pose_data_.pose.pose.position.y = r_vector_rotate.getY();
	r_pose_data_.pose.pose.position.z = r_vector_rotate.getZ();
	r_pose_data_.pose.pose.orientation.x = r_quaternion_rotate.getX();
	r_pose_data_.pose.pose.orientation.y = r_quaternion_rotate.getY();
	r_pose_data_.pose.pose.orientation.z = r_quaternion_rotate.getZ();
	r_pose_data_.pose.pose.orientation.w = r_quaternion_rotate.getW();

	tf2::Vector3 l_vector_base(foot_left.transform.translation.x,foot_left.transform.translation.y,foot_left.transform.translation.z);
	tf2::Vector3 l_vector_rotate = rotation_diff * l_vector_base;
	tf2::Quaternion l_quaternion_base(foot_left.transform.rotation.x,foot_left.transform.rotation.y,foot_left.transform.rotation.z,foot_left.transform.rotation.w);
	tf2::Quaternion l_quaternion_rotate = quaternion_diff * l_quaternion_base;
	l_pose_data_.header = foot_left.header;
	l_pose_data_.header.frame_id = imu_tf_in_name_;
	l_pose_data_.child_frame_id = foot_left.child_frame_id;
	l_pose_data_.pose.pose.position.x = l_vector_rotate.getX();
	l_pose_data_.pose.pose.position.y = l_vector_rotate.getY();
	l_pose_data_.pose.pose.position.z = l_vector_rotate.getZ();
	l_pose_data_.pose.pose.orientation.x = l_quaternion_rotate.getX();
	l_pose_data_.pose.pose.orientation.y = l_quaternion_rotate.getY();
	l_pose_data_.pose.pose.orientation.z = l_quaternion_rotate.getZ();
	l_pose_data_.pose.pose.orientation.w = l_quaternion_rotate.getW();
	/*
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
	*/

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
	//接地点はimu_tf_in_name_を親にして生やす
	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(imu_tf_in_name_, imu_tf_yaw_in_name_, ros::Time(0));
	tf2::Quaternion quaternion_diff;
	quaternion_diff.setX(tf_diff.transform.rotation.x);
	quaternion_diff.setY(tf_diff.transform.rotation.y);
	quaternion_diff.setZ(tf_diff.transform.rotation.z);
	quaternion_diff.setW(tf_diff.transform.rotation.w);
	tf2::Matrix3x3 rotation_diff(quaternion_diff);
	tf2::Vector3 vector_base(gravityPointX,gravityPointY,gravityPointZ);
	tf2::Vector3 vector_rotate = rotation_diff * vector_base;
	ground_pose_data_.header.frame_id = imu_tf_in_name_;
	ground_pose_data_.header.stamp = ros::Time::now();
	ground_pose_data_.pose.position.x = vector_rotate.getX();
	ground_pose_data_.pose.position.y = vector_rotate.getY();
	ground_pose_data_.pose.position.z = vector_rotate.getZ();
	ground_pose_data_.pose.orientation.x = 0.;
	ground_pose_data_.pose.orientation.y = 0.;
	ground_pose_data_.pose.orientation.z = 0.;
	ground_pose_data_.pose.orientation.w = 1.;
//	ground_pose_data_.header.frame_id = imu_tf_yaw_in_name_;
//	ground_pose_data_.pose.position.x = gravityPointX;
//	ground_pose_data_.pose.position.y = gravityPointY;
//	ground_pose_data_.pose.position.z = gravityPointZ;
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
