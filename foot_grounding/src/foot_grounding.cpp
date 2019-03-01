#include <foot_grounding.h>

//FootGrounding here
FootGrounding::FootGrounding(ros::NodeHandle main_nh){

	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	usleep(1000000);
	readParams(main_nh);
	initSubscriber(main_nh);
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
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 1,&FootGrounding::getImuQuaternionCallback, this);
	joint_states_sub_ = node_handle_.subscribe("joint_states_in", 1,&FootGrounding::getJointStatesCallback, this);
}

void FootGrounding::initPublisher(ros::NodeHandle node_handle_){
	imu_ground_height_pub_	= node_handle_.advertise<std_msgs::Float32>("imu_height_out", 1);
	imu_velocity_pub_		= node_handle_.advertise<geometry_msgs::Vector3>("imu_velocity_out", 1);

	r_ground_height_pub_	= node_handle_.advertise<geometry_msgs::Vector3>("r_ground_height_out", 1);
	r_velocity_pub_			= node_handle_.advertise<geometry_msgs::Vector3>("r_velocity_out", 1);
	l_ground_height_pub_	= node_handle_.advertise<geometry_msgs::Vector3>("l_ground_height_out", 1);
	l_velocity_pub_			= node_handle_.advertise<geometry_msgs::Vector3>("l_velocity_out", 1);
}

int FootGrounding::groundingMainLoop(){
	static bool initialize_flag = true;
	static ros::Time ros_last  = ros::Time::now();
//	ros::Time ros_now  = ros::Time::now();
//	geometry_msgs::TransformStamped transformStamped;
	if (initialize_flag){
		//最初の一回は何もしない(ros_lastのみ更新)
//		transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
		initialize_flag = false;
		return 1;
	}
	//TFの更新に合わせて処理
//	transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));

	ros::Time ros_now  = ros::Time::now();
	ros::Duration ros_duration  = ros_now - ros_last;
	geometry_msgs::TransformStamped transformStampedRight;
	geometry_msgs::TransformStamped transformStampedLight;
	geometry_msgs::TransformStamped transformStampedCenter;
	geometry_msgs::TransformStamped transformStampedFootsGround;
	geometry_msgs::TransformStamped transformStampedImuGround;
	transformStampedRight = calcRightGroundpoint();
	transformStampedLight = calcLeftGroundpoint();
	transformStampedCenter =  calcFootsCenter();
	transformStampedFootsGround = calcGroundpoint(transformStampedCenter,transformStampedRight,transformStampedLight);
	transformStampedImuGround = calcImuGround(transformStampedRight,transformStampedFootsGround);
	std_msgs::Float32 tmp;
//	tmp.data = transformStamped.transform.rotation.w;
	imu_ground_height_pub_.publish(tmp);
	ros_last = ros_now;
	return 0;
}

void FootGrounding::getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
	quaternion_.setX(msg->orientation.x);
	quaternion_.setY(msg->orientation.y);
	quaternion_.setZ(msg->orientation.z);
	quaternion_.setW(msg->orientation.w);
	quaternion_update_flag_ = 1;
}
void FootGrounding::getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

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

	double foots_diff_ratio = 1.0  - (foots_diff.transform.translation.z  / footup_thresh_max_);
	if (foots_diff_ratio<0){
		foots_diff_ratio = 0;
	}else if(foots_diff_ratio>2){
		foots_diff_ratio = 2;
	}
	double gravityPointX = foots_diff.transform.translation.x * 0.5 * foots_diff_ratio;
	double gravityPointY = foots_diff.transform.translation.y * 0.5 * foots_diff_ratio;
	double gravityPointZ = foots_diff.transform.translation.z * 0.5 * foots_diff_ratio;
	//右足接点から見た座標で計算
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = right_ground.child_frame_id;//imu_tf_yaw_in_name_;
	transformStamped.child_frame_id  = ground_point_tf_out_name_;
	transformStamped.transform.translation.x = gravityPointX;
	transformStamped.transform.translation.y = gravityPointY;
	transformStamped.transform.translation.z = gravityPointZ;
	transformStamped.transform.rotation.x		= 0.0;
	transformStamped.transform.rotation.y		= 0.0;
	transformStamped.transform.rotation.z		= 0.0;
	transformStamped.transform.rotation.w		= 1.0;
//	tfBroadcaster.sendTransform(transformStamped);
	//足と足の間の中心からみた座標に変換




	tfBroadcaster.sendTransform(transformStamped);
	return transformStamped;
}

geometry_msgs::TransformStamped FootGrounding::calcImuGround(geometry_msgs::TransformStamped right_ground,geometry_msgs::TransformStamped ground){

	geometry_msgs::TransformStamped imu_ground_tf =	right_ground;
	imu_ground_tf.header.frame_id = imu_tf_yaw_in_name_;
	imu_ground_tf.child_frame_id  = "imu_foot_ground";
	imu_ground_tf.transform.translation.x = 0.0;
	imu_ground_tf.transform.translation.y = 0.0;
	imu_ground_tf.transform.translation.z = right_ground.transform.translation.z + ground.transform.translation.z;
	imu_ground_tf.transform.rotation.x		= 0.0;
	imu_ground_tf.transform.rotation.y		= 0.0;
	imu_ground_tf.transform.rotation.z		= 0.0;
	imu_ground_tf.transform.rotation.w		= 1.0;

	tfBroadcaster.sendTransform(imu_ground_tf);
	return imu_ground_tf;
}
