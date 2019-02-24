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
	geometry_msgs::TransformStamped transformStamped;
	/*
	if (initialize_flag){
//		transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
		transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
		initialize_flag = false;
		return 1;
	}
	//TFの更新に合わせて処理
//	transformStamped = tfBuffer_ptr->lookupTransform("body_imu_yaw", "body_imu_reverse", ros::Time::now(), ros::Duration(0.1));
//	transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
	transformStamped = tfBuffer_ptr->lookupTransform("body_imu_yaw", "body_imu_reverse", ros::Time::now(), ros::Duration(1.0));
	*/
//	transformStamped = tfBuffer_ptr->lookupTransform("leg_l_toe_link0", "leg_r_toe_link0", ros::Time(0), ros::Duration(1.0));
//	transformStamped = tfBuffer_ptr->lookupTransform("body_link1", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
//	transformStamped = tfBuffer_ptr->lookupTransform("leg_r_link8", "leg_r_toe_link0", ros::Time::now(), ros::Duration(1.0));
	transformStamped = tfBuffer_ptr->lookupTransform("leg_r_link7", "leg_r_link8", ros::Time(0), ros::Duration(1.0));
	ros::Time ros_now  = ros::Time::now();
	ros::Duration ros_duration  = ros_now - ros_last;

//	calcRightGroundpoint();

	std_msgs::Float32 tmp;
	tmp.data = transformStamped.transform.rotation.w;
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
//	tf2::Stamped imu_reverse_yaw;
//    tf2::Stamped rotate_tf = tfBuffer.lookupTransform(imu_tf_in_name_,r_toe_tf_in_[0],ros::Time(0));
//	imu_reverse_yaw.header.stamp = ros::Time::now();
//	imu_reverse_yaw.header.frame_id = imu_tf_in_name_;
//	imu_reverse_yaw.child_frame_id = imu_tf_reverse_in_name_;
//	imu_reverse_yaw.transform.translation.x = msg->x;
//	imu_reverse_yaw.transform.translation.y = msg->y;
//	imu_reverse_yaw.transform.translation.z = 0.0;
//	tf2::Quaternion q;
//	q.setRPY(0, 0, msg->theta);
//	imu_reverse_yaw.transform.rotation.x = q.x();
//	imu_reverse_yaw.transform.rotation.y = q.y();
//	imu_reverse_yaw.transform.rotation.z = q.z();
//	imu_reverse_yaw.transform.rotation.w = q.w();
//	tfBroadcaster.sendTransform(imu_reverse_yaw);
}
void FootGrounding::getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

}
void FootGrounding::calcRightGroundpoint(){
	geometry_msgs::TransformStamped toe0_tf = tfBuffer_ptr->lookupTransform(imu_tf_yaw_in_name_,r_toe_tf_in_[0],ros::Time(0));
	geometry_msgs::TransformStamped toe1_tf = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0],r_toe_tf_in_[1],ros::Time(0));
	geometry_msgs::TransformStamped toe2_tf = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0],r_toe_tf_in_[2],ros::Time(0));
	geometry_msgs::TransformStamped toe3_tf = tfBuffer_ptr->lookupTransform(r_toe_tf_in_[0],r_toe_tf_in_[3],ros::Time(0));

	geometry_msgs::TransformStamped transformStamped = toe0_tf;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = r_toe_tf_in_[0];
	transformStamped.child_frame_id = "r_center";
	transformStamped.transform.translation.x = toe2_tf.transform.translation.x * 0.5;
	transformStamped.transform.translation.y = toe2_tf.transform.translation.y * 0.5;
	transformStamped.transform.translation.z = toe2_tf.transform.translation.z * 0.5;
//	transformStamped.transform.rotation.x = rotation.x();
//	transformStamped.transform.rotation.y = rotation.y();
//	transformStamped.transform.rotation.z = rotation.z();
//	transformStamped.transform.rotation.w = rotation.w();
	tfBroadcaster.sendTransform(transformStamped);
}



