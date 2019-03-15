#include "drift_correction.h"


//DriftCorrection here
DriftCorrection::DriftCorrection(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber();
	initPublisher();
	drifting_.setX(0);
	drifting_.setY(0);
	drifting_.setZ(0);
}
DriftCorrection::~DriftCorrection() {
	ros::shutdown();
}
void DriftCorrection::readParams(ros::NodeHandle main_nh){
	main_nh.param<bool>("publish_debug_topics", publish_debug_topics_, true);
	main_nh.param<int>("drift_waiting_conter", imu_drift_waiting_counter_, 100);
}

void DriftCorrection::initSubscriber(){
	imu_base_sub_ = node_handle_.subscribe("imu_raw_in", 1,&DriftCorrection::getImuRawCallback, this);
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 1,&DriftCorrection::getImuQuaternionCallback, this);
	joint_goals_sub_ = node_handle_.subscribe("joint_goals_in", 1,&DriftCorrection::getJointGoalsCallback, this);
	joint_states_sub_ = node_handle_.subscribe("joint_states_in", 1,&DriftCorrection::getJointStatesCallback, this);
//	ROS_FATAL("ImuRpy:Subscriber Initialized");
}
void DriftCorrection::initPublisher(){
	imu_drift_correct_pub_ = node_handle_.advertise<sensor_msgs::Imu>("imu_drift_correction_out", 1);
//	ROS_FATAL("ImuRpy:Publisher Initialized");
	if(publish_debug_topics_ == true){
		debug_stopping_pub_ = node_handle_.advertise<std_msgs::Float32>("debug/stopping_flag", 1);
		debug_gyro_drift_pub_ = node_handle_.advertise<std_msgs::Float32>("debug/gyro_flag", 1);
		debug_accel_drift_pub_ = node_handle_.advertise<std_msgs::Float32>("debug/accel_flag", 1);
		debug_joint_states_drift_pub_ = node_handle_.advertise<std_msgs::Float32>("debug/joint_states_flag", 1);
		debug_joint_goals_drift_pub_ = node_handle_.advertise<std_msgs::Float32>("debug/joint_goals_flag", 1);
	}
}
void DriftCorrection::getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg){
	sensor_msgs::Imu imu_out = *msg; //中身をコピー
	gyro_stopping_ = gyroNormal(*msg);
	accel_stopping_ = accelNormal(*msg);
	//ドリフト補正が可能か判定
	//stopping_が1.0以上になったら静止していると判定
	if(gyro_stopping_<thresh_gyro_stopping_
			&& accel_stopping_<thresh_accel_stopping_
			&& joint_target_stopping_<thresh_joint_target_stopping_
			&& joint_sens_stopping_<thresh_joint_sens_stopping_){
		if(stopping_< 1.1){
			stopping_ += 1.1/(double)imu_drift_waiting_counter_;
		}
	}else{
		stopping_ = 0;
	}
	//ドリフト補正開始
	if(stopping_>1.0){
		tf2::Vector3 gyro_latest(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);//最新のジャイロの生データ
		tf2::Matrix3x3 calib_matrix(quaternion_);
		tf2::Vector3 drifting_calib = calib_matrix * gyro_latest;
		//Z軸周り以外はmadgwickがとってくれるので無視
		drifting_calib.setX(0.0);
		drifting_calib.setY(0.0);
		tf2::Vector3 drifting_latest = calib_matrix.inverse() * gyro_latest;
		drifting_ = drifting_*(1.0-drift_correction_speed_norm_) + drifting_latest*drift_correction_speed_norm_;

	}

	imu_out.angular_velocity.x -= drifting_.getX();
	imu_out.angular_velocity.y -= drifting_.getY();
	imu_out.angular_velocity.z -= drifting_.getZ();
	imu_out.header.stamp = ros::Time::now();
	imu_drift_correct_pub_.publish(imu_out);

	if(publish_debug_topics_){
		std_msgs::Float32 tmp;
		tmp.data = stopping_;
		debug_stopping_pub_.publish(tmp);
		tmp.data = gyro_stopping_ / thresh_gyro_stopping_;
		debug_gyro_drift_pub_.publish(tmp);
		tmp.data = accel_stopping_ / thresh_accel_stopping_;
		debug_accel_drift_pub_.publish(tmp);
		tmp.data = joint_target_stopping_ / thresh_joint_target_stopping_;
		debug_joint_goals_drift_pub_.publish(tmp);
		tmp.data = joint_sens_stopping_ / thresh_joint_sens_stopping_;
		debug_joint_states_drift_pub_.publish(tmp);
	}
	joint_target_stopping_*=0.999;//目標値は来ない時もあるのでその時は少しづつ減らす。
}
void DriftCorrection::getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
	quaternion_.setX(msg->orientation.x);
	quaternion_.setY(msg->orientation.y);
	quaternion_.setZ(msg->orientation.z);
	quaternion_.setW(msg->orientation.w);
	tf2::Quaternion yaw_quaternion = roll_pitch_tf.calcYawQuaternion(*msg);
	roll_pitch_tf.broadcastRotatedTf("body_imu_reverse", "body_imu_yaw", yaw_quaternion);
}
void DriftCorrection::getJointGoalsCallback(const sensor_msgs::JointState::ConstPtr& msg){
//	sensor_msgs::JointState jg_tmp = *msg;
	joint_target_stopping_ = jointStatesNormal(*msg);
}

void DriftCorrection::getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
//	sensor_msgs::JointState js_tmp = *msg;
	joint_sens_stopping_ = jointStatesNormal(*msg);
}

double DriftCorrection::gyroNormal(const sensor_msgs::Imu imu_in){
	double normal = 0;
	//2乗はpowより掛け算のほうが早いらしい(参考:https://fenrir.naruoka.org/archives/000623.html)
	normal += imu_in.angular_velocity.x * imu_in.angular_velocity.x;
	normal += imu_in.angular_velocity.y * imu_in.angular_velocity.y;
	normal += imu_in.angular_velocity.z * imu_in.angular_velocity.z;
	return normal;
}
double DriftCorrection::accelNormal(const sensor_msgs::Imu imu_in){
	double normal = 0;
	static sensor_msgs::Imu imu_old = imu_in;
	normal += (imu_in.linear_acceleration.x - imu_old.linear_acceleration.x) * (imu_in.linear_acceleration.x - imu_old.linear_acceleration.x);
	normal += (imu_in.linear_acceleration.y - imu_old.linear_acceleration.y) * (imu_in.linear_acceleration.y - imu_old.linear_acceleration.y);
	normal += (imu_in.linear_acceleration.z - imu_old.linear_acceleration.z) * (imu_in.linear_acceleration.z - imu_old.linear_acceleration.z);
	imu_old = imu_in;
	return normal;
}
double DriftCorrection::jointStatesNormal(const sensor_msgs::JointState joints_in){
	double normal = 0;
	static sensor_msgs::JointState js_old = joints_in;
	int joint_num = joints_in.position.size();
	for(int i=0;i<joint_num;i++){
		normal += (joints_in.position[i]-js_old.position[i])*(joints_in.position[i]-js_old.position[i]);
	}
	js_old = joints_in;
	ROS_DEBUG("DriftCorrection:[%d] Joints Detected",joint_num);
	return normal;
}
double DriftCorrection::jointGoalsNormal(const sensor_msgs::JointState joints_in){
	double normal = 0;
	static sensor_msgs::JointState js_old = joints_in;
	int joint_num = joints_in.position.size();
	for(int i=0;i<joint_num;i++){
		normal += (joints_in.position[i]-js_old.position[i])*(joints_in.position[i]-js_old.position[i]);
	}
	js_old = joints_in;
	ROS_DEBUG("DriftCorrection:[%d] Joints Detected",joint_num);
	return normal;
}
