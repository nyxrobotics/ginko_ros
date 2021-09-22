
//2つのサーボで一自由度を駆動する際の、サーボ同士のズレを補正するための機能。
//dynamic reconfigureで確認しながら調整→servo_offsets.yamlに反映？のような形にしたい


#include <target_filter.h>


TargetFilter::TargetFilter(ros::NodeHandle main_nh){
	r_latest_time_ = ros::Time::now();
	l_latest_time_ = ros::Time::now();
	readParams(main_nh);
	initSubscriber();
	initPublisher();
	initTF2();
	ROS_INFO("TargetFilter : Init OK!");
}

TargetFilter::~TargetFilter() {
	ros::shutdown();
}

void TargetFilter::readParams(ros::NodeHandle main_nh){
	main_nh.param<std::string>("target_tf", target_tf_out_name_, "detected_target");
}

void TargetFilter::initPublisher() {
	target_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("target_out", 10);
}

void TargetFilter::initSubscriber() {
//	ros::TransportHints transport_hints;
//	transport_hints.tcpNoDelay(true);
//	r_target_sub_ = node_handle_.subscribe("target_r", 10, &TargetFilter::getRightTargetCallback, this, transport_hints);
//	l_target_sub_ = node_handle_.subscribe("target_l", 10, &TargetFilter::getLeftTargetCallback, this, transport_hints);
	r_target_sub_ = node_handle_.subscribe("target_r", 10, &TargetFilter::getRightTargetCallback, this);
	l_target_sub_ = node_handle_.subscribe("target_l", 10, &TargetFilter::getLeftTargetCallback, this);
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 10,&TargetFilter::getImuQuaternionCallback, this);

}

void TargetFilter::initTF2() {
//	ROS_INFO("TargetFilter : Init 3");
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(2);
	tf_initialized_ = 0;
}

int TargetFilter::mainLoop(){
//	ROS_FATAL("TargetFilter: Loop Start");
	ros::Time time_now  = ros::Time::now();
	static ros::Time time_last  = time_now;
	//データが古い時はスキップ
	ros::Duration r_dutarion = time_now - r_latest_time_;
	ros::Duration l_dutarion = time_now - l_latest_time_;
	if( l_dutarion.toSec() > 5.0){
		 l_updated_ = 0;
	}
	if( r_dutarion.toSec()  > 5.0){
		 r_updated_ = 0;
	}

	//転倒時はスキップ
	if(imu_fall_direction_  != 0){
		 time_last = time_now;
//		ROS_FATAL("TargetFilter: Part2. Robot is not standing -> skip");
		return 0;
	}
	/*
	if( tfBuffer_ptr->canTransform("body_imu_base_link" , "odom",ros::Time(0)) == false){
		 time_last = time_now;
			ROS_FATAL("TargetFilter: Part2. TF not ready -> skip");
		return 0;
	}else{
		geometry_msgs::TransformStamped transformDiff = tfBuffer_ptr->lookupTransform("body_imu_base_link" , "odom",ros::Time(0));
		tf2::Quaternion quat_diff(transformDiff.transform.rotation.x,
				transformDiff.transform.rotation.y,
				transformDiff.transform.rotation.z,
				transformDiff.transform.rotation.z);
		tf2::Vector3 single_z(0,0,1.0);
		tf2::Matrix3x3 rotation_matrix(quat_diff);
		tf2::Vector3 single_z_rot = rotation_matrix * single_z;
		double dx = single_z_rot.x();
		double dy = single_z_rot.y();
		double xy_norm_tmp = sqrt(dx*dx + dy*dy);
		if(xy_norm_tmp > 0.4){
			 time_last = time_now;
				ROS_FATAL("TargetFilter: Part2. Robot is not standing -> skip");
			return 0;
		}
	}
	*/

	//障害物周辺の除去
	if(r_updated_ == 1){
		geometry_msgs::PoseStamped pose_tmp = r_target_pose_;
		int tf_ready = tfBuffer_ptr->canTransform(odom_tf_in_name_ ,ignore_01_name_, ros::Time(0));
		if(tf_ready == 1){
			geometry_msgs::TransformStamped tf_tmp = tfBuffer_ptr->lookupTransform(odom_tf_in_name_ ,ignore_01_name_,ros::Time(0));
			double dx = pose_tmp.pose.position.x - tf_tmp.transform.translation.x;
			double dy = pose_tmp.pose.position.y - tf_tmp.transform.translation.y;
			double length = sqrt(dx*dx + dy*dy);
			if(length < ignore_radious_){
				r_updated_ = 0;
			}
		}
		tf_ready = tfBuffer_ptr->canTransform(odom_tf_in_name_ ,ignore_02_name_, ros::Time(0));
		if(tf_ready == 1){
			geometry_msgs::TransformStamped tf_tmp = tfBuffer_ptr->lookupTransform(odom_tf_in_name_ ,ignore_02_name_,ros::Time(0));
			double dx = pose_tmp.pose.position.x - tf_tmp.transform.translation.x;
			double dy = pose_tmp.pose.position.y - tf_tmp.transform.translation.y;
			double length = sqrt(dx*dx + dy*dy);
			if(length < ignore_radious_){
				r_updated_ = 0;
			}
		}
	}
	if(l_updated_ == 1){
		geometry_msgs::PoseStamped pose_tmp = l_target_pose_;
		int tf_ready = tfBuffer_ptr->canTransform(odom_tf_in_name_ ,ignore_01_name_, ros::Time(0));
		if(tf_ready == 1){
			geometry_msgs::TransformStamped tf_tmp = tfBuffer_ptr->lookupTransform(odom_tf_in_name_ ,ignore_01_name_,ros::Time(0));
			double dx = pose_tmp.pose.position.x - tf_tmp.transform.translation.x;
			double dy = pose_tmp.pose.position.y - tf_tmp.transform.translation.y;
			double length = sqrt(dx*dx + dy*dy);
			if(length < ignore_radious_){
				l_updated_ = 0;
			}
		}
		tf_ready = tfBuffer_ptr->canTransform(odom_tf_in_name_ ,ignore_02_name_, ros::Time(0));
		if(tf_ready == 1){
			geometry_msgs::TransformStamped tf_tmp = tfBuffer_ptr->lookupTransform(odom_tf_in_name_ ,ignore_02_name_,ros::Time(0));
			double dx = pose_tmp.pose.position.x - tf_tmp.transform.translation.x;
			double dy = pose_tmp.pose.position.y - tf_tmp.transform.translation.y;
			double length = sqrt(dx*dx + dy*dy);
			if(length < ignore_radious_){
				l_updated_ = 0;
			}
		}
	}

	//計算開始
	geometry_msgs::PoseStamped target_pose_tmp_;
	if(r_updated_ == 1 && l_updated_ == 0){
		target_pose_tmp_ = r_target_pose_;
	}else if(r_updated_ == 0 && l_updated_ == 1){
		target_pose_tmp_ = l_target_pose_;
	}else if(r_updated_ == 1 && l_updated_ == 1){
		target_pose_tmp_.pose.position.x = (r_target_pose_.pose.position.x + l_target_pose_.pose.position.x)*0.5;
		target_pose_tmp_.pose.position.y = (r_target_pose_.pose.position.y + l_target_pose_.pose.position.y)*0.5;
		target_pose_tmp_.pose.position.z = (r_target_pose_.pose.position.z + l_target_pose_.pose.position.z)*0.5;
	}else{
//		ROS_FATAL("TargetFilter: Part3.NODATA");
		return 0;
	}
	//target_pose_tmp_.pose.position.z = 0;

	static int init_flag = 0;
	if(init_flag < 1){
		//最初の1回は速度が計算できないので何もしない
		init_flag++;
//		ROS_FATAL("TargetFilter: Part4. Timer is not ready -> skip");
		return 0;
	}else{
		ros::Duration ros_duration  =  time_now -  time_last;
		double dt = ros_duration.toSec();
		if(dt<0.000001){
			dt = 0.000001;
//			ROS_FATAL("TargetFilter: Part4. dt is wrong -> skip");
			return 0;
		}
		double dx = target_pose_tmp_.pose.position.x - target_pose_slow_.pose.position.x;
		double dy = target_pose_tmp_.pose.position.y - target_pose_slow_.pose.position.y;
		double dnorm = sqrt(dx*dx + dy*dy);
		double dnorm_limit = speed_limit_ * dt;
		if(dnorm > dnorm_limit){
			dx = dx * dnorm_limit / dnorm;
			dy = dy * dnorm_limit / dnorm;
		}
		//rviz表示用
		target_pose_slow_.header.stamp =  ros::Time::now();
		target_pose_slow_.pose.position.x = target_pose_slow_.pose.position.x + dx;
		target_pose_slow_.pose.position.y = target_pose_slow_.pose.position.y + dy;
		target_pose_slow_.pose.position.z = 0.5; // target_pose_tmp_.pose.position.z;
		tf2::Quaternion quaternion_tmp(1.5708,0,0);
		target_pose_slow_.pose.orientation.x = quaternion_tmp.getX();
		target_pose_slow_.pose.orientation.y = quaternion_tmp.getY();
		target_pose_slow_.pose.orientation.z = quaternion_tmp.getZ();
		target_pose_slow_.pose.orientation.w = quaternion_tmp.getW();
		target_pub_.publish(target_pose_slow_);

		//Publish TF
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = target_pose_slow_.header.frame_id;
		transformStamped.child_frame_id  = target_tf_out_name_;
		transformStamped.transform.translation.x = target_pose_slow_.pose.position.x;
		transformStamped.transform.translation.y = target_pose_slow_.pose.position.y;
		transformStamped.transform.translation.z = target_pose_slow_.pose.position.z;
		transformStamped.transform.rotation.x		= 0.0;
		transformStamped.transform.rotation.y		= 0.0;
		transformStamped.transform.rotation.z		= 0.0;
		transformStamped.transform.rotation.w		= 1.0;
//		staticBroadcaster.sendTransform(transformStamped);
		tfBroadcaster.sendTransform(transformStamped);
		l_updated_ = 0;
		r_updated_ = 0;

//		ROS_FATAL("TargetFilter: Part5. SUCCEED");
	}
	return 0;
}


void TargetFilter::getRightTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	r_target_pose_ = *msg;
	if(tf_initialized_ == 0){
		odom_tf_in_name_ = r_target_pose_.header.frame_id;
		tf_initialized_ = 1;
		target_pose_slow_ = r_target_pose_;
		target_pose_slow_.pose.position.x = 0;
		target_pose_slow_.pose.position.y = 0;
	}
	r_updated_ = 1;
	r_latest_time_ = ros::Time::now();
}

void TargetFilter::getLeftTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	l_target_pose_ = *msg;
	if(tf_initialized_ == 0){
		odom_tf_in_name_ = l_target_pose_.header.frame_id;
		tf_initialized_ = 1;
		target_pose_slow_ = l_target_pose_;
		target_pose_slow_.pose.position.x = 0;
		target_pose_slow_.pose.position.y = 0;
	}
	l_updated_ = 1;
	l_latest_time_ = ros::Time::now();
}

void TargetFilter::getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
	imu_ready_ = 1;
	imu_quaternion_ = *msg;
	tf2::Quaternion tf2_quat(imu_quaternion_.orientation.x, imu_quaternion_.orientation.y, imu_quaternion_.orientation.z, imu_quaternion_.orientation.w);
	geometry_msgs::Vector3 euler;
	tf2::Matrix3x3(tf2_quat).getEulerYPR(euler.z, euler.y, euler.x);
	euler.z = 0.;
	euler.x = 0.;
	tf2::Vector3 single_z(0,0,1.0);
	tf2::Matrix3x3 rotation_matrix;
	rotation_matrix.setRPY(euler.x, euler.y, euler.z);
	tf2::Vector3 single_z_rot = rotation_matrix * single_z;
	double dx = single_z_rot.x();
	double dy = single_z_rot.y();
	double xy_norm_tmp = sqrt(dx*dx + dy*dy);
	//0:直立、1:前転倒、2:後転倒
	if(dx > 0.4){
		imu_fall_direction_ = 1;
	}else if(dx < -0.4){
		imu_fall_direction_ = 2;
	}else{
		imu_fall_direction_ = 0;
	}
}
