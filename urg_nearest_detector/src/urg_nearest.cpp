
//2つのサーボで一自由度を駆動する際の、サーボ同士のズレを補正するための機能。
//dynamic reconfigureで確認しながら調整→servo_offsets.yamlに反映？のような形にしたい


#include <urg_nearest.h>


UrgNearest::UrgNearest(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber();
	initPublisher();
	initTF2();
	ROS_INFO("UrgNearest : Init OK!");

}

UrgNearest::~UrgNearest() {
	ros::shutdown();
}

void UrgNearest::readParams(ros::NodeHandle main_nh){
//	main_nh.param<std::string>("urg_tf", urg_tf_in_name_, "urg_link");
	main_nh.param<std::string>("target_tf", detected_tf_out_name_, "detected_target");
	main_nh.param<std::string>("fixed_tf", odom_tf_in_name_, "odom");
}

void UrgNearest::initPublisher() {
	target_pose_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("target_pose", 0);

}
void UrgNearest::initSubscriber() {
	urg_sub_ = node_handle_.subscribe("scan", 0,&UrgNearest::getLaserScanCallback, this);
}

void UrgNearest::initTF2() {
//	while(urg_ready == 0){
//		usleep(100000);
//	}
	ROS_INFO("UrgNearest : Init 3");
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(2);//TFが安定するまで待つ(ないと落ちる。良くわからない)
//	tfBuffer_ptr->lookupTransform(odom_tf_in_name_, urg_tf_in_name_, ros::Time::now(), ros::Duration(1.0));
//	sleep(1);//TFが安定するまで待つ(ないとたまに起動時からずっと更新周期が低くなる。良くわからない)
	tf2_ready = 1;
}

int UrgNearest::mainLoop(){
	if(urg_ready == 0 || tf2_ready == 0){
		return 0;
	}
	if(urg_updated == 1){
		geometry_msgs::TransformStamped transformStamped;
		transformStamped = tfBuffer_ptr->lookupTransform(odom_tf_in_name_, urg_tf_in_name_,ros::Time(0));
		tf2::Quaternion quat_tmp(transformStamped.transform.rotation.x,
				transformStamped.transform.rotation.y,
				transformStamped.transform.rotation.z,
				transformStamped.transform.rotation.w);
		tf2::Vector3 trans_tmp(transformStamped.transform.translation.x,
				transformStamped.transform.translation.y,
				transformStamped.transform.translation.z);
		tf2::Matrix3x3 rotationalMatrix(quat_tmp);
//		tf2::Vector3 urg_to_target(0.1,0,0);
		tf2::Vector3 odom_to_target;
//		odom_to_target = (rotationalMatrix * urg_to_target) + trans_tmp;
		odom_to_target = (rotationalMatrix * urg_to_nearest_) + trans_tmp;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = odom_tf_in_name_;
		transformStamped.child_frame_id  = detected_tf_out_name_;
		transformStamped.transform.translation.x = odom_to_target.getX();
		transformStamped.transform.translation.y = odom_to_target.getY();
		transformStamped.transform.translation.z = odom_to_target.getZ();
		transformStamped.transform.rotation.x		= 0.0;
		transformStamped.transform.rotation.y		= 0.0;
		transformStamped.transform.rotation.z		= 0.0;
		transformStamped.transform.rotation.w		= 1.0;
//		tfBroadcaster.sendTransform(transformStamped);
		if(nearest_distance_ < nearest_max_){
			//TFを更新
			staticBroadcaster.sendTransform(transformStamped);
			//pose stampedを同時にパブリッシュ
			geometry_msgs::PoseStamped target_pose;
			target_pose.header.frame_id = odom_tf_in_name_;
			target_pose.header.stamp = ros::Time::now();
			target_pose.pose.position.x = transformStamped.transform.translation.x;
			target_pose.pose.position.y = transformStamped.transform.translation.y;
			target_pose.pose.position.z = transformStamped.transform.translation.z;
			target_pose.pose.orientation.x = 0.;
			target_pose.pose.orientation.y = 0.;
			target_pose.pose.orientation.z = 0.;
			target_pose.pose.orientation.w = 1.;
			target_pose_pub_.publish(target_pose);

		}
		urg_updated = 0;
	}else{
		usleep(100);
	}
	return 0;
}


void UrgNearest::getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	scan_in_ = *msg;
	if(urg_ready == 0){
		urg_tf_in_name_ = scan_in_.header.frame_id;
		urg_ready = 1;
		tfBuffer_ptr->lookupTransform(odom_tf_in_name_, urg_tf_in_name_, ros::Time::now(), ros::Duration(1.0));
		sleep(1);
	}
	urg_updated = 1;
	//最近点の位置を計算
	nearest_distance_ = nearest_max_ + 1.;
	double nearest_angle = 0;
	for(int i = 0; i < scan_in_.ranges.size();i++){
		if(scan_in_.ranges[i] < nearest_distance_ &&
				scan_in_.ranges[i] > scan_in_.range_min &&
				scan_in_.ranges[i] < scan_in_.range_max){
			nearest_distance_ = scan_in_.ranges[i];
			nearest_angle = scan_in_.angle_min + scan_in_.angle_increment * i;
		}
	}
//	ROS_INFO("Nearest : %f,%f",nearest_distance_,nearest_angle);
	double nearest_x = nearest_distance_ * cos(nearest_angle);
	double nearest_y = nearest_distance_ * sin(nearest_angle);
	urg_to_nearest_.setX(nearest_x);
	urg_to_nearest_.setY(nearest_y);
	urg_to_nearest_.setZ(0.);
}
