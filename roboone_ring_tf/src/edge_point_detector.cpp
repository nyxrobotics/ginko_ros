#include <edge_point_detector.h>

//左右の腕のURGを受け取り、それぞれ前後

//EdgePointDetector here
EdgePointDetector::EdgePointDetector(ros::NodeHandle main_nh){
	readParams(main_nh);
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr_.reset(new tf2_ros::Buffer(ros::Duration(10.0), false));
	tfListener_ptr_.reset(new tf2_ros::TransformListener(*tfBuffer_ptr_));
	sleep(1);//TFが安定するまで待つ(ないと落ちる?要検証)
	//tfBuffer_ptr->canTransform(robot_center_tf_,odom_tf_, ros::Time::now(), ros::Duration(10.0));
	//sleep(1);//TFが安定するまで待つ(ないとたまに起動時からずっと更新周期が低くなる。良くわからない)
	initSubscriber(main_nh);
	initPublisher(main_nh);
	//Resize Vectors
	right_detected_edges_.resize(2);
	right_detected_edges_flag_.resize(2);
	right_detected_edges_flag_[0] = 0;
	right_detected_edges_flag_[1] = 0;

	left_detected_edges_.resize(2);
	left_detected_edges_flag_.resize(2);
	left_detected_edges_flag_[0] = 0;
	left_detected_edges_flag_[1] = 0;

	//Low frequency timer loop for pulish debug message
	debug_loop_timer_ = main_nh.createTimer(ros::Duration(0.1), &EdgePointDetector::debugMessageLoop, this);
}

EdgePointDetector::~EdgePointDetector() {
	ros::shutdown();
}

void EdgePointDetector::readParams(ros::NodeHandle node_handle_){
//	node_handle_.param<double>("edge_detection_angle_threshold", edge_detection_angle_thresh_ , 0.0873);
	node_handle_.param<double>("ring_radious", ring_radious_ , 1.273);
//	node_handle_.param<double>("self_ignore_thickness", self_ignore_thickness_, 0.15);
	node_handle_.param<double>("floor_thickness", floor_thickness_, 0.025);
	node_handle_.param<double>("init_pose_x", init_pose_x_, -0.9);
	node_handle_.param<double>("init_pose_y", init_pose_y_,  0.0);

	node_handle_.param<std::string>("robot_center_tf", robot_center_tf_, "body_link1");
	node_handle_.param<std::string>("robot_floor_tf", robot_floor_tf_, "ground_imu_link");
	node_handle_.param<std::string>("odom_tf", odom_tf_, "odom");
	node_handle_.param<std::string>("ring_tf", ring_tf_out_name_, "ring_center");

	//	node_handle_.param<int>("floor_block_num", floor_block_num_, 30);
	node_handle_.param<int>("window_size", window_size_, 30);
	node_handle_.param<int>("center_ignore_count",center_ignore_count_, 30);
	node_handle_.param<int>("floor_count_threshold",floor_count_threshold_, 3);

}

void EdgePointDetector::initSubscriber(ros::NodeHandle node_handle_){
//	init_flag_sub_ = node_handle_.subscribe("ring_init_in", 1,&EdgePointDetector::getInitFlagCallback, this);
	right_urg_sub_ = node_handle_.subscribe("right_scan", 1,&EdgePointDetector::getRightUrgCallback, this);
	left_urg_sub_ = node_handle_.subscribe("left_scan", 1,&EdgePointDetector::getLeftUrgCallback, this);
}

void EdgePointDetector::initPublisher(ros::NodeHandle node_handle_){
	right_poses_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("right_poses", 1);
	left_poses_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("left_poses", 1);
	edge_poses_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("edge_poses", 10);
	right_center_pub_ = node_handle_.advertise<geometry_msgs::PointStamped>("right_center", 1);
	left_center_pub_ = node_handle_.advertise<geometry_msgs::PointStamped>("left_center", 1);
}
void EdgePointDetector::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	//init_flag = msg -> data;
}

void EdgePointDetector::getRightUrgCallback(const sensor_msgs::LaserScan& msg){
	sensor_msgs::LaserScan laserscan_data = msg;
	int points_num_total = laserscan_data.ranges.size();
	for (int i = 0; i < points_num_total; i++) {
		if (laserscan_data.ranges[i] > (laserscan_data.range_min + 0.01)
				&& laserscan_data.range_max > (laserscan_data.ranges[i] + 0.01)
				) {
//			laserscan_data.ranges[i] = laserscan_data.range_max; //無効なデータは最遠点に飛ばす
		}else{
			laserscan_data.ranges[i] = laserscan_data.range_max; //無効なデータは最遠点に飛ばす
		}
	}
	right_scan_ = laserscan_data;
	right_scan_ready_ = true;
}

void EdgePointDetector::getLeftUrgCallback(const sensor_msgs::LaserScan& msg){
	sensor_msgs::LaserScan laserscan_data = msg;
	int points_num_total = laserscan_data.ranges.size();
	for (int i = 0; i < points_num_total; i++) {
		if (laserscan_data.ranges[i] > (laserscan_data.range_min + 0.01)
				&& laserscan_data.range_max > (laserscan_data.ranges[i] + 0.01)
			) {
//			laserscan_data.ranges[i] = laserscan_data.range_max; //無効なデータは最遠点に飛ばす
		}else{
			laserscan_data.ranges[i] = laserscan_data.range_max; //無効なデータは最遠点に飛ばす
		}
	}
	left_scan_ = laserscan_data;
	left_scan_ready_ = true;
}

void EdgePointDetector::getLaserscanPoses(
		const sensor_msgs::LaserScan laserscan_in,
		geometry_msgs::TransformStamped odomToUrgTF,
		geometry_msgs::PoseArray& poses_out) {
	tf2::Vector3 offset;
	tf2::Matrix3x3 rotationMatrix;
	tfToOffsetAndRotationMatrix(odomToUrgTF,offset,rotationMatrix);
	double angle_min = laserscan_in.angle_min;
	double angle_max = laserscan_in.angle_max;
	double angle_increment = laserscan_in.angle_increment;
	int points_num_total = laserscan_in.ranges.size();
	poses_out.poses.resize(points_num_total);
	poses_out.header.frame_id = odomToUrgTF.header.frame_id;
	poses_out.header.stamp = ros::Time::now();
	geometry_msgs::Pose pose_tmp;
	for(int i; i < points_num_total; i++){
		tf2::Vector3 raw_point, transformed_point;
		double angle = angle_min + i * angle_increment;
		raw_point.setX(cos(angle) * laserscan_in.ranges[i]);
		raw_point.setY(sin(angle) * laserscan_in.ranges[i]);
		raw_point.setZ(0);
		transformPoint(raw_point, offset, rotationMatrix, transformed_point);
		pose_tmp.position.x = transformed_point.getX();
		pose_tmp.position.y = transformed_point.getY();
		pose_tmp.position.z = transformed_point.getZ();
		if (laserscan_in.ranges[i] > laserscan_in.range_max - 0.1){
			pose_tmp.position.z = -1.0;
		}

		pose_tmp.orientation.x = 0;
		pose_tmp.orientation.y = 0;
		pose_tmp.orientation.z = 0;
		pose_tmp.orientation.w = 1;

		poses_out.poses[i]=pose_tmp;
	}
}

void EdgePointDetector::getEdgePoses(
		const sensor_msgs::LaserScan laserscan_in,
		const geometry_msgs::PoseArray laserscan_poses_in,
		const int center_cont,
		const double urg_pitch,
		const geometry_msgs::TransformStamped odomToUrgTF,
		geometry_msgs::PoseArray& poses_out) {
	int points_num_total = laserscan_poses_in.poses.size();
	//前半データ
	std::vector<int> edge_counts_tmp;
	edge_counts_tmp.resize(2);
	int edge_detected = 0;
	std::vector<float> poses_z_tmp;
	poses_z_tmp.resize(window_size_);
	if(center_cont >= (center_ignore_count_ + window_size_) ){
		for(int i = window_size_-1; i > 0; i--){
			poses_z_tmp[i] = laserscan_poses_in.poses[window_size_-1 - i].position.z;
		}
		int floor_count = 0, edge_num = 0;
		if(urg_pitch > 0.0){
			for(int i = window_size_ - 1; i < center_cont - center_ignore_count_; i++){
				poses_z_tmp[0] = laserscan_poses_in.poses[i + window_size_].position.z;
				floor_count = 0;
				for(int j = window_size_ - 1; j >= 0 ; j--){
					if(fabs(poses_z_tmp[j]) < floor_thickness_){
						if(floor_count == 0){
							edge_num = (i + window_size_) - j;
						}
						floor_count ++;
					}
				}
				if(floor_count >= floor_count_threshold_){
					edge_counts_tmp[edge_detected] = edge_num;// i + ( window_size_ / 2);
					edge_detected ++;
					i = center_cont; //for i 終了
				}
				for(int j = window_size_-1; j > 0 ; j--){
					poses_z_tmp[j] = poses_z_tmp[j-1];
				}
			}
		}
	}


	//後半データ
//	counter_start = points_num_total;
//	counter_end = center_cont;
//	if( (counter_start - counter_end) >= (center_ignore_count_ + window_size_) ){
//		counter_start -= (center_ignore_count_ + window_size_);
//		for(int i =  0; i < window_size_; i++){
//			poses_z_tmp[window_size_ - 1 - i] = laserscan_poses_in.poses[counter_start - i].position.z;
//		}
//		int floor_count = 0;
//		for(int i = counter_start - 1 ; i > counter_end - 1; i-- ){
//			poses_z_tmp[0] = laserscan_poses_in.poses[i].position.z;
//			floor_count = 0;
//			for(int j = 0; i < window_size_; i++){
//				if(fabs(poses_z_tmp[j]) < floor_thickness_){
//					floor_count ++;
//				}
//			}
//			if(floor_count >= floor_count_threshold_){
//				edge_counts_tmp[edge_detected] = i + ( window_size_ / 2);
//				edge_detected ++;
//				i = counter_end; //for i 終了
//			}
//			for(int j = window_size_-1; i > 0 ; i--){
//				poses_z_tmp[j] = poses_z_tmp[j+1];
//			}
//		}
//	}

	if(edge_detected > 0){
		poses_out.header.frame_id = odomToUrgTF.header.frame_id;
		poses_out.header.stamp = ros::Time::now();
		poses_out.poses.resize(edge_detected);
		for(int i =0 ; i < edge_detected; i++ ){
			poses_out.poses[i] =  laserscan_poses_in.poses[ edge_counts_tmp[i] ];
		}
	}
}

int EdgePointDetector::getLaserscanCenterCount(
		const sensor_msgs::LaserScan laserscan_in,
		geometry_msgs::TransformStamped odomToUrgTF,
		geometry_msgs::PointStamped& center_pose_out,
		double& urg_pitch_out) {
	tf2::Vector3 offset;
	tf2::Matrix3x3 odom_to_urg_rotation_matrix;
	tfToOffsetAndRotationMatrix(odomToUrgTF, offset, odom_to_urg_rotation_matrix);
	tf2::Vector3 vertical_minus_on_odom(0, 0, -1.0);
	tf2::Vector3 center_direction_on_urg = odom_to_urg_rotation_matrix.inverse() * vertical_minus_on_odom;
	center_pose_out.header.frame_id = laserscan_in.header.frame_id;
	center_pose_out.header.stamp = ros::Time::now();
	double x_tmp = center_direction_on_urg.getX();
	double y_tmp = center_direction_on_urg.getY();
	double z_tmp = center_direction_on_urg.getZ();
	double theta_tmp;

	double angle_min = laserscan_in.angle_min;
	double angle_max = laserscan_in.angle_max;
	double angle_increment = laserscan_in.angle_increment;

	if(x_tmp > 0.0001){
		theta_tmp = atan2(y_tmp, x_tmp);
	}else if(x_tmp < -0.0001){
		theta_tmp = atan2(y_tmp, -x_tmp);
	}else{
		theta_tmp = (laserscan_in.angle_min + laserscan_in.angle_max) * 0.5;
	}
	int angle_count = (int)round((theta_tmp - laserscan_in.angle_min ) / angle_increment);

	double laser_length =  laserscan_in.ranges[angle_count];
	center_pose_out.point.x = laser_length * cos(theta_tmp);
	center_pose_out.point.y = laser_length * sin(theta_tmp);
	center_pose_out.point.z = 0; //center_direction_on_urg.getZ();

	urg_pitch_out = x_tmp; //水平より下向きが正、上向きが負(ピッチ軸と同じ極性)
	return angle_count ;
}

void EdgePointDetector::tfToOffsetAndRotationMatrix(
		geometry_msgs::TransformStamped tf_in,
		tf2::Vector3& offset_out,
		tf2::Matrix3x3& rotation_matrix_out) {
	offset_out.setX(tf_in.transform.translation.x);
	offset_out.setY(tf_in.transform.translation.y);
	offset_out.setZ(tf_in.transform.translation.z);
	tf2::Quaternion quaternion;
	quaternion.setX(tf_in.transform.rotation.x);
	quaternion.setY(tf_in.transform.rotation.y);
	quaternion.setZ(tf_in.transform.rotation.z);
	quaternion.setW(tf_in.transform.rotation.w);
	rotation_matrix_out.setRotation(quaternion);
}

void EdgePointDetector::transformPoint(
		tf2::Vector3 point_in,
		tf2::Vector3 offset_in,
		tf2::Matrix3x3 rotation_matrix_in,
		tf2::Vector3& point_out) {
	tf2::Vector3 xyz_rot = rotation_matrix_in*point_in;
	point_out.setX(offset_in.getX()+xyz_rot.getX());
	point_out.setY(offset_in.getY()+xyz_rot.getY());
	point_out.setZ(offset_in.getZ()+xyz_rot.getZ());
}

int EdgePointDetector::mainLoop(){
	//wait for topics
	while (false == right_scan_ready_ || false == left_scan_ready_){
		ros::spinOnce();
	}
	//wait for TF
	right_tf_ready_ = tfBuffer_ptr_->canTransform(right_scan_.header.frame_id, odom_tf_, ros::Time(0));
	while (false == right_tf_ready_){
		right_tf_ready_ = tfBuffer_ptr_->canTransform(right_scan_.header.frame_id, odom_tf_, ros::Time::now(), ros::Duration(1.0));
		ROS_FATAL("TF not found : %s -> %s", odom_tf_.c_str(), right_scan_.header.frame_id.c_str());
	}
	left_tf_ready_ = tfBuffer_ptr_->canTransform(left_scan_.header.frame_id, odom_tf_, ros::Time(0));
	while (false == left_tf_ready_){
		left_tf_ready_ = tfBuffer_ptr_->canTransform(left_scan_.header.frame_id, odom_tf_, ros::Time::now(), ros::Duration(1.0));
		ROS_FATAL("TF not found : %s -> %s", odom_tf_.c_str(), left_scan_.header.frame_id.c_str());
	}

	right_tf_ = tfBuffer_ptr_->lookupTransform( odom_tf_ , right_scan_.header.frame_id ,ros::Time(0));
	left_tf_ = tfBuffer_ptr_->lookupTransform( odom_tf_ , left_scan_.header.frame_id ,ros::Time(0));

	//Start calculation
	ros::spinOnce();

	getLaserscanPoses(	right_scan_,right_tf_,right_poses_);
	getLaserscanPoses(left_scan_,left_tf_,left_poses_);
	int right_center_count, left_center_count;
	right_center_count = getLaserscanCenterCount(right_scan_, right_tf_, right_center_,right_pitch_);
//	left_center_count = getLaserscanCenterCount(left_scan_, left_tf_, left_center_);
	getEdgePoses(right_scan_,right_poses_, right_center_count,right_pitch_ , right_tf_, right_edges_);

//	getEdgePoses(left_poses_, left_center_count, left_tf_, left_edges_);
	//	getLaserscanCenterCount(right_scan_, right_tf_, right_center_);
	//	getLaserscanCenterCount(left_scan_, left_tf_, left_center_);
	//finish function
	right_scan_ready_ = false;
	left_scan_ready_ = false;
	right_tf_ready_ = false;
	left_tf_ready_ = false;

	return true;
}

void EdgePointDetector::debugMessageLoop(const ros::TimerEvent&){
	if(right_poses_.poses.size() != 0){
		right_poses_pub_.publish(right_poses_);
	}
	if(left_poses_.poses.size() != 0){
		left_poses_pub_.publish(left_poses_);
	}
	right_center_pub_.publish(right_center_);
	left_center_pub_.publish(left_center_);
	if(right_edges_.poses.size() > 0){
		edge_poses_pub_.publish(right_edges_);
		right_edges_.poses.resize(0);
	}
}

