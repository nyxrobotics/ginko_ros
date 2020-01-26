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
	node_handle_.param<double>("edge_detection_angle_threshold", edge_detection_angle_thresh_ , 0.0873);
	node_handle_.param<double>("ring_radious", ring_radious_ , 1.273);
	node_handle_.param<double>("self_ignore_thickness", self_ignore_thickness_, 0.15);
	node_handle_.param<double>("init_pose_x", init_pose_x_, -0.9);
	node_handle_.param<double>("init_pose_y", init_pose_y_,  0.0);

	node_handle_.param<std::string>("robot_center_tf", robot_center_tf_, "body_link1");
	node_handle_.param<std::string>("robot_floor_tf", robot_floor_tf_, "ground_imu_link");
	node_handle_.param<std::string>("odom_tf", odom_tf_, "odom");
	node_handle_.param<std::string>("ring_tf", ring_tf_out_name_, "ring_center");

	node_handle_.param<int>("floor_block_num", floor_block_num_, 30);

}

void EdgePointDetector::initSubscriber(ros::NodeHandle node_handle_){
//	init_flag_sub_ = node_handle_.subscribe("ring_init_in", 1,&EdgePointDetector::getInitFlagCallback, this);
	right_urg_sub_ = node_handle_.subscribe("right_scan", 1,&EdgePointDetector::getRightUrgCallback, this);
	left_urg_sub_ = node_handle_.subscribe("left_scan", 1,&EdgePointDetector::getLeftUrgCallback, this);
}

void EdgePointDetector::initPublisher(ros::NodeHandle node_handle_){
	right_edge_marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("right_edge_marker", 1);
	left_edge_marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("left_edge_marker", 1);
}
void EdgePointDetector::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	//init_flag = msg -> data;
}
void EdgePointDetector::getRightUrgCallback(const sensor_msgs::LaserScan& msg){
	sensor_msgs::LaserScan laserscan_data = msg;
	right_scan_ = msg;
	right_scan_ready_ = true;
}
void EdgePointDetector::getLeftUrgCallback(const sensor_msgs::LaserScan& msg){
	sensor_msgs::LaserScan laserscan_data = msg;
	left_scan_ = msg;
	left_scan_ready_ = true;
}
void EdgePointDetector::detectEdge(
		const sensor_msgs::LaserScan laserscan_in,
		geometry_msgs::TransformStamped odomToUrgTF,
		visualization_msgs::MarkerArray& markerarray_out) {
	std::string parent_tf_name = laserscan_in.header.frame_id;
	double angle_min = laserscan_in.angle_min;
	double angle_max = laserscan_in.angle_max;
	double angle_increment = laserscan_in.angle_increment;
	int points_num_total = laserscan_in.ranges.size();

	markerarray_out.markers.resize(points_num_total);

	for (int i = 0; i < points_num_total; i++) {
		if(laserscan_in.ranges[i] > laserscan_in.range_min + 0.001
				&& laserscan_in.range_max > laserscan_in.ranges[i] + 0.1){
			geometry_msgs::Point start_point, end_pont;
			start_point.x = 0;
			start_point.y = 0;
			start_point.z = 0;
			double angle = angle_min + i * angle_increment;
			start_point.x = cos(angle) * laserscan_in.ranges[i];
			start_point.y = sin(angle) * laserscan_in.ranges[i];
			start_point.z = 0;

			geometry_msgs::Vector3 arrow_shape;  //config arrow shape
			arrow_shape.x = 0.002;
			arrow_shape.y = 0.004;
			arrow_shape.z = 0.01;

			markerarray_out.markers[i].header.frame_id = parent_tf_name;
			markerarray_out.markers[i].header.stamp = ros::Time::now();
			markerarray_out.markers[i].ns = "laserscan_num_" + std::to_string(i); //std::to_string(i);
			markerarray_out.markers[i].id = i;
			markerarray_out.markers[i].lifetime = ros::Duration();

			markerarray_out.markers[i].type = visualization_msgs::Marker::ARROW;
			markerarray_out.markers[i].action = visualization_msgs::Marker::ADD;
			markerarray_out.markers[i].scale = arrow_shape;

			markerarray_out.markers[i].points.resize(2);
			markerarray_out.markers[i].points[0] = start_point;
			markerarray_out.markers[i].points[1] = end_pont;

			markerarray_out.markers[i].color.r = 0.0f;
			markerarray_out.markers[i].color.g = 1.0f;
			markerarray_out.markers[i].color.b = 0.0f;
			markerarray_out.markers[i].color.a = 1.0f;
		}else{
			geometry_msgs::Point start_point, end_pont;
			start_point.x = 0;
			start_point.y = 0;
			start_point.z = 0;
			double angle = angle_min + i * angle_increment;
			start_point.x = cos(angle) * laserscan_in.range_max;
			start_point.y = sin(angle) * laserscan_in.range_max;
			start_point.z = 0;
			geometry_msgs::Vector3 arrow_shape;  //config arrow shape
			arrow_shape.x = 0.002;
			arrow_shape.y = 0.004;
			arrow_shape.z = 0.01;

			markerarray_out.markers[i].header.frame_id = parent_tf_name;
			markerarray_out.markers[i].header.stamp = ros::Time::now();
			markerarray_out.markers[i].ns = "laserscan_num_" + std::to_string(i);
			markerarray_out.markers[i].id = i;
			markerarray_out.markers[i].lifetime = ros::Duration();

			markerarray_out.markers[i].type = visualization_msgs::Marker::ARROW;
			markerarray_out.markers[i].action = visualization_msgs::Marker::ADD;
			markerarray_out.markers[i].scale = arrow_shape;

			markerarray_out.markers[i].points.resize(2);
			markerarray_out.markers[i].points[0] = start_point;
			markerarray_out.markers[i].points[1] = end_pont;

			markerarray_out.markers[i].color.r = 0.0f;
			markerarray_out.markers[i].color.g = 1.0f;
			markerarray_out.markers[i].color.b = 0.0f;
			markerarray_out.markers[i].color.a = 1.0f;
		}
	}
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

	detectEdge(right_scan_, right_tf_, right_edge_marker_array_);
	detectEdge(left_scan_, left_tf_, left_edge_marker_array_);

	//finish function
	right_scan_ready_ = false;
	left_scan_ready_ = false;
	right_tf_ready_ = false;
	left_tf_ready_ = false;

	return true;
}
void EdgePointDetector::debugMessageLoop(const ros::TimerEvent&){
	if(right_edge_marker_array_.markers.size() != 0){
		right_edge_marker_pub_.publish(right_edge_marker_array_);
	}
	if(left_edge_marker_array_.markers.size() != 0){
		left_edge_marker_pub_.publish(left_edge_marker_array_);
	}
}

