#include <edge_point_detector.h>

//左右の腕のURGを受け取り、それぞれ前後

//EdgePointDetector here
EdgePointDetector::EdgePointDetector(ros::NodeHandle main_nh){
	readParams(main_nh);
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(10.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	//sleep(2);//TFが安定するまで待つ(ないと落ちる。良くわからない)
//	tfBuffer_ptr->canTransform(robot_center_tf_,odom_tf_, ros::Time::now(), ros::Duration(10.0));
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

}

void EdgePointDetector::initSubscriber(ros::NodeHandle node_handle_){
//	init_flag_sub_ = node_handle_.subscribe("ring_init_in", 1,&EdgePointDetector::getInitFlagCallback, this);
	right_urg_sub_ = node_handle_.subscribe("right_scan", 1,&EdgePointDetector::getRightUrgCallback, this);
	left_urg_sub_ = node_handle_.subscribe("left_scan", 1,&EdgePointDetector::getLeftUrgCallback, this);
}

void EdgePointDetector::initPublisher(ros::NodeHandle node_handle_){
	marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

}

int EdgePointDetector::ringMainLoop(){

	return 0;
}

void EdgePointDetector::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	//init_flag = msg -> data;
}
void EdgePointDetector::getRightUrgCallback(const sensor_msgs::LaserScan& msg){
	sensor_msgs::LaserScan laserscan_data = msg;
	detectEdge(laserscan_data);
}
void EdgePointDetector::getLeftUrgCallback(const sensor_msgs::LaserScan& msg){

}
void EdgePointDetector::detectEdge(const sensor_msgs::LaserScan laserscan_in){
	std::string parent_tf_name = laserscan_in.header.frame_id;
	double angle_min = laserscan_in.angle_min;
	double angle_max = laserscan_in.angle_max;
	double angle_increment = laserscan_in.angle_increment;
	int points_num_total = laserscan_in.ranges.size();

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(points_num_total);
	for(int i=0;i<points_num_total;i++){
		geometry_msgs::Point start_point,end_pont;
		start_point.x = 0;
		start_point.y = 0;
		start_point.z = 0;
		double angle = angle_min + i * angle_increment;
		start_point.x = cos(angle) *laserscan_in.ranges[i] ;
		start_point.y = sin(angle) *laserscan_in.ranges[i];
		start_point.z = 0;

		geometry_msgs::Vector3 arrow_shape;  //config arrow shape
		arrow_shape.x = 0.002;
		arrow_shape.y = 0.004;
		arrow_shape.z = 0.01;

	    marker_array.markers[i].header.frame_id =parent_tf_name;
	    marker_array.markers[i].header.stamp = ros::Time::now();
	    marker_array.markers[i].ns = "laserscan_num_" + std::to_string(i);
	    marker_array.markers[i].id = 0;
	    marker_array.markers[i].lifetime = ros::Duration();

	    marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
	    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
	    marker_array.markers[i].scale = arrow_shape;

	    marker_array.markers[i].points.resize(2);
	    marker_array.markers[i].points[0]=start_point;
	    marker_array.markers[i].points[1]=end_pont;

	    marker_array.markers[i].color.r = 0.0f;
	    marker_array.markers[i].color.g = 1.0f;
	    marker_array.markers[i].color.b = 0.0f;
	    marker_array.markers[i].color.a = 1.0f;
	}
	marker_pub_.publish(marker_array);
}

