#include <ring_tf_publisher.h>

RingTfPublisher::RingTfPublisher(ros::NodeHandle main_nh){
	readParams(main_nh);
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr_.reset(new tf2_ros::Buffer(ros::Duration(10.0), false));
	tfListener_ptr_.reset(new tf2_ros::TransformListener(*tfBuffer_ptr_));
	sleep(1);//TFが安定するまで待つ(ないと落ちる?要検証)
	initSubscriber(main_nh);
//	initPublisher(main_nh);
	//wait for TF
	bool tf_ready_ = tfBuffer_ptr_->canTransform(odom_tf_name_, robot_tf_name_, ros::Time(0));
	while (false == tf_ready_){
		tf_ready_ = tfBuffer_ptr_->canTransform(odom_tf_name_, robot_tf_name_, ros::Time::now(), ros::Duration(1.0));
		ROS_INFO("TF not found : %s -> %s", odom_tf_name_.c_str(), robot_tf_name_.c_str());
	}
	odom_to_robot_tf_ = tfBuffer_ptr_->lookupTransform(odom_tf_name_, robot_tf_name_, ros::Time(0));
	//initialize vectors
	odom_to_ring_tf_lpf_ = odom_to_robot_tf_;
	odom_to_ring_tf_lpf_.child_frame_id = "ring_center";
	tf_offset_buffer_.resize(median_num_);
	for(int i = 0; i<median_num_;i++){
		tf_offset_buffer_[i].setX(odom_to_ring_tf_lpf_.transform.translation.x);
		tf_offset_buffer_[i].setY(odom_to_ring_tf_lpf_.transform.translation.y);
		tf_offset_buffer_[i].setZ(odom_to_ring_tf_lpf_.transform.translation.z);
	}

}

RingTfPublisher::~RingTfPublisher() {
	ros::shutdown();
}

void RingTfPublisher::readParams(ros::NodeHandle node_handle_){
	node_handle_.param<int>("median_num", median_num_, 5);
	node_handle_.param<double>("lpf_constant", lpf_constant_, 0.001);
	node_handle_.param<std::string>("robot_tf_name", robot_tf_name_, "ground_imu_link");
	node_handle_.param<std::string>("odom_tf_name", odom_tf_name_, "odom");
}

void RingTfPublisher::initSubscriber(ros::NodeHandle node_handle_){
	 init_flag_sub_ = node_handle_.subscribe("init_flag", 1,&RingTfPublisher::getInitFlagCallback, this);
	right_edges_sub_ = node_handle_.subscribe("right_edges", 1,&RingTfPublisher::getRightEdgeCallback, this);
//	left_edges_sub_ = node_handle_.subscribe("left_edges", 1,&RingTfPublisher::getLeftEdgeCallback, this);
}

void RingTfPublisher::initPublisher(ros::NodeHandle node_handle_){

}

void RingTfPublisher::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	init_flag_ = msg->data;
}
void RingTfPublisher::getRightEdgeCallback(const geometry_msgs::PoseArray& msg){
	right_edges_ = msg;
	if(right_edges_.poses.size() == 2){
		tf2::Vector3 start_point, end_point, center_point;
		tf2::Vector3 start_to_end_vector;
		start_point.setX(right_edges_.poses[0].position.x);
		start_point.setY(right_edges_.poses[0].position.y);
		start_point.setZ(right_edges_.poses[0].position.z);
		end_point.setX(right_edges_.poses[1].position.x);
		end_point.setY(right_edges_.poses[1].position.y);
		end_point.setZ(right_edges_.poses[1].position.z);
		center_point = (start_point + end_point)*0.5;
		start_to_end_vector = end_point - start_point;
		start_to_end_vector.setZ(0);
		double theta_tmp = atan2(start_to_end_vector.getY(), start_to_end_vector.getX());
		tf2::Quaternion quaternion_tmp(0,0,theta_tmp);
		tf2::Matrix3x3 rotation_matrix(quaternion_tmp);
		tf2::Vector3 ring_to_line_offset  = center_point - tf_offset_lpf_;
		tf2::Vector3 ring_to_line_offset_rotated = rotation_matrix.inverse()*ring_to_line_offset;
		ring_to_line_offset_rotated.setY(0);
		ring_to_line_offset = rotation_matrix*ring_to_line_offset_rotated;
		for(int i = median_num_ - 1; i>0; i--){
			tf_offset_buffer_[i] = tf_offset_buffer_[i-1];
		}
		tf_offset_buffer_[0] = tf_offset_lpf_ + ring_to_line_offset;
	}
}
void RingTfPublisher::getLeftEdgeCallback(const geometry_msgs::PoseArray& msg){
	left_edges_ = msg;
	if(left_edges_.poses.size() == 2){
		tf2::Vector3 start_point, end_point, center_point;
		tf2::Vector3 start_to_end_vector;
		start_point.setX(left_edges_.poses[0].position.x);
		start_point.setY(left_edges_.poses[0].position.y);
		start_point.setZ(left_edges_.poses[0].position.z);
		end_point.setX(left_edges_.poses[1].position.x);
		end_point.setY(left_edges_.poses[1].position.y);
		end_point.setZ(left_edges_.poses[1].position.z);
		center_point = (start_point + end_point)*0.5;
		start_to_end_vector = end_point - start_point;
		start_to_end_vector.setZ(0);
		double theta_tmp = atan2(start_to_end_vector.getY(), start_to_end_vector.getX());
		tf2::Quaternion quaternion_tmp(theta_tmp,0,0);
		tf2::Matrix3x3 rotation_matrix(quaternion_tmp);
		tf2::Vector3 ring_to_line_offset  = center_point - tf_offset_lpf_;
		tf2::Vector3 ring_to_line_offset_rotated = rotation_matrix.inverse()*ring_to_line_offset;
		ring_to_line_offset_rotated.setY(0);
		ring_to_line_offset = rotation_matrix*ring_to_line_offset_rotated;
		for(int i = median_num_ - 1; i>0; i--){
			tf_offset_buffer_[i] = tf_offset_buffer_[i-1];
		}
		tf_offset_buffer_[0] = tf_offset_lpf_ + ring_to_line_offset;
	}
}

int RingTfPublisher::mainLoop(){
	odom_to_ring_tf_lpf_.header.stamp = ros::Time::now();
	odom_to_ring_tf_lpf_.child_frame_id = "ring_center";
	odom_to_ring_tf_lpf_.transform.rotation.x=0;
	odom_to_ring_tf_lpf_.transform.rotation.y=0;
	odom_to_ring_tf_lpf_.transform.rotation.z=0;
	odom_to_ring_tf_lpf_.transform.rotation.w=1.0;
	tf_offset_lpf_ = tf_offset_buffer_[0];
	odom_to_ring_tf_lpf_.transform.translation.x = tf_offset_lpf_.getX();
	odom_to_ring_tf_lpf_.transform.translation.y = tf_offset_lpf_.getY();
	odom_to_ring_tf_lpf_.transform.translation.z = tf_offset_lpf_.getZ();
	tfBroadcaster_.sendTransform(odom_to_ring_tf_lpf_);
	return true;
}
