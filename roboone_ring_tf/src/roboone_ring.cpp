#include <roboone_ring.h>

//RobooneRing here
RobooneRing::RobooneRing(ros::NodeHandle main_nh){
	readParams(main_nh);
	initSubscriber(main_nh);
//	initPublisher(main_nh);
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(1.0), false));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(2);//TFが安定するまで待つ(ないと落ちる。良くわからない)
	tfBuffer_ptr->lookupTransform(foots_center_tf_in_name_ ,odom_tf_in_name_, ros::Time::now(), ros::Duration(1.0));
	sleep(1);//TFが安定するまで待つ(ないとたまに起動時からずっと更新周期が低くなる。良くわからない)

}

RobooneRing::~RobooneRing() {
	ros::shutdown();
}

void RobooneRing::readParams(ros::NodeHandle node_handle_){
//	node_handle_.param<bool>("publish_debug_topics", publish_debug_topics_, true);
//	node_handle_.param<int>("drift_waiting_conter", imu_drift_waiting_counter_, 100);
}

void RobooneRing::initSubscriber(ros::NodeHandle node_handle_){
	init_flag_sub_ = node_handle_.subscribe("ring_init_in", 1,&RobooneRing::getInitFlagCallback, this);
}

void RobooneRing::initPublisher(ros::NodeHandle node_handle_){
}

int RobooneRing::ringMainLoop(){
	if(init_flag == 0){
		usleep(1000);
		return 0;
	}
	//新しいデータが来るまで待機
	//usleep(500);//不要なsleep
	geometry_msgs::TransformStamped transformStamped;
	transformStamped = tfBuffer_ptr->lookupTransform(odom_tf_in_name_, foots_center_tf_in_name_ , ros::Time::now(), ros::Duration(1.0));
	tf2::Quaternion quaternion(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);
	tf2::Vector3 ring_translation(0.9,0,0);
//	ring_translation.setX(0.9);
//	ring_translation.setY(0);
//	ring_translation.setZ(0);

	tf2::Matrix3x3 rotationalMatrix(quaternion);
	tf2::Vector3 translation_rotate = rotationalMatrix * ring_translation;
	geometry_msgs::TransformStamped static_transformStamped;

	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = odom_tf_in_name_;
	static_transformStamped.child_frame_id  = ring_tf_out_name_;
	static_transformStamped.transform.translation.x = transformStamped.transform.translation.x + translation_rotate.getX();
	static_transformStamped.transform.translation.y = transformStamped.transform.translation.y + translation_rotate.getY();
	static_transformStamped.transform.translation.z = 0.;
	static_transformStamped.transform.rotation.x = quaternion.getX();
	static_transformStamped.transform.rotation.y = quaternion.getY();
	static_transformStamped.transform.rotation.z = quaternion.getZ();
	static_transformStamped.transform.rotation.w = quaternion.getW();
	static_broadcaster.sendTransform(static_transformStamped);

	init_flag = 0;
	return 0;
}


void RobooneRing::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	init_flag = msg -> data;
}
