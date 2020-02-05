#ifndef RING_TF_PUBLISHER_H
#define RING_TF_PUBLISHER_H

#include <ros/ros.h>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>
//edge messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
//init flag
#include <std_msgs/Int32.h>
//

#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

class RingTfPublisher{
private:
	//動作:
	//初期化：最初の位置(固定値)にロボットが来るようにTFを設定
	//エッジのサブスクライバ割り込み：前後でエッジが検出された場合にリングの中央を寄せるようにTFを再計算
	//main:一定周期でTFをパブリッシュ

	//初期化
	ros::Subscriber init_flag_sub_, right_edges_sub_, left_edges_sub_;
	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr_;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr_;
	tf2_ros::TransformBroadcaster tfBroadcaster_;
	tf2_ros::StaticTransformBroadcaster static_broadcaster_;

	//subscribed messages
	geometry_msgs::PoseArray right_edges_, left_edges_;
	int init_flag_ = 1; //最初は一回初期化する

	//ROS params
	int median_num_;
	double lpf_constant_;
	std::string robot_tf_name_, odom_tf_name_;
	//内部保持用
	std::vector<tf2::Vector3> tf_offset_buffer_;
	tf2::Vector3 tf_offset_median_, tf_offset_lpf_;
	geometry_msgs::TransformStamped odom_to_robot_tf_;
	//出力用
	geometry_msgs::TransformStamped odom_to_ring_tf_lpf_;

public:
	RingTfPublisher(ros::NodeHandle main_nh);
	~RingTfPublisher();
	int mainLoop();
private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);
	void readParams(ros::NodeHandle node_handle_);

	void getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg);
	void getRightEdgeCallback(const geometry_msgs::PoseArray& msg);
	void getLeftEdgeCallback(const geometry_msgs::PoseArray& msg);
};

#endif //RING_TF_PUBLISHER_H
