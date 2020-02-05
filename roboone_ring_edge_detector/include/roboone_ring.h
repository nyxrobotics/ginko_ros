#ifndef ROBOONE_RING_H
#define ROBOONE_RING_H

#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <iostream>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <boost/bind.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"

class RobooneRing {
private:
	//動作:
	//初期化：最初の位置(固定値)にロボットが来るようにTF staticを吐く
	//ring座標から見たロボット位置を受け取った時(実装予定)：

	//初期化
	int init_flag = 1; //最初は一回初期化する
	ros::Subscriber init_flag_sub_;


	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster static_broadcaster;


	//入力
	std::string foots_center_tf_in_name_	= "ground_foot_center_link";	//ジャイロのTF
	std::string odom_tf_in_name_			= "odom";	//ジャイロの回転の基準になっているTF
	//出力
	std::string ring_tf_out_name_			= "ring";		//ジャイロの位置、ロボット前方をx軸とする床に水平なTF



public:
	RobooneRing(ros::NodeHandle main_nh);
	~RobooneRing();
	void readParams(ros::NodeHandle node_handle_);
	int ringMainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);

	void getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg);

};














#endif //ROBOONE_RING_H
