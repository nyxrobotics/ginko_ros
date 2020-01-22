#ifndef EDGE_POINT_DETECTOR_H
#define EDGE_POINT_DETECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
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
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

class EdgePointDetector {
private:
	//動作:
	//初期化：最初の位置(固定値)にロボットが来るようにTF staticを吐く
	//ring座標から見たロボット位置を受け取った時(実装予定)：

	//初期化
	int init_flag = 1; //最初は一回初期化する
	ros::Subscriber init_flag_sub_;
	ros::Subscriber right_urg_sub_;
	ros::Subscriber left_urg_sub_;
	ros::Publisher marker_pub_;

	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster static_broadcaster;

	//内部処理用変数
	std::vector<tf2::Vector3> right_detected_edges_;
	std::vector<tf2::Vector3> left_detected_edges_;
	std::vector<int> right_detected_edges_flag_;
	std::vector<int> left_detected_edges_flag_;
	//ROS Parameters
	double edge_detection_angle_thresh_ = 0.0873; //[rad] エッジ検出をする際に、URGの距離データが空の部分を「データ抜け」/「無限遠」のどちらかを判別するしきい値
	double ring_radious_ = 1.273; //[m] リングのサイズ(予選：1.273、本戦：1.8)
	double self_ignore_thickness_ = 0.15; //[m] ロボット周辺の無視する半径。不要かも。
	double init_pose_x_ = -0.9; //[m] 初期位置x成分(リング中心→ロボット位置)
	double init_pose_y_ =  0.0; //[m] 初期位置y成分(リング中心→ロボット位置)
	//入力
	std::string robot_center_tf_ = "body_link1"; //ロボットの中心として計算するTF。主に太ももの付け根(胴体側)。
	std::string robot_floor_tf_ = "ground_imu_link";
	std::string odom_tf_ = "odom"; //床位置のリンク
	//出力
	std::string ring_tf_out_name_	 = "ring_center";

public:
	EdgePointDetector(ros::NodeHandle main_nh);
	~EdgePointDetector();
	void readParams(ros::NodeHandle node_handle_);
	int ringMainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);

	void getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg);
	void getRightUrgCallback(const sensor_msgs::LaserScan& msg);
	void getLeftUrgCallback(const sensor_msgs::LaserScan& msg);

	void detectEdge(const sensor_msgs::LaserScan laserscan_in);

};














#endif //EDGE_POINT_DETECTOR_H
