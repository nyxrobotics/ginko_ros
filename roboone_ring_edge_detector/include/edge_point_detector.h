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
//extract laserscan into pointcloud
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
//show urg center arrow
#include <geometry_msgs/PointStamped.h>

class EdgePointDetector {
private:
	//動作:
	//初期化：最初の位置(固定値)にロボットが来るようにTF staticを吐く
	//ring座標から見たロボット位置を受け取った時(実装予定)：

	//初期化
	ros::Subscriber right_urg_sub_;
	ros::Subscriber left_urg_sub_;

	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr_;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr_;

	//内部処理用変数
	std::vector<tf2::Vector3> right_detected_edges_;
	std::vector<tf2::Vector3> left_detected_edges_;
	std::vector<int> right_detected_edges_flag_;
	std::vector<int> left_detected_edges_flag_;

	sensor_msgs::LaserScan right_scan_;
	geometry_msgs::TransformStamped right_tf_;
	bool right_scan_ready_ = false, right_tf_ready_ = false;
	sensor_msgs::LaserScan left_scan_;
	geometry_msgs::TransformStamped left_tf_;
	bool left_scan_ready_ = false, left_tf_ready_ = false;
	//LaserScan into PointCloud2
//	laser_geometry::LaserProjection laser_projector_;
//	sensor_msgs::PointCloud2 right_cloud_, left_cloud_;

	//ROS Parameters
//	double edge_detection_angle_thresh_ = 0.0873; //[rad] エッジ検出をする際に、URGの距離データが空の部分を「データ抜け」/「無限遠」のどちらかを判別するしきい値
	double ring_radious_ = 1.273; //[m] リングのサイズ(予選：1.273、本戦：1.8)
//	double self_ignore_thickness_ = 0.15; //[m] ロボット周辺の無視する半径。不要かも。
	double floor_thickness_ = 0.025; //[m] 床判定の厚み。センサの誤差に合わせて調整
	double init_pose_x_ = -0.9; //[m] 初期位置x成分(リング中心→ロボット位置)
	double init_pose_y_ =  0.0; //[m] 初期位置y成分(リング中心→ロボット位置)
//	int floor_block_num_;
	int window_size_, floor_count_threshold_, center_ignore_count_;
	int bottom_count_threshold_ = 10;

	//入力
	std::string robot_center_tf_ = "body_link1"; //ロボットの中心として計算するTF。主に太ももの付け根(胴体側)。
	std::string robot_floor_tf_ = "ground_imu_link";
	std::string odom_tf_ = "odom"; //床位置のリンク
	//出力
	std::string ring_tf_out_name_	 = "ring_center";
	//デバッグ用変数
	ros::Timer debug_loop_timer_;
	ros::Publisher right_center_pub_, left_center_pub_;
	geometry_msgs::PointStamped right_center_, left_center_;
    geometry_msgs::PoseArray right_edges_, left_edges_, merged_edges_;
    double right_pitch_,left_pitch_;
	//エッジ点表示用
    geometry_msgs::PoseArray right_poses_, left_poses_;
	ros::Publisher right_poses_pub_, left_poses_pub_, right_edges_pub_, left_edges_pub_, merged_edges_pub_;
	bool left_poses_ready_ = false, right_poses_ready_ = false;

public:
	EdgePointDetector(ros::NodeHandle main_nh);
	~EdgePointDetector();
	void readParams(ros::NodeHandle node_handle_);
	int mainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);

	void getRightUrgCallback(const sensor_msgs::LaserScan& msg);
	void getLeftUrgCallback(const sensor_msgs::LaserScan& msg);

	void getLaserscanPoses(
			const sensor_msgs::LaserScan laserscan_in,
			geometry_msgs::TransformStamped odomToUrgTF,
			geometry_msgs::PoseArray& poses_out);
	void getEdgePoses(
			const sensor_msgs::LaserScan laserscan_in,
			const geometry_msgs::PoseArray laserscan_poses_in,
			const int center_cont,
			const double urg_pitch,
			const geometry_msgs::TransformStamped odomToUrgTF,
			geometry_msgs::PoseArray& poses_out) ;
	void mergeEdges(
			const geometry_msgs::PoseArray right_poses_in,
			const geometry_msgs::PoseArray left_poses_in,
			geometry_msgs::PoseArray& poses_out);
	int getLaserscanCenterCount(
			const sensor_msgs::LaserScan laserscan_in,
			geometry_msgs::TransformStamped odomToUrgTF,
			geometry_msgs::PointStamped& center_pose_out,
			double& urg_pitch_out);
	void tfToOffsetAndRotationMatrix(
			geometry_msgs::TransformStamped tf_in,
			tf2::Vector3& offset_out,
			tf2::Matrix3x3& rotation_matrix_out);
	void transformPoint(
			tf2::Vector3 point_in,
			tf2::Vector3 offset_in,
			tf2::Matrix3x3 rotation_matrix_in,
			tf2::Vector3& point_out);
	void debugMessageLoop(const ros::TimerEvent&);
};



#endif //EDGE_POINT_DETECTOR_H
