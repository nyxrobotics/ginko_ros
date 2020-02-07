#ifndef GINKO_OFFSETS_H
#define GINKO_OFFSETS_H

#include <ros/ros.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#define JOINT_BUFFER_NUM 20

//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseStamped.h"
//detect robot falling
#include <sensor_msgs/Imu.h>
class UrgNearest {
private:
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	// ROS Topic Subscriber
	ros::Subscriber urg_sub_;

	//股関節の補正のためTF2を使う
	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster staticBroadcaster;

	//入力
	std::string urg_tf_in_name_	 = "urg_link";
	std::string odom_tf_in_name_ = "odom";
	//出力
	std::string detected_tf_out_name_ = "target";

	bool urg_ready = 0;
	bool urg_updated = 0;
	bool tf2_ready = 0;
	sensor_msgs::LaserScan scan_in_;
	tf2::Vector3 urg_to_nearest_;
	double nearest_distance_;
	double nearest_max_ = 3.6;
	ros::Publisher target_pose_pub_;
	//転倒検知
	ros::Subscriber imu_quaternion_sub_;
	sensor_msgs::Imu imu_quaternion_;
	int imu_ready_ = 0;
	int imu_fall_direction_ = 0;//0:直立、1:前転倒、2:後転倒

public:
	UrgNearest(ros::NodeHandle main_nh);
	~UrgNearest();
	int mainLoop();

private:
	void initPublisher();
	void initSubscriber();
	void initTF2();
	void readParams(ros::NodeHandle main_nh);
	void getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
};


#endif //GINKO_OFFSETS_H
