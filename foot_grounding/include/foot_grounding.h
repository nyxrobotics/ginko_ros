#ifndef FOOT_GROUNDING_H
#define FOOT_GROUNDING_H

#include <ros/ros.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <iostream>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <boost/bind.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

class FootGrounding {
private:
	//動作:
	//足裏8点の位置関係を見て、両足接地、片足面接地、片足エッジ接地、を判定する。(自己位置計算用)
	//Publisher:現在高さ、現在速度、TF
	//TF:自己位置増減計算の起点となる接地点、及び胴体真下のbase_linkをpublishする。
	//TF:左右の足の接地点に該当する場所、両足の中央点
	//NodeHandler,Publisher,Subscriber
	//ros::NodeHandle node_handle_;
	ros::Subscriber imu_quaternion_sub_;//足裏のどの頂点が接地しているかを判定するため、現在姿勢を受け取る。
	ros::Subscriber joint_states_sub_;	//関節角度の更新周期に合わせて再計算する。データ自体は使わない。

	ros::Publisher imu_ground_height_pub_;  //MPU6500中央からみた重力方向への床の高さ
	ros::Publisher imu_velocity_pub_;       //重力方向を鉛直、胸部前方をx軸とした時の、MPU6500中央のxyz方向の速度
	ros::Publisher r_ground_height_pub_;	//MPU6050中央から見た足先位置
	ros::Publisher r_velocity_pub_;			//MPU6050中央から見た足先速度
	ros::Publisher l_ground_height_pub_;
	ros::Publisher l_velocity_pub_;

//	tf2_ros::Buffer tfBuffer(ros::Duration(1.0), false);
//	tf2_ros::TransformListener tfListener(tfBuffer);
	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;

	//rosparam
	//TODO:パラメータにする
	std::string imu_tf_in_name_				= "body_imu_base_link";	//入力：ジャイロのTF()
	std::string imu_tf_reverse_in_name_		= "body_imu_reverse";	//入力：ジャイロの回転の基準になっているTF()
	std::string imu_tf_yaw_in_name_			= "body_imu_yaw";		//入力：ジャイロのTF()
	std::string ground_imu_tf_out_name_		= "ground_imu_link";	//出力：ジャイロを地面に投影した位置(odomを生やす基準点にする予定)
	std::string ground_point_tf_out_name_	= "ground_point_link";	//足裏の接地点
	std::string ground_center_tf_out_name_	= "ground_foot_center_link";//足裏の接地点
	std::string ground_r_tf_out_name_	= "ground_foot_r_link";		//右足裏の接地点
	std::string ground_l_tf_out_name_	= "ground_foot_l_link";		//左足裏の接地点
//	double footup_thresh_min_  = 0.001;	//どちらかの足が上がっている判定のスレッショルド
	double footup_thresh_max_  = 0.003;	//minからmaxまではground_point_linkが線形で遷移
//	double toe_edg_thresh_min_ = 0.002; //片方の足において足先エッジが上がっている判定のスレッショルド
	double toe_edg_thresh_max_ = 0.005;	//minからmaxまではground_point_linkが線形で遷移

	bool publish_debug_topic = true;
	std::string r_toe_center_tf_ = "leg_r_toe_center";
	int r_toe_tf_num = 4;
	std::string r_toe_tf_in_[4] = {
			"leg_r_toe_link0",
			"leg_r_toe_link1",
			"leg_r_toe_link2",
			"leg_r_toe_link3"};

	std::string l_toe_center_tf_ = "leg_l_toe_center";
	int l_toe_tf_num = 4;
	std::string l_toe_tf_in_[4] = {
			"leg_l_toe_link0",
			"leg_l_toe_link1",
			"leg_l_toe_link2",
			"leg_l_toe_link3"};

	//内部用変数
	tf2::Quaternion quaternion_;	//現在姿勢
	int quaternion_update_flag_ = 0;
	int joint_update_flag_ = 0;

public:
	FootGrounding(ros::NodeHandle main_nh);
	~FootGrounding();
	void readParams(ros::NodeHandle node_handle_);
	int groundingMainLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
	geometry_msgs::TransformStamped calcRightGroundpoint();
	geometry_msgs::TransformStamped calcLeftGroundpoint();
	geometry_msgs::TransformStamped calcFootsCenter();
	geometry_msgs::TransformStamped calcGroundpoint(geometry_msgs::TransformStamped center,geometry_msgs::TransformStamped right_ground,geometry_msgs::TransformStamped left_ground);
	geometry_msgs::TransformStamped calcImuGround(geometry_msgs::TransformStamped ground);
};














#endif //DRIFT_CORRECTION_H
