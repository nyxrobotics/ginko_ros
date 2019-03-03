#ifndef GINKO_ODOM_H
#define GINKO_ODOM_H

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
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"

class GinkoOdometry {
private:
	//動作:
	//①足の動き:
	//	接地点・ロボット本体の姿勢をサブスクライブ
	//	※現在姿勢が一定以上傾いているときは計算を諦める
	//	計算を諦めるときは直前の速度のローパス(0.2秒くらいで0になる)
	//	基本的には最も地面に近い足裏頂点を基準に回転する
	//	ある程度両方地面に近い時のみ、足裏の回転速度がジャイロの回転速度に近い方の足裏中心に寄せる
	//	足裏の前後移動速度+回転半径*回転速度→前進速度vx1
	ros::Subscriber imu_height_sub_;	//MPU6500中央からみた重力方向への床の高さ
	ros::Subscriber imu_height_vel_sub_;//MPU6500中央からみた重力方向への床の高さ
	ros::Subscriber imu_height_acc_sub_;//MPU6500中央からみた重力方向への床の高さ
	ros::Subscriber r_pose_sub_;		//MPU6500中央から見た足先位置・速度
	ros::Subscriber l_pose_sub_;		//MPU6500中央から見た足先位置・速度
	ros::Subscriber r_ratio_sub_;		//どちらの足の動きを基準に動いているかのパラメータ
	ros::Subscriber l_ratio_sub_;		//合計は基本１、一定以上斜めになると両方0になる
	ros::Subscriber ground_pose_sub_;

	std_msgs::Float32  imu_height_data_;
	std_msgs::Float32  imu_height_vel_data_;
	std_msgs::Float32  imu_height_acc_data_;
	nav_msgs::Odometry r_pose_data_;
	nav_msgs::Odometry l_pose_data_;
	std_msgs::Float32  r_ratio_data_;
	std_msgs::Float32  l_ratio_data_;
	geometry_msgs::PoseStamped ground_pose_data_;

	//②加速度センサ:
	//	加速度センサのx値を常に積分→前進速度vx2
	//	※現在姿勢が一定以上傾いているときはオフセット判定を諦める
	//	静止時(ドリフト補正が走っている時)は加速度センサの重力(z軸回転を抜いたxy回転だけの基準座標)オフセットを判定(時定数1秒くらい)
	//	vx2は(vx1との差が)増え続けていたら加速度センサのセンサ座標オフセットを判定(時定数10秒くらい)
	//	vx2は(①に寄せる)ローパスをかける(時定数10秒くらい)
	ros::Subscriber imu_sub_;
	sensor_msgs::Imu imu_data;

	//③オドメトリ:
	//	角度は完全にジャイロを信頼
	//	最終的には、vx1の低周波成分+vx2の高周波成分→前進速度、みたいな感じにしたい
	//	現状いい方法が思いつかないので、よっぽどのことがなければvx1を使う
	//	①で「現在姿勢が一定以上傾いているとき」は、vx2を使う

	ros::Publisher odom_pub_;
	nav_msgs::Odometry odom_data_;
	std::string odom_parent_name_ = "ground_imu_link";
	std::string odom_angle_base_ = "body_imu_reverse";
	std::string odom_name_ = "odom";
//	boost::shared_ptr<tf2_ros::Buffer> tfBuffer_ptr;
//	boost::shared_ptr<tf2_ros::TransformListener> tfListener_ptr;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	double odom_x = 0.;
	double odom_y = 0.;

public:
	GinkoOdometry(ros::NodeHandle main_nh);
	~GinkoOdometry();
	void readParams(ros::NodeHandle node_handle_);
	int odomLoop();

private:
	void initPublisher(ros::NodeHandle node_handle_);
	void initSubscriber(ros::NodeHandle node_handle_);
	void getImuHeightCallback(const std_msgs::Float32::ConstPtr& msg);
	void getImuHeightVelCallback(const std_msgs::Float32::ConstPtr& msg);
	void getImuHeightAccCallback(const std_msgs::Float32::ConstPtr& msg);
	void getFootRightCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void getFootLeftCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void getFoorRightRatioCallback(const std_msgs::Float32::ConstPtr& msg);
	void getFoorLeftRatioCallback(const std_msgs::Float32::ConstPtr& msg);
	void getImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void getGroundPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void odomTfPublish(const nav_msgs::Odometry odom);
};














#endif //GINKO_ODOM_H
