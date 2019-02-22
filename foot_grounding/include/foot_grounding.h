#ifndef DRIFT_CORRECTION_H
#define DRIFT_CORRECTION_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <iostream>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

class FootGrounding {
private:
	//動作:
	//足裏8点の位置関係を見て、両足接地、片足面接地、片足エッジ接地、を判定する。(自己位置計算用)
	//Publisher:現在高さ、現在速度、TF
	//TF:自己位置増減計算の起点となる接地点、及び胴体真下のbase_linkをpublishする。
	//NodeHandler,Publisher,Subscriber
	ros::NodeHandle node_handle_;
	ros::Subscriber imu_quaternion_sub_;//足裏のどの頂点が接地しているかを判定するため、現在姿勢を受け取る。
	ros::Subscriber joint_states_sub_;	//関節角度の更新周期に合わせて再計算する。データ自体は使わない。

	ros::Publisher ground_height_pub_;  //MPU6500中央からみた重力方向への床の高さ
	ros::Publisher velocity_pub_;       //重力方向を鉛直、胸部前方をx軸とした時の、MPU6500中央のxyz方向の速度

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	//rosparam
	std::string center_tf_in_name_ = "body_imu_base_link";//ジャイロのTF
	std::string base_tf_out_name_  = "ground_base_link";//ジャイロを地面に投影した位置(odomを生やす基準点にする予定)
	std::string ground_point_tf_out_name_ = "ground_point_link";//足裏の接地点
	double footup_thresh_min_ = 0.001;		//どちらかの足が上がっている判定のスレッショルド
	double footup_thresh_max_ = 0.003;		//minからmaxまではground_point_linkが線形で遷移
	double toe_edg_thresh_min_ = 0.0005;	//片方の足において足先エッジが上がっている判定のスレッショルド
	double toe_edg_thresh_max_ = 0.002;		//minからmaxまではground_point_linkが線形で遷移

	//内部処理用変数
	sensor_msgs::Imu imu_raw_;
	sensor_msgs::Imu imu_quaternion_;

	double stopping_ = 0;		//算出する現在のドリフト値
	double gyro_stopping_ = 0;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double accel_stopping_ = 0;	//1~0の値を取る。これが1に近くなるとドリフト補正を始める。
	double joint_target_stopping_ = 0;
	double joint_sens_stopping_ = 0;

	//TODO:パラメータにする
	//各種センサの静止判定のスレッショルド
	double thresh_gyro_stopping_ = 0.006;
	double thresh_accel_stopping_= 0.2;
	double thresh_joint_target_stopping_ = 0.01;
	double thresh_joint_sens_stopping_ = 0.0001;
	//ドリフト補正の1ステップで反映する割合
	double drift_correction_speed_norm_ = 0.01;

	tf2::Quaternion quaternion_;	//ドリフト補正を重力周りにかけるため、現在姿勢も受け取る必要がある。
	tf2::Vector3 drifting_;			//現在姿勢に対して重力周りの回転だけを取り出してドリフトとして引く。

public:
	FootGrounding(ros::NodeHandle main_nh);
	~FootGrounding();

	double drift_thresh = 0.99;
	bool publish_debug_topic = true;
	void readParams(ros::NodeHandle main_nh);

private:
	void initPublisher();
	void initSubscriber();
	void getImuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void getJointGoalsCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void getJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

	double gyroNormal(const sensor_msgs::Imu imu_in);
	double accelNormal(const sensor_msgs::Imu imu_in);
	double jointStatesNormal(const sensor_msgs::JointState joints_in);
	double jointGoalsNormal(const sensor_msgs::JointState joints_in);
};














#endif //DRIFT_CORRECTION_H
