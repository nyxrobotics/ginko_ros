#ifndef QUATERNION_TO_RPY_H
#define QUATERNION_TO_RPY_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

class ImuRpy {
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber imu_quaternion_sub_; //クォータニオンをサブスクライブ
	ros::Publisher imu_euler_pub_; //オイラー角でRPYをパブリッシュ

	//tf2_ros::TransformBroadcaster tf2_br_;
	//tf2_ros::Buffer tf2_buf_;
	//tf2_ros::TransformListener tf2_lis_(tf2_buf_);

public:
	ImuRpy();
	~ImuRpy();

private:
	void initSubscriber();
	void initPublisher();
	void getQuaternionCallback(const sensor_msgs::ImuConstPtr& msg);

};
















#endif //QUATERNION_TO_RPY_H
