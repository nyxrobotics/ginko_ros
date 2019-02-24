#ifndef ROLLPITCH_TF_H
#define ROLLPITCH_TF_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <iostream>
//used for handling TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>


class RollPitchTF {
	private:
		tf2_ros::TransformBroadcaster tfBroadcaster;
		ros::Publisher yaw_angle_pub_;
		bool publish_debug_topic_ = true;
		ros::NodeHandle main_nh;

	public:
		RollPitchTF();
		~RollPitchTF();
		void initPublisher(ros::NodeHandle main_nh);
		tf2::Quaternion calcRollPitchQuaternion(const sensor_msgs::Imu imu_in);
		tf2::Quaternion calcYawQuaternion(const sensor_msgs::Imu imu_in);
		void broadcastRotatedTf(std::string parent_name,std::string tf_name,tf2::Quaternion rotation);
		void broadcastRotatedTfReverse(std::string parent_name,std::string tf_name,tf2::Quaternion rotation);
	private:


	};


#endif //ANGLE_CORRECTION_H
