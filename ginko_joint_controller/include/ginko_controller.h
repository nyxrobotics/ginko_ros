/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef GINKO_JOINT_CONTROLLER_H
#define GINKO_JOINT_CONTROLLER_H

#include <ros/ros.h>

#include <vector>
#include <string>

//#include <sstream>
//#include <cmath>
//#include <cstdlib>
//#include <boost/bind.hpp>
//#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

//GinkoTimer
#include <std_msgs/Int8.h>
#include <time.h>

//GinkoSerial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> // 受信バッファに届いているデータの数を所得するために使用
//#include "serial.hpp"
#include <linux/serial.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <ginko_joint_controller/servo_offsetsConfig.h> //(project)/cfg/servo_offsets.cfgから自動生成されるらしい


namespace ginko {
//GinkoController
#define LOOP_FREQUENCY  (30)
#define SERVO_NUM     25
//GinkoSerial
#define RxRingBufferLength 10000


class GinkoTimer { //CLOCK_MONOTONICを使ってタイマー管理を行う関数を用意したい
private:
	struct timespec ts_now_;
	struct timespec ts_stamp_;

public:
	GinkoTimer();
	~GinkoTimer();
	void usecStart(void);			//関数呼び出しの時点で開始時刻を更新
	long usecGet(void);				//開始時刻からの経過時間を所得する
	void usleepSpan(long usec);		//関数呼び出しの時点で開始時刻を更新し、そこからの経過時間を見る
	void usleepCyclic(long usec);	//前回の関数呼び出し終了時点からの経過時間を見る
	void msecStart(void);
	long msecGet(void);
	void msleepSpan(long msec);
	void msleepCyclic(long msec);
private:
	void initTimer(void);

};


class GinkoSerial {
private:
	GinkoTimer ginko_timer_;
    std::string port_name_;
    int baud_ = 115200;
    int return_packet_size_ = 26;
    void check_error(const char*, int);
    struct termios tio_, tio_backup_;
    int fd_ = 0;     // file descriptor
    unsigned char ring_buffer_[RxRingBufferLength] = {};
	int ring_rp_ = 0, ring_wp_   = 1;
	double tx_pose_[SERVO_NUM]   = {};
	double rx_pose_[SERVO_NUM]   = {};
	double rx_vel_[SERVO_NUM]    = {};
	double rx_torque_[SERVO_NUM] = {};
    //boost::shared_ptr<serial_port> sp;

public:
    GinkoSerial();
    GinkoSerial(std::string port_name, unsigned long baudrate);
	~GinkoSerial();
	void portOpen(std::string port_name,unsigned long baudrate);
	void portClose(void);
	void setServoBaudrate(unsigned int baudrate);
	void sendTargetPosition(const double *value);
	void sendTargetPositionWithSpeed(const double *value,const double ms);
	void switchTorque(unsigned char id, bool sw);
	void requestReturnPacket(int servo_id);

	void updateRxRingBuffer(void);
	int readRingBufferReadySize(void);
	int ringBufferGotoOldestHeader(void);//return id //チェックサムまで確認する //なかったら0返す
	void getOldestPacketAndIncrementRing(void); //該当idからきたサーボ情報を反映する

	double readServoPosition(int servo_id);
	double readServoVelocity(int servo_id);
	double readServoTorque(int servo_id);

private:
    void send_packet(void*, int);
    void receive_packet(void*, int);
    int get_fd();
	int readRxBufferReadySize(void);
	//void reloadStatusBuffer(void);
	//void ringBufferGotoLatestHeader(int servo_id);//特定サーボの最新のデータまで進む //チェックサムまで確認する

};

class GinkoController {
private:
	GinkoSerial ginko_serial_;
	GinkoTimer ginko_timer_;
	unsigned char torque_enable_ = 0 , torque_request_ = 0 , pose_request_ = 0, ofs_reconf_request = 0;
	double init_pose_[SERVO_NUM]={};
	double target_pose_[SERVO_NUM]={};
	double state_pose_[SERVO_NUM]={};
	double servo_offsets_[SERVO_NUM]={};
	unsigned int timestamp_ms_ = 0;
	const unsigned int startup_ms_ = 2000;
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	ros::NodeHandle priv_node_handle_;
	// ROS Parameters
	// ROS Topic Publisher
	ros::Publisher joint_states_pub_;
	// ROS Topic Subscriber
	ros::Subscriber goal_joint_states_sub_;
	ros::Subscriber torque_enable_sub_;
	// ROS Service Server
	// ROS Service Client
	// Ginko Parameters
	std::string robot_name_;
//	float protocol_version_;

//	DynamixelWorkbench *joint_controller_;
//	DynamixelWorkbench *gripper_controller_;

	std::vector<uint8_t> joint_id_;
	std::vector<uint8_t> gripper_id_;

	std::string joint_mode_;
	std::string gripper_mode_;

	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;

public:
	GinkoController();
	~GinkoController();
	void control_loop();

private:
//	void initMsg();
	void initPublisher();
	void initSubscriber();
	void initOffsetsReconfigure();
	void offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level);
	void updateJointStates();
	void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void torqueEnableCallback(const std_msgs::Int8 &msg);
};






}

#endif //GINKO_JOINT_CONTROLLER_H
