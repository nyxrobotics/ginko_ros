#ifndef GINKO_SERIAL_H
#define GINKO_SERIAL_H

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
#include <ginko_params.h>
#include "ginko_timer.h"


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
	ros::NodeHandle node_handle_;
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


#endif //GINKO_SERIAL_H
