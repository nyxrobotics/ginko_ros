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
#include<omp.h>

class GinkoSerial {
private:
	GinkoTimer ginko_timer_;
	GinkoParams ginko_params_;
    //std::string port_name_;
    int baud_ = 115200;
    int return_packet_size_ = 26;
    void check_error(const char*, int);
    static const unsigned char _com_count = GinkoParams::_com_count;//なぜこれでコンパイルが通るのかわからない
    int fd_[_com_count]; // file descriptor
    struct termios tio_[_com_count], tio_backup_[_com_count];
    unsigned char ring_buffer_[_com_count][RxRingBufferLength] = {};
	int ring_rp_[_com_count]; //= 0,
	int ring_wp_[_com_count]; //   = 1;

	double tx_pose_[SERVO_NUM]   = {};
	double rx_pose_[SERVO_NUM]   = {};
	double rx_vel_[SERVO_NUM]    = {};
	double rx_torque_[SERVO_NUM] = {};
	ros::NodeHandle node_handle_;
    //boost::shared_ptr<serial_port> sp;

public:
    GinkoSerial();
//    GinkoSerial(std::string port_name, unsigned long baudrate);
	~GinkoSerial();
	void portOpen(unsigned char com_num,std::string port_name,unsigned long baudrate);
	void portClose(unsigned char com_num);
	void setServoBaudrate(unsigned int baudrate);
	void sendTargetPosition(const double *value);
	void sendTargetPositionWithSpeedSingleCom(const unsigned char comnum,const double *value, const double ms);
	void sendTargetPositionWithSpeed(const double *value,const double ms);
	void switchAllTorque(bool sw);
	void switchTorque(unsigned char servo_id, bool sw);
	int requestReturnPacket(int servo_id);

	void updateRxRingBuffer(unsigned char com_num);
	int readRingBufferReadySize(unsigned char com_num);
	int ringBufferGotoOldestHeader(unsigned char com_num);//return id //チェックサムまで確認する //なかったら0返す
	void getOldestPacketAndIncrementRing(unsigned char com_num); //該当idからきたサーボ情報を反映する

	double readServoPosition(int servo_id);
	double readServoVelocity(int servo_id);
	double readServoTorque(int servo_id);

private:
    void send_packet(unsigned char com_num, void*, int);
    void receive_packet(unsigned char com_num, void *buf_ptr, int size);
    int get_fd(unsigned char com_num);
	int readRxBufferReadySize(unsigned char com_num);
	//void reloadStatusBuffer(void);
	//void ringBufferGotoLatestHeader(int servo_id);//特定サーボの最新のデータまで進む //チェックサムまで確認する

};


#endif //GINKO_SERIAL_H
