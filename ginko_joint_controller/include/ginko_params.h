#ifndef GINKO_PARAMS_H
#define GINKO_PARAMS_H

//GinkoController
#define LOOP_FREQUENCY  (100)
#define SERVO_NUM     25
//GinkoSerial
#define RxRingBufferLength 10000


#include <stdint.h>
#include <string>
#include <vector>
#include <omp.h>

#pragma execution_character_set("utf-8")


class GinkoParams{
private:


public:
	GinkoParams();
	~GinkoParams();

	static const uint16_t _com_count = 4;
	const int _baudrate = 460800;
//	const std::string _com_names[_com_count] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
	const std::string _com_names[_com_count] = {"/dev/ttyFT232R_3.1", "/dev/ttyFT232R_3.2", "/dev/ttyFT232R_3.3", "/dev/ttyFT232R_3.4"};
	const uint16_t _servo_count[_com_count] = {7,7,5,6};
	static const uint16_t _servo_count_max = 7; //_servo_countの要素のうち、最大値(実行ループのボトルネックになる)
	const std::string _joint_name[_com_count][_servo_count_max]={
			{"leg_r_joint8" , "leg_r_joint7"    ,"leg_r_joint6"    , "leg_r_joint4" , "leg_r_joint3" , "leg_r_joint1" , "leg_r_joint0"},
			{"leg_l_joint8" , "leg_l_joint7"    ,"leg_l_joint6"    , "leg_l_joint4" , "leg_l_joint3" , "leg_l_joint1" , "leg_l_joint0"},
			{"arm_r_joint1" , "arm_r_joint1_rev","arm_r_joint0"    ,"arm_r_joint2"  ,"arm_r_joint3" },
			{"body_joint1"  ,"arm_l_joint"      ,"arm_l_joint1_rev", "arm_l_joint0" , "arm_l_joint2" , "arm_l_joint3" }
	};

	const uint16_t _servo_id[_com_count][_servo_count_max]={
			{1 ,2 ,3 ,4 ,5 ,6 ,7},
			{8 ,9 ,10,11,12,13,14},
			{16,17,18,19,20},
			{15,21,22,23,24,25}
	};
};


#endif //GINKO_PARAMS_H
