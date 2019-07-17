#ifndef GINKO_PARAMS_H
#define GINKO_PARAMS_H

//GinkoController
#define LOOP_FREQUENCY  (100)
#define SERVO_NUM     25
#define GAZEBO_JOINT_NUM     25
//GinkoSerial


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

//	const uint16_t _servo_count = 25;
	const std::string _servo_joint_name[SERVO_NUM]={
		"leg_r_joint8" , "leg_r_joint7"    ,"leg_r_joint6"     , "leg_r_joint4"   ,"leg_r_joint3" ,"leg_r_joint1" ,"leg_r_joint0",
		"leg_l_joint8" , "leg_l_joint7"    ,"leg_l_joint6"     , "leg_l_joint4"   ,"leg_l_joint3" ,"leg_l_joint1" ,"leg_l_joint0",
		"arm_r_joint0" ,"arm_r_joint1"     ,"arm_r_joint1_rev","arm_r_joint2"    ,"arm_r_joint3" ,
		"body_joint1"  ,"arm_l_joint0"     ,"arm_l_joint1"      ,"arm_l_joint1_rev","arm_l_joint2" ,"arm_l_joint3"
	};
	const uint16_t _servo_id[SERVO_NUM]={
		1 ,2 ,3 ,4 ,5 ,6 ,7,
		8 ,9 ,10,11,12,13,14,
		16,17,18,19,20,
		15,21,22,23,24,25
	};

//	const uint16_t _gazebo_joints_count = 19;
	const std::string _gazebo_joint_name[GAZEBO_JOINT_NUM]={
		"body_joint1",
		"arm_r_joint0","arm_r_joint1","arm_r_joint1_rev","arm_r_joint2","arm_r_joint3",
		"arm_l_joint0","arm_l_joint1","arm_l_joint1_rev","arm_l_joint2","arm_l_joint3",
		"leg_r_joint0","leg_r_joint1","leg_r_joint3","leg_r_joint4","leg_r_joint6","leg_r_joint7","leg_r_joint8",
		"leg_l_joint0","leg_l_joint1","leg_l_joint3","leg_l_joint4","leg_l_joint6","leg_l_joint7","leg_l_joint8"
	};

	const std::string _gazebo_target_topic_footer="_position_controller/command";
};


#endif //GINKO_PARAMS_H
