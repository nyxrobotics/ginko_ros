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

#ifndef GINKO_PLAYER_H
#define GINKO_PLAYER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include "ginko_timer.h"

//Motions



namespace ginko_player {
using namespace ginko_timer;


class GinkoPlayer { //CLOCK_MONOTONICを使ってタイマー管理を行う関数を用意したい
private:
	GinkoTimer ginko_timer_;

//	ros::NodeHandle node_handle_;
	ros::Publisher goal_joint_states_pub_;
	ros::Publisher torque_enable_pub_;

public:
	GinkoPlayer();
	~GinkoPlayer();
	void playPose(int motion[][SERVO_NUM + 2], uint8_t pose_num);
	void playMotion(int motion[][SERVO_NUM + 2]);
	void torqueEnable(char torque_enable);
	void initPublisher();
private:
//	void initPublisher();
	void posePublish(double goalJoint[SERVO_NUM]);

};




}

#endif //GINKO_JOINT_CONTROLLER_H
