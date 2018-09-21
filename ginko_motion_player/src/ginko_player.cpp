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

#include "ginko_player.h"

using namespace ginko_player;


//GinkoTimer here
GinkoPlayer::GinkoPlayer() {
	//init timer
    ginko_timer_.usecStart();
//    initPublisher();
}
GinkoPlayer::~GinkoPlayer() {
}
void GinkoPlayer::initPublisher(){
	ros::NodeHandle node_handle_;
	goal_joint_states_pub_ 	= node_handle_.advertise<sensor_msgs::JointState>("/goal_joint_position", 10);
	torque_enable_pub_ 		= node_handle_.advertise<std_msgs::Int8>("/torque_enable", 10);
}
void GinkoPlayer::posePublish(double goalJoint[SERVO_NUM]) {
	sensor_msgs::JointState joint_state;
	for (int i = 0; i < SERVO_NUM; i++) {
		joint_state.position.push_back((float)goalJoint[i]);
//		joint_state.position.push_back(1.0);
	}
	goal_joint_states_pub_.publish(joint_state);
}
void GinkoPlayer::torqueEnable(char torque_enable) {
	std_msgs::Int8 enable_msg;
	enable_msg.data = torque_enable;
	torque_enable_pub_.publish(enable_msg);
}
void GinkoPlayer::playPose(int motion[][SERVO_NUM + 2], uint8_t pose_num) {
	int travel_time_ms = motion[pose_num + 1][0];
	int wait_time_ms =  motion[pose_num + 1][1];
	int poseTimeStepTravel = (travel_time_ms + (1000/2/LOOP_FREQUENCY))/(1000/LOOP_FREQUENCY);
	int poseTimeStepWait = (wait_time_ms + (1000/2/LOOP_FREQUENCY))/(1000/LOOP_FREQUENCY);
//	int poseTimeStep = 0;
	static double endJoint[SERVO_NUM]={};
	double nextJoint[SERVO_NUM];
	double prevJoint[SERVO_NUM]={};//standing(1フレーム動作)を呼ぶことで初期化される形になる

	if(poseTimeStepTravel < 1){//1フレームに満たないものは引き伸ばして1フレームで再生するしかなかった
		poseTimeStepTravel = 1;
		if(poseTimeStepWait>0){
			poseTimeStepWait--;
		}
	}

	for (int i=0;i<SERVO_NUM;i++){
		prevJoint[i]=endJoint[i];//前回の目標位置がスタート位置
	}

	for (int i=0;i<SERVO_NUM;i++){
		int id_tmp = motion[0][i + 2]; //ID
		if (pose_num > 0) {
			endJoint[id_tmp - 1] = (double)(motion[1][i + 2] + motion[pose_num + 1][i + 2]) * 3.1416 / 1800.0;
		} else {
			endJoint[id_tmp - 1] = (double)(motion[1][i + 2]) * 3.1416 / 1800.0;
		}
	}

	for (int i=0;i<poseTimeStepTravel;i++){
		for (int j=0;j<SERVO_NUM;j++){
			nextJoint[j] = prevJoint[j] + (endJoint[j] - prevJoint[j]) * ((double)(i+1)/(double)poseTimeStepTravel);
		}
//		nextJoint[4] = (double)nextJoint[3];
//		nextJoint[6] = (double)nextJoint[5];
//		nextJoint[11] = (double)nextJoint[10];
//		nextJoint[13] = (double)nextJoint[12];
//		nextJoint[18] = -(double)nextJoint[17];
//		nextJoint[23] = -(double)nextJoint[22];

	    ROS_INFO("travel_time_ms:%d, poseTimeStepTravel:%d, i:%d", travel_time_ms ,poseTimeStepTravel,i );
//	    ROS_INFO("SetJoint,%f,%f,%f,%f,%f",endJoint[1] ,nextJoint[1] ,nextJoint[2] ,nextJoint[3] ,nextJoint[4] );
		posePublish(nextJoint);
		ginko_timer_.usleepCyclic(1000000/LOOP_FREQUENCY);
	}
	for (int i=0;i<poseTimeStepWait;i++){
	    ROS_INFO("wait_time_ms:%d, poseTimeStepWait:%d, i:%d", wait_time_ms ,poseTimeStepWait,i );
		posePublish(endJoint);
		ginko_timer_.usleepCyclic(1000000/LOOP_FREQUENCY);
	}

}
void GinkoPlayer::playMotion(int motion[][SERVO_NUM + 2]) {
	int number_of_total_frames = motion[0][0];
	int i;
	for (i = 0; i < number_of_total_frames; i++) {
		playPose(motion, i + 1);
	}
}
