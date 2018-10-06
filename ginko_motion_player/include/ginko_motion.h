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

#ifndef GINKO_MOTION_H
#define GINKO_MOTION_H


#include <iostream>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH



//namespace ginko_motion {
//inko_joint_controllerのGinkoControllerと同じ
#define LOOP_FREQUENCY  (30)
#define SERVO_NUM     25



//class GinkoFSM { //FSMを使って状態遷移を可視化できるようにする
//private:

//	volatile bool leftBumper, rightBumper, wallSensor;
//	random_numbers::RandomNumberGenerator _randomizer;
//	string _motiomCommand = "TorqueFree";
//	unsigned char _motiomCommandChanged = 0;

//	ros::NodeHandle node_handle_;
//	ros::Subscriber _motiomCommandSub;
//	ros::Publisher _PosePublisher;

//public:
//
//private:
//	void initSubscriber();
	void motionCommandCallback(const std_msgs::String::ConstPtr& msg);
	string getMotionCommand(void);
	unsigned char getCommandChanged(void);
	decision_making::TaskResult torqueFreeTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
	decision_making::TaskResult torqueOnTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
//};



//}

#endif //OPEN_MANIPULATOR_Ginko_CONTROLLER_H
