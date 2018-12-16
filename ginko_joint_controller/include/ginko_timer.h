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

#ifndef GINKO_TIMER_H
#define GINKO_TIMER_H

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
#include "ginko_definitions.h"


namespace ginko_timer {
using namespace ginko_definitions;

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




}

#endif //GINKO_TIMER_H
