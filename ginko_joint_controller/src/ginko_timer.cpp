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

#include "ginko_timer.h"

using namespace ginko_timer;


//GinkoTimer here
GinkoTimer::GinkoTimer() {
	clock_getres(CLOCK_MONOTONIC, &ts_stamp_);
	clock_getres(CLOCK_MONOTONIC, &ts_now_);
	clock_gettime(CLOCK_MONOTONIC, &ts_stamp_);
	clock_gettime(CLOCK_MONOTONIC, &ts_now_);
}
GinkoTimer::~GinkoTimer() {
}
void GinkoTimer::usecStart(void) {
	clock_gettime(CLOCK_MONOTONIC, &ts_stamp_);
}
long GinkoTimer::usecGet(void) {
	long us_interval = 0;
	struct timespec ts_interval_;
	clock_gettime(CLOCK_MONOTONIC, &ts_now_);
	ts_interval_.tv_sec=ts_now_.tv_sec - ts_stamp_.tv_sec;
	ts_interval_.tv_nsec=ts_now_.tv_nsec - ts_stamp_.tv_nsec;
	us_interval = ((double)ts_interval_.tv_sec * 1000000.)+(ts_interval_.tv_nsec / 1000.) ;
	return us_interval;
}
void GinkoTimer::usleepSpan(long usec) { //ヘッダに書いた通りの機能になってるかわからない
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = 1000 * usec;
	clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
}
void GinkoTimer::usleepCyclic(long usec) {
	while (usecGet() < usec) {

	}
	usecStart();
}
void GinkoTimer::msecStart(void) {
	usecStart();
}
long GinkoTimer::msecGet(void) {
	long ms_interval = 0;
	struct timespec ts_interval_;
	clock_gettime(CLOCK_MONOTONIC, &ts_now_);
	ts_interval_.tv_sec=ts_now_.tv_sec - ts_stamp_.tv_sec;
	ts_interval_.tv_nsec=ts_now_.tv_nsec - ts_stamp_.tv_nsec;
	ms_interval = ((double)ts_interval_.tv_sec * 1000.)+(ts_interval_.tv_nsec / 1000000.) ;
	return ms_interval;
}
void GinkoTimer::msleepSpan(long msec) { //ヘッダに書いた通りの機能になってるかわからない
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = 1000 * msec;
	for (int i = 0; i < 1000; i++) {
		clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
	}
}
void GinkoTimer::msleepCyclic(long msec) {
	while (msecGet() < msec) {

	}
	msecStart();
}


