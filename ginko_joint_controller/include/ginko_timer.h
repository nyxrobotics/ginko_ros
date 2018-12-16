

#ifndef GINKO_TIMER_H
#define GINKO_TIMER_H

#include <ros/ros.h>

#include <vector>
#include <string>

//GinkoTimer
#include <std_msgs/Int8.h>
#include <time.h>

class GinkoTimer { //CLOCK_MONOTONICを使ってタイマー管理を行う
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

#endif //GINKO_TIMER_H
