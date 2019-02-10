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


