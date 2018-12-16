#ifndef GINKO_CONTROLLER_H
#define GINKO_CONTROLLER_H

#include <ros/ros.h>

//dynamic reconfigure----
#include <dynamic_reconfigure/server.h>
#include <ginko_joint_controller/servo_offsetsConfig.h> //(project)/cfg/servo_offsets.cfgから自動生成されるらしい
#include "ginko_params.h"



class ServoOffsets {
private:
	unsigned char ofs_reconf_request = 0;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;

public:
	ServoOffsets();
	~ServoOffsets();
	double servo_offsets_[SERVO_NUM]={
			0,		0,		0.052,	0,		0.09,
			0,		0.104,	0,		0,		0.104,
			0.02,	0.1,	0,		0.104,	0,
			0,		0,		0.12,	0,		0,
			0,		0,		0.1,	0,		0};

private:
	void initOffsetsReconfigure();
	void offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level);
};


#endif //GINKO_CONTROLLER_H
