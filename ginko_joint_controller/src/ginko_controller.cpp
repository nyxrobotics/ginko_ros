

#include "ginko_controller.h"
#include <omp.h>

double mapd(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//GinkoControler here
GinkoController::GinkoController(){
	initPublisher();
	initSubscriber();
	ROS_INFO("Ginko_controller : Init OK!");
	ginko_serial_.switchTorque(255,false);
	ROS_INFO("Ginko_controller : Torque Off OK!");
}
GinkoController::~GinkoController() {
	ginko_serial_.switchTorque(255,false);
//	ginko_serial_.portClose();
	ros::shutdown();
}
void GinkoController::initPublisher() {
	joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
}
void GinkoController::initSubscriber() {
	goal_joint_states_sub_ = node_handle_.subscribe("goal_joint_position", 1,&GinkoController::goalJointPositionCallback, this);
	torque_enable_sub_ = node_handle_.subscribe("torque_enable",1, &GinkoController::torqueEnableCallback, this);
}

void GinkoController::requestJointStates(unsigned char comnum) {
	unsigned char servocount = ginko_params_._servo_count[comnum];
	ginko_serial_.updateRxRingBuffer(comnum);
	for (int servonum = 0; servonum < servocount; servonum++) {
		ginko_serial_.requestReturnPacket(ginko_params_._servo_id[comnum][servonum]);
		ginko_serial_.updateRxRingBuffer(comnum);
		int id_tmp = ginko_serial_.ringBufferGotoOldestHeader(comnum);
		while(id_tmp != 0){
			ginko_serial_.getOldestPacketAndIncrementRing(comnum);

			get_joint_position[id_tmp-1]=ginko_serial_.readServoPosition(id_tmp);
			get_joint_velocity[id_tmp-1]=ginko_serial_.readServoVelocity(id_tmp);
			get_joint_effort[id_tmp-1]=ginko_serial_.readServoTorque(id_tmp);

			state_pose_[id_tmp-1]=ginko_serial_.readServoPosition(id_tmp);
			// ROS_INFO("id:%d state_pose_:%f",id_tmp,state_pose_[id_tmp-1]);

			id_tmp = ginko_serial_.ringBufferGotoOldestHeader(comnum);
		}
	}
}

void GinkoController::updateJointStates() {
	sensor_msgs::JointState joint_state;//push_backで追加しているので毎回初期化が必須
	joint_state.header.frame_id = "world";
	joint_state.header.stamp = ros::Time::now();// - ros::Duration(0.1);

	//サーボのID順に書く必要アリ

	joint_state.name.push_back("leg_r_joint8");
	joint_state.name.push_back("leg_r_joint7");
	joint_state.name.push_back("leg_r_joint6");
	joint_state.name.push_back("leg_r_joint4");
	joint_state.name.push_back("leg_r_joint3");
	joint_state.name.push_back("leg_r_joint1");
	joint_state.name.push_back("leg_r_joint0");

	joint_state.name.push_back("leg_l_joint8");
	joint_state.name.push_back("leg_l_joint7");
	joint_state.name.push_back("leg_l_joint6");
	joint_state.name.push_back("leg_l_joint4");
	joint_state.name.push_back("leg_l_joint3");
	joint_state.name.push_back("leg_l_joint1");
	joint_state.name.push_back("leg_l_joint0");

	joint_state.name.push_back("body_joint1");

	joint_state.name.push_back("arm_r_joint0");
	joint_state.name.push_back("arm_r_joint1");
	joint_state.name.push_back("arm_r_joint1_rev");
	joint_state.name.push_back("arm_r_joint2");
	joint_state.name.push_back("arm_r_joint3");

	joint_state.name.push_back("arm_l_joint0");
	joint_state.name.push_back("arm_l_joint1");
	joint_state.name.push_back("arm_l_joint1_rev");
	joint_state.name.push_back("arm_l_joint2");
	joint_state.name.push_back("arm_l_joint3");

	for (int index = 0; index < SERVO_NUM; index++) {
		joint_states_pos[index]=get_joint_position[index];
		joint_states_vel[index]=get_joint_velocity[index];
		joint_states_eff[index]=get_joint_effort[index];
	}

	for (int index = 0; index < SERVO_NUM; index++) {
		joint_state.position.push_back(joint_states_pos[index]);
		joint_state.velocity.push_back(joint_states_vel[index]);
		joint_state.effort.push_back(joint_states_eff[index]);
	}

	joint_states_pub_.publish(joint_state);
}
void GinkoController::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	for (int index = 0; index < SERVO_NUM; index++){
		target_pose_[index] = msg->position.at(index);//送られてくるjointの目標値の数が少ないと配列外参照になるので注意
	}

	for(int comnum = 0; comnum<ginko_params_._com_count; comnum++){
		pose_request_[comnum] = 1;
	}
}
void GinkoController::torqueEnableCallback(const std_msgs::Int8 &msg) { //0:off, 1:on, 2:break

	for(int comnum = 0; comnum<ginko_params_._com_count; comnum++){
		torque_enable_[comnum] = msg.data;
		torque_request_[comnum] = 1;
	}
	// ROS_INFO("torque on* %d",msg.data);
}
void GinkoController::control_loop_com(unsigned char comnum) {
	//ROS_INFO("Ginko_controller : Loop Start!");

	//1:トルクの切り替え(サブスクライブがあった場合のみ)
	static unsigned char torque_enable_pre_[GinkoParams::_com_count] = {};//ゼロ初期化必須
	if (torque_request_[comnum] != 0) {
		//トルク切り替え処理
		if(torque_enable_[comnum]== 1){
			if(torque_enable_pre_[comnum] != 1){
				ROS_INFO("torque on");
				ginko_timer_.msecStart();
				timestamp_ms_= ginko_timer_.msecGet();
				for (int index = 0; index < SERVO_NUM; index++){
					init_pose_[index] = state_pose_[index];
				}
			}
			ginko_serial_.switchAllTorque_com(comnum,true);
		}else{
			ginko_serial_.switchAllTorque_com(comnum,false);
			timestamp_ms_ = startup_ms_;
		}
		torque_request_[comnum] = 0;
	}
	torque_enable_pre_[comnum] = torque_enable_[comnum];




	//2:目標値の反映
	if(torque_enable_[comnum]==1){
		if(timestamp_ms_ < startup_ms_){
			timestamp_ms_= ginko_timer_.msecGet();
//			ROS_INFO("startup_ms:%d , start:%f, end:%f",timestamp_ms_,init_pose_[0],target_pose_[0]);
			double startup_pose_[SERVO_NUM]={};
			for (int index = 0; index < SERVO_NUM; index++){
				startup_pose_[index] = init_pose_[index] + (target_pose_[index] - init_pose_[index])*(double)timestamp_ms_/(double)startup_ms_;
			}
			//ginko_serial_.sendTargetPosition(startup_pose_);
			ginko_serial_.sendTargetPositionWithSpeedSingleCom(comnum,startup_pose_,1000./(double)(LOOP_FREQUENCY));
		}else{
//			if(pose_request_ == 1){
//				ginko_serial_.sendTargetPosition(target_pose_);
				ginko_serial_.sendTargetPositionWithSpeedSingleCom(comnum,target_pose_,1000./(double)(LOOP_FREQUENCY));
//				pose_request_ = 0;
//			}
		}
	}
//		ginko_timer_.usleepSpan(100);
	//3:リターン角度の更新
		requestJointStates(comnum);
}

void GinkoController::control_loop_main(void) {
	//4:現在値のパブリッシュ(comポート全てに対して一回)(すべての処理の最初で良いと思われる)
	updateJointStates();
}
