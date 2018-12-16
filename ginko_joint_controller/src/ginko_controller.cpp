

#include "ginko_controller.h"


double mapd(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//GinkoControler here
GinkoController::GinkoController(){
	initPublisher();
	initSubscriber();
	initOffsetsReconfigure();
	ROS_INFO("Ginko_controller : Init OK!");
	ginko_serial_.switchTorque(255,false);
}
GinkoController::~GinkoController() {
//	for (uint8_t num = 0; num < SERVO_NUM; num++)
//		joint_controller_->itemWrite(joint_id_.at(num), "Torque_Enable", false);
//
//	gripper_controller_->itemWrite(gripper_id_.at(0), "Torque_Enable", false);
	ginko_serial_.switchTorque(255,false);
	ginko_serial_.portClose();
	ros::shutdown();
}
void GinkoController::initPublisher() {
	joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(
			robot_name_ + "/joint_states", 10);
}
void GinkoController::initSubscriber() {
	goal_joint_states_sub_ = node_handle_.subscribe(
			robot_name_ + "/goal_joint_position", 100,
			&GinkoController::goalJointPositionCallback, this);
	torque_enable_sub_ = node_handle_.subscribe(robot_name_ + "/torque_enable",
			1, &GinkoController::torqueEnableCallback, this);
}
void GinkoController::initOffsetsReconfigure() {
//	ここに宣言すると共有化に失敗してコールバックが呼ばれない。プライベート変数に入れた。
//	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
//	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;
	callback_server = boost::bind(&GinkoController::offsetsReconfigureCallback,this, _1, _2);
	ROS_INFO("Reconfigure Initializad");
	param_server.setCallback(callback_server);
}
void GinkoController::offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level) {
//	ROS_INFO("Reconfigure Request");

	servo_offsets_[0] = config.servo_01_ofs;
	servo_offsets_[1] = config.servo_02_ofs;
	servo_offsets_[2] = config.servo_03_ofs;
	servo_offsets_[3] = config.servo_04_ofs;
	servo_offsets_[4] = config.servo_05_ofs;

	servo_offsets_[5] = config.servo_06_ofs;
	servo_offsets_[6] = config.servo_07_ofs;
	servo_offsets_[7] = config.servo_08_ofs;
	servo_offsets_[8] = config.servo_09_ofs;
	servo_offsets_[9] = config.servo_10_ofs;

	servo_offsets_[10] = config.servo_11_ofs;
	servo_offsets_[11] = config.servo_12_ofs;
	servo_offsets_[12] = config.servo_13_ofs;
	servo_offsets_[13] = config.servo_14_ofs;
	servo_offsets_[14] = config.servo_15_ofs;

	servo_offsets_[15] = config.servo_16_ofs;
	servo_offsets_[16] = config.servo_17_ofs;
	servo_offsets_[17] = config.servo_18_ofs;
	servo_offsets_[18] = config.servo_19_ofs;
	servo_offsets_[19] = config.servo_20_ofs;

	servo_offsets_[20] = config.servo_21_ofs;
	servo_offsets_[21] = config.servo_22_ofs;
	servo_offsets_[22] = config.servo_23_ofs;
	servo_offsets_[23] = config.servo_24_ofs;
	servo_offsets_[24] = config.servo_25_ofs;

	ofs_reconf_request = 1;
}

void GinkoController::updateJointStates() {
	sensor_msgs::JointState joint_state;

	float joint_states_pos[SERVO_NUM] = {};
	float joint_states_vel[SERVO_NUM] = {};
	float joint_states_eff[SERVO_NUM] = {};

	static double get_joint_position[SERVO_NUM] = {};
	static double get_joint_velocity[SERVO_NUM] = {};
	static double get_joint_effort[SERVO_NUM] = {};
//	ginko_serial_.updateRxRingBuffer();
	for (int index = 0; index < SERVO_NUM; index++) {
		ginko_serial_.requestReturnPacket(index+1);
		ginko_serial_.updateRxRingBuffer();

		int id_tmp = ginko_serial_.ringBufferGotoOldestHeader();
		while(id_tmp != 0){
			ginko_serial_.getOldestPacketAndIncrementRing();
			get_joint_position[id_tmp-1]=ginko_serial_.readServoPosition(id_tmp);
			get_joint_velocity[id_tmp-1]=ginko_serial_.readServoVelocity(id_tmp);
			get_joint_effort[id_tmp-1]=ginko_serial_.readServoTorque(id_tmp);

			state_pose_[id_tmp-1]=ginko_serial_.readServoPosition(id_tmp);
//			ROS_INFO("id:%d state_pose_:%f",id_tmp,state_pose_[id_tmp-1]);
			id_tmp = ginko_serial_.ringBufferGotoOldestHeader();
		}
	}

	joint_state.header.frame_id = "world";
	joint_state.header.stamp = ros::Time::now();

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
		joint_states_pos[index]=get_joint_position[index]+servo_offsets_[index];
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
//		target_pose_[index] = msg->position.at(index) - servo_offsets_[index];//送られてくるjointの目標値の数が少ないと配列外参照になるので注意
		target_pose_[index] = msg->position.at(index);
	}
	pose_request_ = 1;
}
void GinkoController::torqueEnableCallback(const std_msgs::Int8 &msg) { //0:off, 1:on, 2:break
	torque_enable_ = msg.data;
	torque_request_ = 1;
//	ROS_INFO("torque on* %d",msg.data);
}
void GinkoController::control_loop() {
	//1:トルクの切り替え(サブスクライブがあった場合のみ)
	static unsigned char torque_enable_pre_ = 0;
	if (torque_request_ != 0) {
		//トルク切り替え処理
		if(torque_enable_== 1){
			if(torque_enable_pre_ != 1){
				ROS_INFO("torque on");
				ginko_timer_.msecStart();
				timestamp_ms_= ginko_timer_.msecGet();
				for (int index = 0; index < SERVO_NUM; index++){
					init_pose_[index] = state_pose_[index] + servo_offsets_[index];
				}
			}
			ginko_serial_.switchTorque(255,true);
		}else{
			ginko_serial_.switchTorque(255,false);
			timestamp_ms_ = startup_ms_;
		}
		torque_request_ = 0;
	}
	torque_enable_pre_ = torque_enable_;

	//2:目標値の反映
	if(torque_enable_==1){
		if(timestamp_ms_ < startup_ms_){
			timestamp_ms_= ginko_timer_.msecGet();
//			ROS_INFO("startup_ms:%d , start:%f, end:%f",timestamp_ms_,init_pose_[0],target_pose_[0]);
			double startup_pose_[SERVO_NUM]={};
			for (int index = 0; index < SERVO_NUM; index++){
				startup_pose_[index] = init_pose_[index] + (target_pose_[index] - init_pose_[index])*(double)timestamp_ms_/(double)startup_ms_;
			}
			//ginko_serial_.sendTargetPosition(startup_pose_);
			ginko_serial_.sendTargetPositionWithSpeed(startup_pose_,1000./(double)(LOOP_FREQUENCY));
		}else{
//			if(pose_request_ == 1){
//				ginko_serial_.sendTargetPosition(target_pose_);
				ginko_serial_.sendTargetPositionWithSpeed(target_pose_,1000./(double)(LOOP_FREQUENCY));
//				pose_request_ = 0;
//			}
		}
	}

	//3:現在値のパブリッシュ,リターン角度の更新
	updateJointStates();

}

