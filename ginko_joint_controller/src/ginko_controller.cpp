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

#include "ginko_controller.h"

using namespace ginko;

double mapd(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//GinkoControler here
GinkoController::GinkoController() :
		node_handle_(""), priv_node_handle_("~") {
//	ginko_serial_.GinkoSerial("/dev/ttyUSB0",115200);
	initPublisher();
	initSubscriber();
	initOffsetsReconfigure();
//	robot_name_ = priv_node_handle_.param<std::string>("robot_name","open_manipulator");
//	std::string device_name = priv_node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
//	uint32_t dxl_baud_rate = priv_node_handle_.param<int>("baud_rate", 1000000);
//	protocol_version_ = priv_node_handle_.param<float>("protocol_version", 2.0);
//	joint_mode_ = priv_node_handle_.param<std::string>("joint_controller","position_mode");
//
//
//
//
//	joint_id_.push_back(priv_node_handle_.param<int>("joint1_id", 1));
//	joint_id_.push_back(priv_node_handle_.param<int>("joint2_id", 2));
//	joint_id_.push_back(priv_node_handle_.param<int>("joint3_id", 3));
//	joint_id_.push_back(priv_node_handle_.param<int>("joint4_id", 4));
//
//	gripper_mode_ = priv_node_handle_.param<std::string>("gripper_controller",
//			"current_mode");
//
//	gripper_id_.push_back(priv_node_handle_.param<int>("gripper_id", 5));
//
//
//	ginko_serial_.portOpen(device_name,115200);
//
//	joint_controller_ = new DynamixelWorkbench;
//	gripper_controller_ = new DynamixelWorkbench;
//
//	joint_controller_->begin(device_name.c_str(), dxl_baud_rate);
//	gripper_controller_->begin(device_name.c_str(), dxl_baud_rate);
//
//	initPublisher();
//	initSubscriber();

	ROS_INFO("Ginko_controller : Init OK!");
	ginko_serial_.switchTorque(255,false);
//	ginko_serial_.setServoBaudrate(460800);
//	ginko_serial_.setServoBaudrate(115200);
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
		target_pose_[index] = msg->position.at(index) - servo_offsets_[index];//送られてくるjointの目標値の数が少ないと配列外参照になるので注意
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

//GinkoSerial here
GinkoSerial::GinkoSerial() {
	ginko_timer_.usecStart();
//	portOpen("/dev/ttyUSB0",115200);
	portOpen("/dev/ttyUSB0",460800);
	ginko_timer_.msleepSpan(1000);
}
GinkoSerial::GinkoSerial(std::string port_name, unsigned long baudrate) {
	ginko_timer_.usecStart();
	portOpen(port_name,baudrate);
}
GinkoSerial::~GinkoSerial() {
	portClose();
}
void GinkoSerial::check_error(const char* action, int status) {
	if (status < 0) {
		ROS_ERROR("%s failed: %s(%s)", action, port_name_.data(),
				std::strerror(errno));
		exit(-1);
	}
	return;
}
void GinkoSerial::portOpen(std::string port_name, unsigned long baudrate) {
//	portClose();
	port_name_ += port_name;
	baud_ = baudrate;
//	sp.reset(new serial_port("/dev/ttyUSB0"));

	fd_ = open(port_name_.data(), O_RDWR| O_NONBLOCK);
	check_error("Open", fd_);

	check_error("Save", tcgetattr(fd_, &tio_backup_));
	// save the current port status

	std::memset(&tio_, 0, sizeof(tio_));
	tio_.c_cc[VMIN] = 0;
	tio_.c_cc[VTIME] = 1;
	if (baudrate == 460800) {
		tio_.c_cflag = B460800 | CS8 | CREAD | CLOCAL;
	} else if (baudrate == 230400) {
		tio_.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
	} else {
		tio_.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	}

	tio_.c_iflag = IGNBRK | IGNPAR;	// init term-io
	check_error("Flash", tcflush(fd_, TCIOFLUSH));	// flush setting
	check_error("Set", tcsetattr(fd_, TCSANOW, &tio_));	// change the port setting

	//Set Low-Latency mode //latency_timer->1ms
	struct serial_struct serial_settings;
	ginko_timer_.usleepSpan(10000);
	ioctl(fd_, TIOCGSERIAL, &serial_settings);
	serial_settings.flags |= ASYNC_LOW_LATENCY;
	ioctl(fd_, TIOCSSERIAL, &serial_settings);
	ginko_timer_.usleepSpan(10000);
	tcflush(fd_,TCIOFLUSH);;//clear buffer
	ginko_timer_.usleepSpan(10000);
	ROS_INFO("USB connected!");
}
void GinkoSerial::portClose(void) {
	check_error("Reset", tcsetattr(fd_, TCSANOW, &tio_backup_));
	// restore the port setting

	close(fd_);

	ROS_INFO("USB disconnected!");
}
void GinkoSerial::setServoBaudrate(unsigned int baudrate) {
	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;
	p[3] = 0x00;
	p[4] = 0x06;
	p[5] = 0x01;
	p[6] = 0x01;
	if (baudrate == 460800) {
		p[7] = 0x0A;
	} else if (baudrate == 230400) {
		p[7] = 0x09;
	} else {
		p[7] = 0x07;	//baud:115200
	}
	p[8] = 0x00;    // check sum
	for (int i = 2; i < 8; i++) {
		p[8] ^= p[i];
	}
	send_packet((void*) p, sizeof(p));
	ginko_timer_.msleepSpan(10);

	//setting other parms
	//zero damper
	p[0] = 0xFA;	//header
	p[1] = 0xAF;	//header
	p[2] = 0xFF;	//id
	p[3] = 0x00;	//flags
	p[4] = 0x14;	//address
	p[5] = 0x01;	//length
	p[6] = 0x01;	//count
	p[7] = 0x00;	//data
	p[8] = 0x00;    // check sum
	for (int i = 2; i < 8; i++) {
		p[8] ^= p[i];
	}
	send_packet((void*) p, sizeof(p));
	ginko_timer_.msleepSpan(10);
	//gain
	unsigned char p1[14];
	p1[0] = 0xFA;	//header
	p1[1] = 0xAF;	//header
	p1[2] = 0xFF;	//id
	p1[3] = 0x00;	//flags
	p1[4] = 0x18;	//address
	p1[5] = 0x04;	//length
	p1[6] = 0x01;	//count
	p1[7] = 0x00;	//data:margin CW
	p1[8] = 0x00;	//data:margin CCW
	p1[9] = 0x08;	//data:slope CW
	p1[10] = 0x08;	//data:slope CCW
	p1[11] = 0x00;	//data:punch Low
	p1[12] = 0x00;	//data:punch High
	p[13] = 0x00;    // check sum
	for (int i = 2; i < 13; i++) {
		p[13] ^= p[i];
	}
	send_packet((void*) p1, sizeof(p1));
	ginko_timer_.msleepSpan(10);

	unsigned char p2[8];    //write to rom
	p2[0] = 0xFA;
	p2[1] = 0xAF;
	p2[2] = 0xFF;
	p2[3] = 0x40;
	p2[4] = 0xFF;
	p2[5] = 0x00;
	p2[6] = 0x00;
	p2[7] = 0x40;
	send_packet((void*) p2, sizeof(p2));
	ginko_timer_.msleepSpan(10);
}
void GinkoSerial::sendTargetPosition(const double *value) {
	int l = 8 + 3 * SERVO_NUM;
	unsigned char p[l];
	int goal[SERVO_NUM];

	for (int i = 0; i < SERVO_NUM; i++) {
		double angle = value[i];
		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}
		tx_pose_[i]=angle;//目標位置と現在位置を比較するときに使うためのバッファ
		goal[i] = (int) (3600 / (2 * M_PI) * angle);
	}

	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0x00;            // ID  (ロングパケット時は0x00)
	p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
	p[4] = 0x1E;            // Adr
	p[5] = 0x03;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
	p[6] = SERVO_NUM;		// Cnt (サーボの数)

	for (int i = 0; i < SERVO_NUM; i++) {
		p[7 + 3 * i] = i + 1;   // each ID
		p[8 + 3 * i] = (unsigned char) (goal[i] & 0xFF);
		p[9 + 3 * i] = (unsigned char) (goal[i] >> 8 & 0xFF);
	}

	p[l - 1] = 0x00;    // check sum
	for (int j = 2; j < l - 1; j++) {
		p[l - 1] ^= p[j];
	}

	send_packet((void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepCyclic((100 + l * 87) / 4);
	}else if (baud_ == 230400) {
		ginko_timer_.usleepSpan((100 + l * 87) / 2);
	} else { //baud_==115200
		ginko_timer_.usleepSpan((100 + l * 87));
	}

	return;
}

void GinkoSerial::sendTargetPositionWithSpeed(const double *value,const double ms) {
	int l = 8 + 5 * SERVO_NUM;
	unsigned char p[l];
	int goal[SERVO_NUM];
	int travel_time = ms/10;

	for (int i = 0; i < SERVO_NUM; i++) {
		double angle = value[i];
		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}
		tx_pose_[i]=angle;//目標位置と現在位置を比較するときに使うためのバッファ
		goal[i] = (int) (3600 / (2 * M_PI) * angle);
	}

	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0x00;            // ID  (ロングパケット時は0x00)
	p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
	p[4] = 0x1E;            // Adr
	p[5] = 0x05;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
	p[6] = SERVO_NUM;		// Cnt (サーボの数)

	for (int i = 0; i < SERVO_NUM; i++) {
		p[7 + 5 * i] = i + 1;   // each ID
		p[8 + 5 * i] = (unsigned char) (goal[i] & 0xFF);
		p[9 + 5 * i] = (unsigned char) (goal[i] >> 8 & 0xFF);
		p[10+ 5 * i] = (unsigned char) (travel_time & 0xFF);
		p[11+ 5 * i] = (unsigned char) (travel_time >> 8 & 0xFF);
	}

	p[l - 1] = 0x00;    // check sum
	for (int j = 2; j < l - 1; j++) {
		p[l - 1] ^= p[j];
	}
	ginko_timer_.usecStart();
	send_packet((void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepCyclic(300+100*SERVO_NUM);
	}else if (baud_ == 230400) {
		ginko_timer_.usleepCyclic(600+200*SERVO_NUM);
	} else { //baud_==115200
		ginko_timer_.usleepCyclic(1200+400*SERVO_NUM);
	}

	return;
}
void GinkoSerial::switchTorque(unsigned char id, bool sw) {
	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = id;      // ID
	p[3] = 0x00;    // Flg
	p[4] = 0x24;    // Adr
	p[5] = 0x01;    // Len
	p[6] = 0x01;    // Cnt

	if (sw) {
		p[7] = 0x01; // Torque on
	} else {
		p[7] = 0x00; // Torque off
	}

	p[8] = 0x00;    // check sum
	for (int j = 2; j < 8; j++) {
		p[8] ^= p[j];
	}

	ginko_timer_.usecStart();
	send_packet((void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepCyclic(300);
	}else if (baud_ == 230400) {
		ginko_timer_.usleepCyclic(600);
	} else { //baud_==115200
		ginko_timer_.usleepCyclic(1200);
	}
	return;
}
void GinkoSerial::requestReturnPacket(int servo_id) {
	unsigned char p[8];

	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = servo_id;   // ID
	p[3] = 0x09;    // Flg
	p[4] = 0x00;    // Adr
	p[5] = 0x00;    // Len(0x00:all)
	p[6] = 0x01;    // Cnt
	p[7] = 0x00;    // check sum
	for (int j = 2; j < 7; j++) {
		p[7] ^= p[j];
	}
	ginko_timer_.usecStart();
	send_packet((void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepCyclic(1000);
	}else if (baud_ == 230400) {
		ginko_timer_.usleepCyclic(2100);
	} else { //baud_==115200
		ginko_timer_.usleepCyclic(3200);
//		ginko_timer_.usleepCyclic((100 + sizeof(p) * 87));			//送信待機時間
//		ginko_timer_.usleepCyclic((return_packet_size_ * 87));		//受信待機時間
	}
}
int GinkoSerial::readRxBufferReadySize(void) {
	int bytes_available = 0;
	ioctl(fd_, FIONREAD, &bytes_available);
	return bytes_available;
}
void GinkoSerial::updateRxRingBuffer(void) {
	int bytes_available = readRxBufferReadySize();
//	ROS_INFO("bytes_available : %d",bytes_available);
	if (bytes_available > 0) {
		unsigned char copy_buf[bytes_available];
		receive_packet((void*) copy_buf, bytes_available);
		for (int j = 0; j < bytes_available; j++) {
			ring_buffer_[ring_wp_] = copy_buf[j];
			ring_wp_ = (ring_wp_ + 1) % RxRingBufferLength;
		}
	}
}
int GinkoSerial::readRingBufferReadySize(void){
	int buffer_instock = 0;
	if (ring_wp_ < ring_rp_) {
		buffer_instock = RxRingBufferLength + ring_wp_ - ring_rp_ - 1;
	} else {
		buffer_instock = ring_wp_ - ring_rp_ - 1;
	}
	return buffer_instock;
}
int GinkoSerial::ringBufferGotoOldestHeader(void){//バグりそう //何も見つからない場合は0を返す //
	int buffer_instock = readRingBufferReadySize();
	int servo_id = 0;
	for (int j = 0; j < (buffer_instock + 1 - return_packet_size_); j++) {
		if (ring_buffer_[(ring_rp_ + j + 1) % RxRingBufferLength] == 0xFD
				&& ring_buffer_[(ring_rp_ + j + 2) % RxRingBufferLength]== 0xDF
				&& ring_buffer_[(ring_rp_ + j + 3) % RxRingBufferLength] != 0x00) {
			unsigned char cs = 0x00;
			for (int i = 2; i < (return_packet_size_ - 1); i++) {
				cs ^= ring_buffer_[(ring_rp_ + j + 1 + i) % RxRingBufferLength];
			}
			if(cs == ring_buffer_[(ring_rp_ + j + 1 + (return_packet_size_ - 1) ) % RxRingBufferLength]){
				servo_id = ring_buffer_[(ring_rp_ + j + 3) % RxRingBufferLength];
				ring_rp_ = (ring_rp_ + j) % RxRingBufferLength;
//				ROS_WARN("Return Packet Detected id:%2d", servo_id);
				return servo_id;
			}
		}
	}
	return 0;
}
void GinkoSerial::getOldestPacketAndIncrementRing(void){
	unsigned char copy_buf[return_packet_size_]={};
	for (int j = 0; j < return_packet_size_; j++) {
		copy_buf[j] = ring_buffer_[(ring_rp_ + j + 1) % RxRingBufferLength];
	}
	int servo_id = copy_buf[2];
	if(servo_id > 0 && servo_id<=SERVO_NUM){
		double pose  =((double) ((int16_t) copy_buf[7]  + (int16_t) (copy_buf[8]  << 8)) * M_PI / 1800); //pose
		double speed =((double) ((int16_t) copy_buf[11] + (int16_t) (copy_buf[12] << 8)) * M_PI / 1800); //speed
		double effort=((double) ((int16_t) copy_buf[13] + (int16_t) (copy_buf[14] << 8)) *2.    /1000.);//電流トルク定数がわからない.1Aで2Nmと仮定
		if(tx_pose_[servo_id]>pose){
		}else{
			effort *= -1.0;
		}

		rx_pose_[servo_id-1] = pose;
		rx_vel_ [servo_id-1] = speed;
		rx_torque_[servo_id-1] = effort;

////		if(servo_id==2){
//			ROS_WARN("Return Packet Detected id:"
//					"%x %x %x %x %x "
//					"%x %x %x %x %x "
//					"%x %x %x %x %x "
//					"%x %x %x %x %x "
//					"%x %x %x %x %x %x"
//					,copy_buf[0],copy_buf[1],copy_buf[2],copy_buf[3],copy_buf[4]
//					,copy_buf[5],copy_buf[6],copy_buf[7],copy_buf[8],copy_buf[9]
//					,copy_buf[10],copy_buf[11],copy_buf[12],copy_buf[13],copy_buf[14]
//					,copy_buf[15],copy_buf[16],copy_buf[17],copy_buf[18],copy_buf[19]
//					,copy_buf[20],copy_buf[21],copy_buf[22],copy_buf[23],copy_buf[24],copy_buf[25]
//					);
////		}
		ring_rp_ = (ring_rp_ + 1) % RxRingBufferLength; //1バイト進めておけばヘッダの0xFA 0xAFの並びが崩れるから次の処理で1パケット分進む
	}

}
double GinkoSerial::readServoPosition(int servo_id){
	return rx_pose_[servo_id-1];
}
double GinkoSerial::readServoVelocity(int servo_id){
	return rx_vel_[servo_id-1];
}
double GinkoSerial::readServoTorque(int servo_id){
	return rx_torque_[servo_id-1];
}
void GinkoSerial::send_packet(void* ptr, int size) {
    if(write(fd_, ptr, size) < 0) {
        ROS_ERROR("Writing failed: %s(%s)", port_name_.data(), std::strerror(errno));
    }
//    usleep(10);

    return;
}
void GinkoSerial::receive_packet(void *buf_ptr, int size) {
    if(read(fd_, buf_ptr, size) < 0) {
        ROS_ERROR("Reading failed: %s(%s)", port_name_.data(), std::strerror(errno));
    }
}
int GinkoSerial::get_fd() {
    return fd_;
}




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

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_controller");
	GinkoController ginko_controller; //宣言のタイミングでginko_controllerの初期化が呼ばれる。
	ros::Rate loop_rate(LOOP_FREQUENCY);

	while (ros::ok()) {
		ginko_controller.control_loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
