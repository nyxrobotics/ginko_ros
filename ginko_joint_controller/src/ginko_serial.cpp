#include "ginko_serial.h"

//GinkoSerial here
GinkoSerial::GinkoSerial() {
	ginko_timer_.usecStart();
	//paramはstringを呼ぶ時のみ<std::string>を書く必要がある
	//http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters
//	std::string device_name = node_handle_.param<std::string>(ros::this_node::getName() + "/device_name", "/dev/ttyUSB0");
	unsigned long baudrate = node_handle_.param(ros::this_node::getName() + "/baud_rate", 460800);
//	portOpen(device_name,460800);
//	portOpen(device_name, baudrate);
	for(int i=0;i<ginko_params_._com_count;i++){
		//write pointを先頭、read pointをその次の場所で初期化
		ring_rp_[i]=0;
		ring_wp_[i]=1;
		//ポートオープン
		std::string portname = ginko_params_._com_names[i];
//		ROS_INFO("%s",portname.c_str());
		portOpen(i, portname, ginko_params_._baudrate);
//		portOpen(i, ginko_params_._com_names[i], ginko_params_._baudrate);
//		ginko_timer_.msleepSpan(250);
	}
	ginko_timer_.msleepSpan(100);
}
//GinkoSerial::GinkoSerial(std::string port_name, unsigned long baudrate) {
//	ginko_timer_.usecStart();
//	portOpen(port_name, baudrate);
//}
GinkoSerial::~GinkoSerial() {
//	portClose();
	int i;
	for(i=0;i<ginko_params_._com_count;i++){
		portClose(i);
		ginko_timer_.msleepSpan(250);
	}
}
void GinkoSerial::check_error(const char* action, int status) {
	if (status < 0) {
		ROS_ERROR("%s failed: %s(%s)", action, "test",
				std::strerror(errno));
		exit(-1);
	}
	return;
}
void GinkoSerial::portOpen(unsigned char com_num, std::string port_name, unsigned long baudrate) {
	std::string tmp_port_name_;
	tmp_port_name_ += port_name;
	baud_ = baudrate;
	fd_[com_num] = open(tmp_port_name_.data(), O_RDWR | O_NONBLOCK);
//	fd_[com_num] = open(port_name, O_RDWR | O_NONBLOCK);
	check_error("Open", fd_[com_num]);
	check_error("Save", tcgetattr(fd_[com_num], &tio_backup_[com_num]));
	// save the current port status

	std::memset(&tio_[com_num], 0, sizeof(tio_[com_num]));
	tio_[com_num].c_cc[VMIN] = 0;
	tio_[com_num].c_cc[VTIME] = 1;
	if (baudrate == 460800) {
		tio_[com_num].c_cflag = B460800 | CS8 | CREAD | CLOCAL;
	} else if (baudrate == 230400) {
		tio_[com_num].c_cflag = B230400 | CS8 | CREAD | CLOCAL;
	} else {
		tio_[com_num].c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	}

	tio_[com_num].c_iflag = IGNBRK | IGNPAR;	// init term-io
	check_error("Flash", tcflush(fd_[com_num], TCIOFLUSH));	// flush setting
	check_error("Set", tcsetattr(fd_[com_num], TCSANOW, &tio_[com_num]));	// change the port setting

	//Set Low-Latency mode //latency_timer->1ms
	struct serial_struct serial_settings;
	ginko_timer_.usleepSpan(10000);
	ioctl(fd_[com_num], TIOCGSERIAL, &serial_settings);
	serial_settings.flags |= ASYNC_LOW_LATENCY;
	ioctl(fd_[com_num], TIOCSSERIAL, &serial_settings);
	ginko_timer_.usleepSpan(10000);
	tcflush(fd_[com_num], TCIOFLUSH);//clear buffer
	ginko_timer_.usleepSpan(10000);
	ROS_INFO("USB connected! com:%d, name:%s",com_num,tmp_port_name_.c_str());
}
void GinkoSerial::portClose(unsigned char com_num) {
	check_error("Reset", tcsetattr(fd_[com_num], TCSANOW, &tio_backup_[com_num]));
	// restore the port setting
	close(fd_[com_num]);
	ginko_timer_.usleepSpan(100);
	ROS_INFO("USB disconnected! com:%d",com_num);
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


//	send_packet((void*) p, sizeof(p));
	for(int i=0;i<ginko_params_._com_count;i++){
		send_packet(i,(void*) p, sizeof(p));
	}
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
	//	send_packet((void*) p, sizeof(p));
	for(int i=0;i<ginko_params_._com_count;i++){
		send_packet(i,(void*) p, sizeof(p));
	}
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
	p1[13] = 0x00;    // check sum
	for (int i = 2; i < 13; i++) {
		p1[13] ^= p1[i];
	}
	//	send_packet((void*) p1, sizeof(p1));
	for(int i=0;i<ginko_params_._com_count;i++){
		send_packet(i,(void*) p1, sizeof(p1));
	}
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
//	send_packet((void*) p2, sizeof(p2));
	for(int comnum=0;comnum<ginko_params_._com_count;comnum++){
		send_packet(comnum,(void*) p2, sizeof(p2));
	}
	ginko_timer_.msleepSpan(10);
}
void GinkoSerial::sendTargetPosition(const double *value) {//*value:サーボID順の配列、目標角度
	int goal[SERVO_NUM];
	for (int i = 0; i < SERVO_NUM; i++) {
		double angle = value[i];
		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}
		tx_pose_[i] = angle;    //目標位置と現在位置を比較するときに使うためのバッファ
		goal[i] = (int) (3600 / (2 * M_PI) * angle);
	}
//	#pragma omp parallel for
	for(int comnum=0;comnum<ginko_params_._com_count;comnum++){//それぞれのポートでシリアル送信
		unsigned char servocount = ginko_params_._servo_count[comnum];
		int l = 8 + 3 * servocount;
		unsigned char p[l]; //配列の要素数はstatic constじゃないといけないと思っていたのだけど、なぜこれで行けるのか不明。
		p[0] = 0xFA;
		p[1] = 0xAF;
		p[2] = 0x00;            // ID  (ロングパケット時は0x00)
		p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
		p[4] = 0x1E;            // Adr
		p[5] = 0x03;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
		p[6] = servocount;		// Cnt (サーボの数)

		for (int i = 0; i < servocount; i++) {
			int id = ginko_params_._servo_id[comnum][i];//i + 1;
			p[7 + 3 * i] = id;   // each ID
			p[8 + 3 * i] = (unsigned char) (goal[id-1] & 0xFF);
			p[9 + 3 * i] = (unsigned char) (goal[id-1] >> 8 & 0xFF);
		}

		p[l - 1] = 0x00;    // check sum
		for (int j = 2; j < l - 1; j++) {
			p[l - 1] ^= p[j];
		}

		send_packet(comnum,(void*) p, sizeof(p));
		if (baud_ == 460800) {
			ginko_timer_.usleepSpan((100 + l * 87) / 4);
		} else if (baud_ == 230400) {
			ginko_timer_.usleepSpan((100 + l * 87) / 2);
		} else { //baud_==115200
			ginko_timer_.usleepSpan((100 + l * 87));
		}
	}


	return;
}
void GinkoSerial::sendTargetPositionWithSpeedSingleCom(const unsigned char comnum,const double *value, const double ms) {

	int goal[SERVO_NUM];
	int travel_time = ms / 10;

	for (int i = 0; i < SERVO_NUM; i++) {
		double angle = value[i];
		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}
		tx_pose_[i] = angle; //目標位置と現在位置を比較するときに使うためのバッファ
		goal[i] = (int) (3600 / (2 * M_PI) * angle);
	}
//	#pragma omp parallel for
//	for(int comnum=0;comnum<ginko_params_._com_count;comnum++){//それぞれのポートでシリアル送信
		unsigned char servocount = ginko_params_._servo_count[comnum];
		int l = 8 + 5 * servocount;
		unsigned char p[l];
		p[0] = 0xFA;
		p[1] = 0xAF;
		p[2] = 0x00;            // ID  (ロングパケット時は0x00)
		p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
		p[4] = 0x1E;            // Adr
		p[5] = 0x05;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
		p[6] = servocount;		// Cnt (サーボの数)

		for (int i = 0; i < servocount; i++) {
			p[7 + 5 * i] = i + 1;   // each ID
			p[8 + 5 * i] = (unsigned char) (goal[i] & 0xFF);
			p[9 + 5 * i] = (unsigned char) (goal[i] >> 8 & 0xFF);
			p[10 + 5 * i] = (unsigned char) (travel_time & 0xFF);
			p[11 + 5 * i] = (unsigned char) (travel_time >> 8 & 0xFF);
		}

		p[l - 1] = 0x00;    // check sum
		for (int j = 2; j < l - 1; j++) {
			p[l - 1] ^= p[j];
		}
		send_packet(comnum,(void*) p, sizeof(p));
		if (baud_ == 460800) {
			ginko_timer_.usleepSpan((100 + l * 100) / 4);
//			ginko_timer_.usleepSpan((10 + l * 100) / 4);
		} else if (baud_ == 230400) {
			ginko_timer_.usleepSpan((100 + l * 100) / 2);
		} else { //baud_==115200
			ginko_timer_.usleepSpan((100 + l * 100));
		}
//	}
	return;
}
void GinkoSerial::sendTargetPositionWithSpeed(const double *value, const double ms) {

	int goal[SERVO_NUM];
	int travel_time = ms / 10;

	for (int i = 0; i < SERVO_NUM; i++) {
		double angle = value[i];
		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}
		tx_pose_[i] = angle; //目標位置と現在位置を比較するときに使うためのバッファ
		goal[i] = (int) (3600 / (2 * M_PI) * angle);
	}
//	#pragma omp parallel for
	for(int comnum=0;comnum<ginko_params_._com_count;comnum++){//それぞれのポートでシリアル送信
		unsigned char servocount = ginko_params_._servo_count[comnum];
		int l = 8 + 5 * servocount;
		unsigned char p[l];
		p[0] = 0xFA;
		p[1] = 0xAF;
		p[2] = 0x00;            // ID  (ロングパケット時は0x00)
		p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
		p[4] = 0x1E;            // Adr
		p[5] = 0x05;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
		p[6] = servocount;		// Cnt (サーボの数)

		for (int i = 0; i < servocount; i++) {
			p[7 + 5 * i] = i + 1;   // each ID
			p[8 + 5 * i] = (unsigned char) (goal[i] & 0xFF);
			p[9 + 5 * i] = (unsigned char) (goal[i] >> 8 & 0xFF);
			p[10 + 5 * i] = (unsigned char) (travel_time & 0xFF);
			p[11 + 5 * i] = (unsigned char) (travel_time >> 8 & 0xFF);
		}

		p[l - 1] = 0x00;    // check sum
		for (int j = 2; j < l - 1; j++) {
			p[l - 1] ^= p[j];
		}
		send_packet(comnum,(void*) p, sizeof(p));
		if (baud_ == 460800) {
			ginko_timer_.usleepSpan((100 + l * 87) / 4);
		} else if (baud_ == 230400) {
			ginko_timer_.usleepSpan((100 + l * 87) / 2);
		} else { //baud_==115200
			ginko_timer_.usleepSpan((100 + l * 87));
		}
	}
	return;
}
void GinkoSerial::switchAllTorque(bool sw) {
	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;    // ID
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
//	#pragma omp parallel for
	for(int comnum=0;comnum<ginko_params_._com_count;comnum++){
		send_packet(comnum,(void*) p, sizeof(p));
	}
	if (baud_ == 460800) {
		ginko_timer_.usleepSpan(300);
	} else if (baud_ == 230400) {
		ginko_timer_.usleepSpan(600);
	} else { //baud_==115200
		ginko_timer_.usleepSpan(1200);
	}
	return;
}
void GinkoSerial::switchAllTorque_com(unsigned char comnum, bool sw) {
	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;    // ID
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

	send_packet(comnum,(void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepSpan(300);
	} else if (baud_ == 230400) {
		ginko_timer_.usleepSpan(600);
	} else { //baud_==115200
		ginko_timer_.usleepSpan(1200);
	}
	return;
}
void GinkoSerial::switchTorque(unsigned char servo_id, bool sw) {
	if (servo_id == 0xFF){
		switchAllTorque(sw);
		return;
	}
	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = servo_id;      // ID
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

	int com_select=0;//それぞれのidがどのポートに該当するか探す。計算効率が悪い。
	for(int comnum=0; comnum<ginko_params_._com_count; comnum++){
		for(int servonum=0;servonum<ginko_params_._servo_count[comnum];servonum++){
			if(ginko_params_._servo_id[comnum][servonum]==servo_id){
				com_select = comnum;
				//ループ抜ける→バグったので無視
				//servonum = ginko_params_._servo_count[comnum];
				//comnum = ginko_params_._com_count;
			}
		}
	}

	send_packet(com_select,(void*) p, sizeof(p));

	if (baud_ == 460800) {
		ginko_timer_.usleepSpan(300);
	} else if (baud_ == 230400) {
		ginko_timer_.usleepSpan(600);
	} else { //baud_==115200
		ginko_timer_.usleepSpan(1200);
	}
	return;
}
int GinkoSerial::requestReturnPacket(int servo_id) {
	//ROS_INFO("request : %d",servo_id);
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
	int com_select=0;//それぞれのidがどのポートに該当するか探す。計算効率が悪い。
	for(int comnum=0; comnum<ginko_params_._com_count; comnum++){
		for(int servonum=0;servonum<ginko_params_._servo_count[comnum];servonum++){
			if(ginko_params_._servo_id[comnum][servonum]==servo_id){
				com_select = comnum;
				//ループ抜ける→バグったので無視
				//servonum = ginko_params_._servo_count[comnum];
				//comnum = ginko_params_._com_count;
			}
		}
	}
	send_packet(com_select, (void*) p, sizeof(p));
	if (baud_ == 460800) {
		ginko_timer_.usleepSpan(1000);
	} else if (baud_ == 230400) {
		ginko_timer_.usleepSpan(2100);
	} else { //baud_==115200
		ginko_timer_.usleepSpan(3200);
	}
	//ROS_INFO("request : %d -> finish,comnum = %d",servo_id,com_select);
	return com_select;
}
int GinkoSerial::readRxBufferReadySize(unsigned char com_num) {
	int bytes_available = 0;
	ioctl(fd_[com_num], FIONREAD, &bytes_available);
	return bytes_available;
}
void GinkoSerial::updateRxRingBuffer(unsigned char com_num) {
	int bytes_available = readRxBufferReadySize(com_num);
//	ROS_INFO("bytes_available : %d",bytes_available);
	if (bytes_available > 0) {
		unsigned char copy_buf[bytes_available];
		receive_packet(com_num, (void*) copy_buf, bytes_available);
		for (int j = 0; j < bytes_available; j++) {
			ring_buffer_[com_num][ring_wp_[com_num]] = copy_buf[j];
			ring_wp_[com_num] = (ring_wp_[com_num] + 1) % RxRingBufferLength;
		}
	}
}
int GinkoSerial::readRingBufferReadySize(unsigned char com_num) {
	int buffer_instock = 0;
	if (ring_wp_[com_num] < ring_rp_[com_num]) {
		buffer_instock = RxRingBufferLength + ring_wp_[com_num] - ring_rp_[com_num] - 1;
	} else {
		buffer_instock = ring_wp_[com_num] - ring_rp_[com_num] - 1;
	}
	return buffer_instock;
}
int GinkoSerial::ringBufferGotoOldestHeader(unsigned char com_num) { //バグりそう //何も見つからない場合は0を返す //
	int buffer_instock = readRingBufferReadySize(com_num);
	int servo_id = 0;
	for (int j = 0; j < (buffer_instock + 1 - return_packet_size_); j++) {
		if (ring_buffer_[com_num][(ring_rp_[com_num] + j + 1) % RxRingBufferLength ] == 0xFD
				&& ring_buffer_[com_num][(ring_rp_[com_num] + j + 2) % RxRingBufferLength ] == 0xDF
				&& ring_buffer_[com_num][(ring_rp_[com_num] + j + 3) % RxRingBufferLength ]
						!= 0x00) {
			unsigned char cs = 0x00;
			for (int i = 2; i < (return_packet_size_ - 1); i++) {
				cs ^= ring_buffer_[com_num][(ring_rp_[com_num] + j + 1 + i) % RxRingBufferLength];
			}
			if (cs
					== ring_buffer_[com_num][(ring_rp_[com_num] + j + 1
							+ (return_packet_size_ - 1)) % RxRingBufferLength]) {
				servo_id =
						ring_buffer_[com_num][(ring_rp_[com_num] + j + 3) % RxRingBufferLength];
				ring_rp_[com_num] = (ring_rp_[com_num] + j) % RxRingBufferLength;
//				ROS_WARN("Return Packet Detected id:%2d", servo_id);
				return servo_id;
			}
		}
	}
	return 0;
}
void GinkoSerial::getOldestPacketAndIncrementRing(unsigned char com_num) {
	unsigned char copy_buf[return_packet_size_] = { };
	for (int j = 0; j < return_packet_size_; j++) {
		copy_buf[j] = ring_buffer_[com_num][(ring_rp_[com_num] + j + 1) % RxRingBufferLength ];
	}
	int servo_id = copy_buf[2];
	if (servo_id > 0 && servo_id <= SERVO_NUM) {
		double pose = ((double) ((int16_t) copy_buf[7]
				+ (int16_t) (copy_buf[8] << 8)) * M_PI / 1800); //pose
		double speed = ((double) ((int16_t) copy_buf[11]
				+ (int16_t) (copy_buf[12] << 8)) * M_PI / 1800); //speed
		double effort = ((double) ((int16_t) copy_buf[13]
				+ (int16_t) (copy_buf[14] << 8)) * 2. / 1000.); //電流トルク定数がわからない.1Aで2Nmと仮定
		if (tx_pose_[servo_id] > pose) {
		} else {
			effort *= -1.0;
		}

		rx_pose_[servo_id - 1] = pose;
		rx_vel_[servo_id - 1] = speed;
		rx_torque_[servo_id - 1] = effort;

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
		ring_rp_[com_num] = (ring_rp_[com_num] + 1) % RxRingBufferLength ; //1バイト進めておけばヘッダの0xFA 0xAFの並びが崩れるから次の処理で1パケット分進む
	}

}
double GinkoSerial::readServoPosition(int servo_id) {
	return rx_pose_[servo_id - 1];
}
double GinkoSerial::readServoVelocity(int servo_id) {
	return rx_vel_[servo_id - 1];
}
double GinkoSerial::readServoTorque(int servo_id) {
	return rx_torque_[servo_id - 1];
}
void GinkoSerial::send_packet(unsigned char com_num, void* ptr, int size) {
	if (write(fd_[com_num], ptr, size) < 0) {
		ROS_ERROR("Writing failed: %s(%s)", ginko_params_._com_names[com_num].c_str(),
				std::strerror(errno));
	}
//    usleep(10);

	return;
}
void GinkoSerial::receive_packet(unsigned char com_num, void *buf_ptr, int size) {
	if (read(fd_[com_num], buf_ptr, size) < 0) {
		ROS_ERROR("Reading failed: %s(%s)", ginko_params_._com_names[com_num].c_str(),
				std::strerror(errno));
	}
}
int GinkoSerial::get_fd(unsigned char com_num) {
	return fd_[com_num];
}
