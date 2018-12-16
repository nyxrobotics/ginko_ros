
#include "ginko_serial.h"


//GinkoSerial here
GinkoSerial::GinkoSerial() {
	ginko_timer_.usecStart();
//	portOpen("/dev/ttyUSB0",115200);
//	portOpen("/dev/ttyUSB0",460800);
	std::string device_name = node_handle_.param<std::string>(ros::this_node::getName() + "/device_name", "/dev/ttyUSB1");
	portOpen(device_name,460800);
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
