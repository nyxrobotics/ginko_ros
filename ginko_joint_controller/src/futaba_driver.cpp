#include "futaba_driver.hpp"
#include "serial.hpp"

PLUGINLIB_EXPORT_CLASS(ginko_joint::revolute, nodelet::Nodelet)

namespace ginko_joint {
revolute::revolute() {
}
revolute::~revolute() {
}
void revolute::onInit() {
	od.reset(new futaba_driver(getNodeHandle(), getPrivateNodeHandle()));
}
}

futaba_driver::futaba_driver(ros::NodeHandle nh, ros::NodeHandle prv_nh) {
	init(nh);
	std::string pn;

	bool flag = nh.getParam("futaba_driver/count", count);
	flag &= nh.getParam("futaba_driver/port", pn);

	std::string tc_str = "futaba_driver/torque_const_";

	for (int i = 0; i < count; i++) {
		std::ostringstream num;
		num << i + 1;
		flag &= nh.getParam((tc_str + num.str()).data(), torque_const[i]);
	}

	if (flag) {
		ROS_INFO("Finish Reading Parameters.");
	} else {
		ROS_ERROR("Reading Parameters Failed.");
	}

	sp.reset(new serial_port(pn));
	ginko_usleep(100000);
//	servo_reboot(255);
 	switch_torque(255,true);
	ginko_usleep(100000);
//	set_baud_115200();
}

futaba_driver::~futaba_driver() {
	switch_torque(255,false);
	ginko_usleep(100000);
//	servo_reboot(255);
//	usleep(100000);
}

void futaba_driver::init(ros::NodeHandle nh) {
	present_states_pub = nh.advertise<sensor_msgs::JointState>(
			"futaba/present_states", 1);
	goal_states_sub = nh.subscribe("futaba/goal_state", 1,
			&futaba_driver::update_position, this);

	ROS_INFO("Finish Initialization.");

	return;
}

void futaba_driver::write_position(int *position) {
	int l = 8 + 3 * count;
	unsigned char p[l];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0x00;            // ID  (ロングパケット時は0x00)
	p[3] = 0x00;   			// Flg (ロングパケット時は0x00)
	p[4] = 0x1E;            // Adr
	p[5] = 0x03;			// Len (サーボ１個あたりのデータ長, ID(1byte) + Data(2byte))
	p[6] = count;			// Cnt (サーボの数)
	for (int i = 0; i < count; i++) {
		p[7 + 3 * i] = i + 1;   // each ID
		p[8 + 3 * i] = (unsigned char) (position[i] & 0xFF);
		p[9 + 3 * i] = (unsigned char) (position[i] >> 8 & 0xFF);
	}
	p[l - 1] = 0x00;    // check sum
	for (int j = 2; j < l - 1; j++) {
		p[l - 1] ^= p[j];
	}

	sp->send_packet((void*) p, sizeof(p));

	return;
}

bool futaba_driver::get_status(std::vector<double> *position,
		std::vector<double> *torque) {
	unsigned char p[8], buf[26];
	std::memset(buf, 0, sizeof(buf));
	int bytes_available = 0;
	int fd = sp->get_fd();

	for (int i = 0; i < count; i++) {

		p[0] = 0xFA;
		p[1] = 0xAF;
		p[2] = i + 1;   // ID
		p[3] = 0x09;    // Flg
		p[4] = 0x00;    // Adr
		p[5] = 0x00;    // Len(0x00:all)
		p[6] = 0x01;    // Cnt
		p[7] = 0x00;    // check sum
		for (int j = 2; j < 7; j++) {
			p[7] ^= p[j];
		}
		ginko_usleep(2000);
		sp->send_packet((void*) p, sizeof(p));
		ginko_usleep(2000);



		//Waits for all data get ready
//		ros::Time begin = ros::Time::now();
//		ros::Time now = ros::Time::now();
//		double dt = (now - begin).toSec();
//		while (bytes_available < sizeof(buf)) {				//パケットが来るまで待機
//			ioctl(fd, FIONREAD, &bytes_available);
//			now = ros::Time::now();
//			dt = (now - begin).toSec();
//			if(dt > 0.004){
//				ROS_ERROR("Return Packet Timed out:id%2d", i + 1);
//				return false;
//			}
//		}
//		ginko_usleep(4000);
		ioctl(fd, FIONREAD, &bytes_available);
		if(bytes_available > sizeof(buf)){					//余計なパケットが溜まっている時は最後だけ取り出す
			unsigned char learge_buf[bytes_available];		//配列の要素数を変数にして宣言するのっていつからできるようになったんだろう。便利だ。
			sp->receive_packet((void*) learge_buf, bytes_available);
			for(int j=0; j<sizeof(buf); j++){
				buf[sizeof(buf) - j - 1] = learge_buf[bytes_available - j - 1];
			}
			ROS_ERROR("Receiving Buffer Full Warn:id%2d", i + 1);
		}else{												//正常なとき
			sp->receive_packet((void*) buf, sizeof(buf));
		}

		// calculate check sum of return packet.
		unsigned char cs = 0x00;
		for (int i = 2; i < (sizeof(buf) - 1); i++) {
			cs ^= buf[i];
		}

		if (buf[0] != 0xFD || buf[1] != 0xDF || cs != buf[(sizeof(buf) - 1)]) {
			ROS_ERROR("Receiving Error:id%2d", i + 1);
			ROS_WARN("buf : %x,%x,%x,%x,%x,%x,%x,%x,%x,%x",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9]);
//			return false;
		}else{

			position->push_back((double) ((int16_t)buf[7] + (int16_t)(buf[8] << 8) ) * M_PI / 1800); //pose

			int current = buf[13] + buf[14] * 0xFF;    // the unit may be [mA]
			torque->push_back((double) current * torque_const[i]);
		}
//		ioctl(fd, TCFLSH, 0); // flush receive
//		ioctl(fd, TCFLSH, 1); // flush transmit
//		ioctl(fd, TCFLSH, 2); // flush both
	}

	return true;
}

void futaba_driver::switch_torque(unsigned char id, bool sw) {
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

	sp->send_packet((void*) p, sizeof(p));
	return;
}

void futaba_driver::servo_reboot(unsigned char id) {
	unsigned char p[8];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = id;      // ID
	p[3] = 0x20;    // Flg
	p[4] = 0xFF;    // Adr
	p[5] = 0x01;    // Len
	p[6] = 0x00;    // Cnt

	p[7] = 0x00;    // check sum
	for (int j = 2; j < 7; j++) {
		p[7] ^= p[j];
	}

	sp->send_packet((void*) p, sizeof(p));
	return;
}


void futaba_driver::update_position(
		const sensor_msgs::JointState::ConstPtr &js) {
//	switch_torque(255,true);
	int goal_position[count];

	for (int i = 0; i < count; i++) {
		double angle = js->position.at(i);

		if (angle < -2.6) {
			angle = -2.6;
		} else if (angle > 2.6) {
			angle = 2.6;
		}

		goal_position[i] = (int) (3600 / (2 * M_PI) * angle);
	}
	write_position(goal_position);
	publish_status();
	return;
}

void futaba_driver::publish_status() {

	sensor_msgs::JointState js_pub;
	std::vector<double> position, torque;

	if (get_status(&position, &torque)) {
		js_pub.position = position;
		js_pub.effort = torque;

		present_states_pub.publish(js_pub);
	} else {
		ROS_ERROR("Getting Status Failed");
	}

	return;
}

void futaba_driver::ginko_usleep(int usec) {

	  struct timespec ts;
	  ts.tv_sec = 0;
	  ts.tv_nsec = 1000*usec;

	  clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);

}

void futaba_driver::set_baud_115200(void) { //何かの間違いでボーレートを変えた時に115200に戻す

	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;
	p[3] = 0x00;
	p[4] = 0x06;
	p[5] = 0x01;
	p[6] = 0x01;
	p[7] = 0x07;
	p[8] = 0xFE;
	sp->send_packet((void*) p, sizeof(p));

	unsigned char p2[8];
	p2[0] = 0xFA;
	p2[1] = 0xAF;
	p2[2] = 0xFF;
	p2[3] = 0x40;
	p2[4] = 0xFF;
	p2[5] = 0x00;
	p2[6] = 0x00;
	p2[7] = 0x40;

	sp->send_packet((void*) p2, sizeof(p2));

	return;
}
void futaba_driver::set_baud_460800(void) { //何かの間違いでボーレートを変えた時に115200に戻す

	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;
	p[3] = 0x00;
	p[4] = 0x06;
	p[5] = 0x01;
	p[6] = 0x01;
	p[7] = 0x0A;
	p[8] = 0xF3;
	sp->send_packet((void*) p, sizeof(p));

	unsigned char p2[8];
	p2[0] = 0xFA;
	p2[1] = 0xAF;
	p2[2] = 0xFF;
	p2[3] = 0x40;
	p2[4] = 0xFF;
	p2[5] = 0x00;
	p2[6] = 0x00;
	p2[7] = 0x40;

	sp->send_packet((void*) p2, sizeof(p2));

	return;
}
void futaba_driver::set_baud_230400(void) { //何かの間違いでボーレートを変えた時に115200に戻す

	unsigned char p[9];
	p[0] = 0xFA;
	p[1] = 0xAF;
	p[2] = 0xFF;
	p[3] = 0x00;
	p[4] = 0x06;
	p[5] = 0x01;
	p[6] = 0x01;
	p[7] = 0x09;
	p[8] = 0xF0;
	sp->send_packet((void*) p, sizeof(p));

	unsigned char p2[8];
	p2[0] = 0xFA;
	p2[1] = 0xAF;
	p2[2] = 0xFF;
	p2[3] = 0x40;
	p2[4] = 0xFF;
	p2[5] = 0x00;
	p2[6] = 0x00;
	p2[7] = 0x40;

	sp->send_packet((void*) p2, sizeof(p2));

	return;
}


