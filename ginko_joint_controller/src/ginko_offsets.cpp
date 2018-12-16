
//2つのサーボで一自由度を駆動する際の、サーボ同士のズレを補正するための機能。
//dynamic reconfigureで確認しながら調整→servo_offsets.yamlに反映？のような形にしたい


#include <ginko_offsets.h>



//GinkoControler here
GinkoOffsets::GinkoOffsets(){
	initOffsetsReconfigure();
	ROS_INFO("GinkoOffsetsReconfigure : Init OK!");
}

GinkoOffsets::~GinkoOffsets() {

}

void GinkoOffsets::initOffsetsReconfigure() {
//	ここに宣言すると共有化に失敗してコールバックが呼ばれない。プライベート変数に入れた。
//	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig> param_server;
//	dynamic_reconfigure::Server<ginko_joint_controller::servo_offsetsConfig>::CallbackType callback_server;
	callback_server = boost::bind(&GinkoOffsets::offsetsReconfigureCallback,this, _1, _2);
	ROS_INFO("Reconfigure Initializad");
	param_server.setCallback(callback_server);
}
void GinkoOffsets::offsetsReconfigureCallback(ginko_joint_controller::servo_offsetsConfig &config, uint32_t level) {
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



