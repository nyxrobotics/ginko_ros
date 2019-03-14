
//2つのサーボで一自由度を駆動する際の、サーボ同士のズレを補正するための機能。
//dynamic reconfigureで確認しながら調整→servo_offsets.yamlに反映？のような形にしたい


#include <offsets_reconf.h>



//GinkoControler here
GinkoOffsets::GinkoOffsets(){
//	reconfigureOffsetsClient();
//	initOffsetsReconfigure();
//	initPublisher();
	initSubscriber();
	ROS_INFO("OffsetsReconfigureClient : Init OK!");

}

GinkoOffsets::~GinkoOffsets() {

}
void GinkoOffsets::initPublisher() {
//	joint_states_ofs_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states_ofs", 1);
//	goal_joint_position_ofs_pub_ = node_handle_.advertise<sensor_msgs::JointState>("goal_joint_position_ofs", 1);
}
void GinkoOffsets::initSubscriber() {
	joint_states_sub_ = node_handle_.subscribe("joint_states_in", 1,&GinkoOffsets::getJointStatesCallback, this);
//	goal_joint_position_sub_  = node_handle_.subscribe("goal_joint_position_in",1, &GinkoOffsets::getGoalJointCallback, this);
	init_flag_sub_ = node_handle_.subscribe("ofs_init_in", 1,&GinkoOffsets::getInitFlagCallback, this);
}
void GinkoOffsets::initOffsetsReconfigure() {

	ROS_INFO("Reconfigure Client Initializad");
	param_server.setCallback(callback_server);
}


void GinkoOffsets::getJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	for (int servo_index = 0; servo_index < SERVO_NUM; servo_index++){
		for (int buffer_index = JOINT_BUFFER_NUM-1 ; buffer_index > 0; buffer_index--){ //バッファを進める
			servo_states_buffer_[servo_index][buffer_index] = servo_states_buffer_[servo_index][buffer_index - 1];
		}
		servo_states_buffer_[servo_index][0] = msg->position.at(servo_index); //最新の値を更新。データがサーボのID順に並んでいる前提。
	}
}

void GinkoOffsets::calcJointStatesMedian() {
	double joints_median[SERVO_NUM] {};

	for (int servo_index = 0; servo_index < SERVO_NUM; servo_index++){
		double median_buffer[JOINT_BUFFER_NUM] = {};
		for (int buffer_index = 0; buffer_index < JOINT_BUFFER_NUM; buffer_index++){
			median_buffer[buffer_index] = servo_states_buffer_[servo_index][buffer_index] ;
		}
		//median_bufferの中身をサイズ順にする
        for (int i = 0; i < JOINT_BUFFER_NUM - 1; i++) {
                int j = i;
                for (int k = i; k < JOINT_BUFFER_NUM; k++) {
                        if (median_buffer[k] < median_buffer[j]) j = k;
                }
                if (i < j) {
                        double v = median_buffer[i];
                        median_buffer[i] = median_buffer[j];
                        median_buffer[j] = v;
                }
        }
//        if(servo_index == 2){
//			ROS_INFO("median(servo-2): %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//					median_buffer[0],median_buffer[1],median_buffer[2],median_buffer[3],median_buffer[4],
//					median_buffer[5],median_buffer[6],median_buffer[7],median_buffer[8],median_buffer[9],
//					median_buffer[10],median_buffer[11],median_buffer[12],median_buffer[13],median_buffer[14],
//					median_buffer[15],median_buffer[16],median_buffer[17],median_buffer[18],median_buffer[19]);
//        }
        int middle = JOINT_BUFFER_NUM / 2;
        joints_median[servo_index] = median_buffer[middle];
        servo_states_[servo_index] = joints_median[servo_index];
//        ROS_INFO("result: %f %f",joints_median[index],servo_states_[index]);
	}
}

void GinkoOffsets::calcJointOffsets() {
	calcJointStatesMedian();
	double joints_median[SERVO_NUM] {};
	for (int index = 0; index < SERVO_NUM; index++){
		servo_offsets_calib_[index] = 0.;
	}
//	ROS_INFO("median: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//			servo_states_[0],servo_states_[1],servo_states_[2],servo_states_[3],servo_states_[4],
//			servo_states_[5],servo_states_[6],servo_states_[7],servo_states_[8],servo_states_[9],
//			servo_states_[10],servo_states_[11],servo_states_[12],servo_states_[13],servo_states_[14],
//			servo_states_[15],servo_states_[16],servo_states_[17],servo_states_[18],servo_states_[19]);


	double diff_tmp;
	//右足首
	diff_tmp = 0. - servo_states_[0];
	servo_offsets_calib_[0] += diff_tmp;
	diff_tmp = 0. - servo_states_[1];
	servo_offsets_calib_[1] += diff_tmp;
	//右脛
	diff_tmp = 0.7854 - servo_states_[2];
	servo_offsets_calib_[2] += diff_tmp;
	servo_offsets_calib_[3] += diff_tmp;
	servo_offsets_calib_[3] += (servo_states_[2] - servo_states_[3]);
	//右腿
	diff_tmp = (-0.7854 - servo_states_[4]);
	servo_offsets_calib_[4] += diff_tmp;
	servo_offsets_calib_[5] += diff_tmp;
	servo_offsets_calib_[5] += (servo_states_[4] - servo_states_[5]);

	//左足首
	diff_tmp = 0. - servo_states_[7];
	servo_offsets_calib_[7] += diff_tmp;
	diff_tmp = 0. - servo_states_[8];
	servo_offsets_calib_[8] += diff_tmp;
	//左脛
	diff_tmp = (-0.7854 - servo_states_[9]);
	servo_offsets_calib_[9] += diff_tmp;
	servo_offsets_calib_[10] += diff_tmp;
	servo_offsets_calib_[10] += (servo_states_[9] - servo_states_[10]);
	//左腿
	diff_tmp = 0.7854 - servo_states_[11];
	servo_offsets_calib_[11] += diff_tmp;
	servo_offsets_calib_[12] += diff_tmp;
	servo_offsets_calib_[12] += (servo_states_[11] - servo_states_[12]);

	//右肩
	diff_tmp = 1.1781 - servo_states_[16];
	servo_offsets_calib_[16] += diff_tmp;
	servo_offsets_calib_[17] -= diff_tmp;
	servo_offsets_calib_[17] += (-servo_states_[16] - servo_states_[17]);
	//左肩
	diff_tmp = -1.1781 - servo_states_[21];
	servo_offsets_calib_[21] += diff_tmp;
	servo_offsets_calib_[22] -= diff_tmp;
	servo_offsets_calib_[22] += (-servo_states_[21] - servo_states_[22]);
}

void GinkoOffsets::reconfigureOffsetsClient() {
//	for (int index = 0; index < SERVO_NUM; index++){
//		servo_offsets_calib_[index] = (double)index / 100.0;
//	}
	calcJointOffsets();
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;
	for (int index = 0; index < SERVO_NUM; index++){
        std::stringstream paramName;
        int num_tmp = index + 1;
        //reference: http://www.geocities.jp/eneces_jupiter_jp/cpp1/013-001.html
        paramName << "servo_" << std::setw(2) << std::setfill( '0' ) << num_tmp << "_ofs";
//        ROS_INFO("%s",paramName.str().c_str());
    	double_param.name = paramName.str().c_str();
//    	double_param.value = 1.14514;
    	double_param.value = servo_offsets_calib_[index];
    	conf.doubles.push_back(double_param);
	}

	srv_req.config = conf;
//    ROS_INFO("call reconfigure client start ->");
    if (ros::service::call("/servo_motor/ginko_joint_offset/set_parameters", srv_req, srv_resp)) {
//       ROS_INFO("call to set ar_track_alvar parameters succeeded");
     } else {
       ROS_INFO("call to set ar_track_alvar parameters failed");
     }
//    ROS_INFO("call reconfigure client end <-");
}
void GinkoOffsets::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	init_flag = msg -> data;
	reconfigureOffsetsClient();
}
