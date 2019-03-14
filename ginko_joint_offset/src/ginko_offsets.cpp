
//2つのサーボで一自由度を駆動する際の、サーボ同士のズレを補正するための機能。
//dynamic reconfigureで確認しながら調整→servo_offsets.yamlに反映？のような形にしたい


#include <ginko_offsets.h>



//GinkoControler here
GinkoOffsets::GinkoOffsets(){
//	reconfigureOffsetsClient();
	initOffsetsReconfigure();
	initPublisher();
	initSubscriber();
	ROS_INFO("GinkoOffsetsReconfigure : Init OK!");

}

GinkoOffsets::~GinkoOffsets() {

}
void GinkoOffsets::initPublisher() {
	joint_states_ofs_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states_ofs", 1);
	goal_joint_position_ofs_pub_ = node_handle_.advertise<sensor_msgs::JointState>("goal_joint_position_ofs", 1);
}
void GinkoOffsets::initSubscriber() {
	joint_states_sub_ = node_handle_.subscribe("joint_states_in", 1,&GinkoOffsets::getJointStatesCallback, this);
	goal_joint_position_sub_  = node_handle_.subscribe("goal_joint_position_in",1, &GinkoOffsets::getGoalJointCallback, this);
//	init_flag_sub_ = node_handle_.subscribe("ofs_init_in", 1,&GinkoOffsets::getInitFlagCallback, this);
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
	ROS_INFO("Reconfigure Request: %f %f %f %f %f",config.servo_01_ofs,config.servo_02_ofs,config.servo_03_ofs,config.servo_04_ofs,config.servo_05_ofs);


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

void GinkoOffsets::getJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	for (int index = 0; index < SERVO_NUM; index++){
		servo_states_[index] = msg->position.at(index);
		joint_states_[index] = servo_states_[index] + servo_offsets_[index];
	}
	sensor_msgs::JointState joint_state_in = *msg;

	sensor_msgs::JointState joint_state_out;
	joint_state_out.header.frame_id = joint_state_in.header.frame_id;
	joint_state_out.header.stamp = joint_state_in.header.stamp;
	joint_state_out.name = joint_state_in.name;
	joint_state_out.velocity = joint_state_in.velocity;
	joint_state_out.effort = joint_state_in.effort;
	//サーボのID順に書く必要アリ
	for (int index = 0; index < SERVO_NUM; index++) {
		joint_state_out.position.push_back(joint_states_[index]);
	}

	joint_states_ofs_pub_.publish(joint_state_out);

//	static int counter = 0;
//	if(counter == 100){
//		reconfigureOffsetsClient();
//		counter = 0;
//	}else{
//		counter ++;
//	}
}
void GinkoOffsets::getGoalJointCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	for (int index = 0; index < SERVO_NUM; index++){
		joint_goal_[index] = msg->position.at(index);//送られてくるjointの目標値の数が少ないと配列外参照になるので注意
		servo_goal_[index] = joint_goal_[index] - servo_offsets_[index];
	}
	sensor_msgs::JointState joint_state_in = *msg;

	sensor_msgs::JointState joint_state_out;
	joint_state_out.header.frame_id = joint_state_in.header.frame_id;
	joint_state_out.header.stamp = joint_state_in.header.stamp;
	joint_state_out.name = joint_state_in.name;
	joint_state_out.velocity = joint_state_in.velocity;
	joint_state_out.effort = joint_state_in.effort;
	//サーボのID順に書く必要アリ
	for (int index = 0; index < SERVO_NUM; index++) {
		joint_state_out.position.push_back(servo_goal_[index]);
	}
	goal_joint_position_ofs_pub_.publish(joint_state_out);
}
void GinkoOffsets::reconfigureOffsetsClient() {
	for (int index = 0; index < SERVO_NUM; index++){
		servo_offsets_calib_[index] = (double)index / 100.0;
	}
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;
	for (int index = 0; index < SERVO_NUM; index++){
        std::stringstream paramName;
        int num_tmp = index + 1;
        //reference: http://www.geocities.jp/eneces_jupiter_jp/cpp1/013-001.html
        paramName << "servo_" << std::setw(2) << std::setfill( '0' ) << num_tmp << "_ofs";
        ROS_INFO("%s",paramName.str().c_str());
    	double_param.name = paramName.str().c_str();
    	double_param.value = 1.14514;
    	conf.doubles.push_back(double_param);
	}

	srv_req.config = conf;
    ROS_INFO("call reconfigure client start ->");
//	ros::service::call("/servo_motor/ginko_joint_offset/set_parameters", srv_req, srv_resp);
//	ros::service::call("set_parameters", srv_req, srv_resp);
//	node_handle_.setParam("/servo_motor/ginko_joint_offset/servo_01_ofs", 1.14514);

    if (ros::service::call("/servo_motor/ginko_joint_offset/set_parameters", srv_req, srv_resp)) {
       ROS_INFO("call to set ar_track_alvar parameters succeeded");
     } else {
       ROS_INFO("call to set ar_track_alvar parameters failed");
     }
    ROS_INFO("call reconfigure client end <-");
}
void GinkoOffsets::getInitFlagCallback(const std_msgs::Int32::ConstPtr& msg){
	init_flag = msg -> data;
//	reconfigureOffsetsClient();
}
