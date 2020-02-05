#include <battle_planner.h>

//BattlePlanner here
BattlePlanner::BattlePlanner(ros::NodeHandle main_nh){
	initTF2();
	readParams(main_nh);
	initSubscriber(main_nh);
	initPublisher(main_nh);
	battle_state_ = "off";
}

BattlePlanner::~BattlePlanner() {
	ros::shutdown();
}

void BattlePlanner::readParams(ros::NodeHandle node_handle_){
}

void BattlePlanner::initSubscriber(ros::NodeHandle node_handle_){
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	battle_command_sub_ = node_handle_.subscribe("battle_command", 10,&BattlePlanner::getBattleCommandCallback, this,transport_hints);
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 10,&BattlePlanner::getImuQuaternionCallback, this, transport_hints);
}

void BattlePlanner::initTF2(){
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(10.0), true));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	//sleep(10);//TFが安定するまで待つ
}

void BattlePlanner::initPublisher(ros::NodeHandle node_handle_){
	motion_command_pub_ = node_handle_.advertise<std_msgs::String>("motion_command_out", 10);
}

int BattlePlanner::mainLoop(){
	if(imu_ready_ == 0){
		usleep(100000);
		return 0;
	}
//	ROS_FATAL("Battle Planner: STANDBY ");
	if(battle_state_ == "off"){
		if(battle_command_.data == "on"){
			ROS_FATAL("Battle Planner: Send Command: TORQUE_ON");
			motion_command_.data = "TORQUE_ON";	motion_command_pub_.publish(motion_command_);
			usleep(100000);
			ROS_FATAL("Battle Planner: Send Command: STANDING");
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
			battle_state_ = "standby";
			battle_command_.data = "";
			sleep(5);
//			motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
			searchTarget(target_tf_, 5.0,true);
			// sleep(5);
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
		}else{
//			ROS_FATAL("Battle Planner: Send Command: TORQUE_OFF");
			motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
			battle_state_ = "off";
			battle_command_.data = "";
			usleep(100000);
			return 0;
		}
	}
	ros::spinOnce();
	if(battle_command_.data == "off"){
		ROS_FATAL("Battle Planner: Send Command: TORQUE_OFF");
		motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
		battle_state_ = "off";
		battle_command_.data = "";
		usleep(100000);
		return 0;
	}


	ros::spinOnce();
	static int imu_fall_direction_prev = imu_fall_direction_;
	if(imu_fall_direction_prev != 0 && imu_fall_direction_ == 0){
		sleep(3);
		ros::spinOnce();
		if(imu_fall_direction_ == 0){
//			motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
			searchTarget(target_tf_, 5.0,true);
			imu_fall_direction_prev = imu_fall_direction_;
			sleep(1);
		}
	}
	imu_fall_direction_prev = imu_fall_direction_;

	if(imu_fall_direction_ == 0){
		if(battle_command_.data == "battle"){
			battle_state_ = "battle";
			battle_command_.data = "";
		}
		battleMotionSelect();
		ROS_FATAL("Battle Planner: Status: BATTLE!!!");
		usleep(100000);
		return 0;
	}

	sleep(1);
	ros::spinOnce();
	if(battle_command_.data == "off"){
		ROS_FATAL("Battle Planner: Send Command: TORQUE_OFF");
		motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
		battle_state_ = "off";
		battle_command_.data = "";
		usleep(100000);
		return 0;
	}

	if(imu_fall_direction_ == 0){
		usleep(100000);
		return 0;
	}
	if(imu_fall_direction_ == 1){
		ROS_FATAL("Battle Planner: Send Command: WAKEUP_FRONT");
		motion_command_.data = "WAKEUP_FRONT";	motion_command_pub_.publish(motion_command_);
		sleep(3);
		 motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
//		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		// sleep(5);
		return 0;
	}
	if(imu_fall_direction_ == 2){
		ROS_FATAL("Battle Planner: Send Command: WAKEUP_BACK");
		motion_command_.data = "WAKEUP_BACK";	motion_command_pub_.publish(motion_command_);
		sleep(3);
		 motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
//		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		// sleep(5);
		return 0;
	}
	usleep(100000);
	return 0;
}



void BattlePlanner::getBattleCommandCallback(const std_msgs::String::ConstPtr& msg){
	battle_command_ready_ = 1;
	battle_command_update_ = 1;
	battle_command_ = *msg;
}

void BattlePlanner::getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
	imu_ready_ = 1;
	imu_quaternion_ = *msg;
	tf2::Quaternion tf2_quat(imu_quaternion_.orientation.x, imu_quaternion_.orientation.y, imu_quaternion_.orientation.z, imu_quaternion_.orientation.w);
	geometry_msgs::Vector3 euler;
	tf2::Matrix3x3(tf2_quat).getEulerYPR(euler.z, euler.y, euler.x);
	euler.z = 0.;
	euler.x = 0.;
	tf2::Vector3 single_z(0,0,1.0);
//	tf2::Matrix3x3 rotation_matrix(tf2_quat);
	tf2::Matrix3x3 rotation_matrix;
	rotation_matrix.setRPY(euler.x, euler.y, euler.z);
	tf2::Vector3 single_z_rot = rotation_matrix * single_z;
	double dx = single_z_rot.x();
	double dy = single_z_rot.y();
	double xy_norm_tmp = sqrt(dx*dx + dy*dy);

	if(dx > 0.4){
		imu_fall_direction_ = 1;
		motion_command_.data = "STANDING";
		motion_command_pub_.publish(motion_command_);
	}else if(dx < -0.4){
		imu_fall_direction_ = 2;
		motion_command_.data = "STANDING";
		motion_command_pub_.publish(motion_command_);
	}else{
		imu_fall_direction_ = 0;
	}
//	ROS_FATAL("Battle Planner: fall/stand: %d",imu_fall_direction_);
}

int BattlePlanner::battleMotionSelect(){

	int flag_tmp;
	while (SUCCEED != searchTarget(target_tf_, 5.0,false)){
		return 0;
	}

	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(robot_tf_, target_tf_, ros::Time(0));
	double dx = tf_diff.transform.translation.x;
	double dy = tf_diff.transform.translation.y;
	double area_theta = atan2(dy,dx);
	double area_distance = sqrt(dx*dx + dy*dy);
	//左右旋回
	if(fabs(area_theta)>area_angle_threth_ && fabs(area_theta)< (3.1416 - area_angle_threth_)){
		if(area_distance < 0.7){
			if( (area_theta < 0. && dx > 0.0)
			||(area_theta > 0. && dx < 0.0)
			){
				ROS_FATAL("Battle Planner: Send Command: TURN_RIGHT");
				motion_command_.data = "TURN_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_FATAL("Battle Planner: Send Command: TURN_LEFT");
				motion_command_.data = "TURN_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}else{
			if(area_theta < 0.){
				ROS_FATAL("Battle Planner: Send Command: WALK_RIGHT");
				motion_command_.data = "WALK_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_FATAL("Battle Planner: Send Command: WALK_LEFT");
				motion_command_.data = "WALK_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}
		return 0;
	}
	//前進・後退
	if(area_distance > area_distance_threth_){
		if(dx < 0.){
			ROS_FATAL("Battle Planner: Send Command: WALK_BACK");
			motion_command_.data = "WALK_BACK";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: WALK_FRONT");
			motion_command_.data = "WALK_FRONT";	motion_command_pub_.publish(motion_command_);
		}
		return 0;
	}
	//攻撃

	if(area_theta < 0.){//右
		if(area_theta > -0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Battle Planner: Send Command: ATK_R1");
			motion_command_.data = "ATK_R1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > -0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Battle Planner: Send Command: ATK_R2");
			motion_command_.data = "ATK_R2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > (-3.1416 + area_angle_threth_*0.5)){
			ROS_FATAL("Battle Planner: Send Command: ATK_RB2");
			motion_command_.data = "ATK_RB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: ATK_RB1");
			motion_command_.data = "ATK_RB1";	motion_command_pub_.publish(motion_command_);
		}
		//motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		return 0;
	}else{//左
		if(area_theta < 0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Battle Planner: Send Command: ATK_L1");
			motion_command_.data = "ATK_L1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < 0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Battle Planner: Send Command: ATK_L2");
			motion_command_.data = "ATK_L2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < (3.1416 - area_angle_threth_*0.5)){
			ROS_FATAL("Battle Planner: Send Command: ATK_LB2");
			motion_command_.data = "ATK_LB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: ATK_LB1");
			motion_command_.data = "ATK_LB1";	motion_command_pub_.publish(motion_command_);
		}
		//motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		return 0;
	}

	return 0;
}

int BattlePlanner::approachTarget(const std::string target_tf_name,
		const double distance_margin,
		const double tf_timeout,
		const double motion_timeout,
		bool init_flag){
	//0 時刻計測
	ros::Time now_time = ros::Time::now();
	static ros::Time start_time = now_time;
	if(init_flag == true){
		start_time = now_time;
		return INITIALIZED;
	}
	//1.1 TFが来ていない場合
	if( tfBuffer_ptr->canTransform(robot_tf_, target_tf_name,ros::Time(0)) == false){
		ROS_INFO("TF NOT FOUND");
		return TF_TIMEOUT;
	}
	//1.2 TFが古すぎる場合
	//measuring the time interval of the TF
	ros::Time latest_time;
	int err;
	std::string err_string;
	tf2::CompactFrameID compact_id;
	compact_id = tfBuffer_ptr->_lookupFrameNumber(target_tf_name);
	err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
	ros::Duration latest_dutarion = now_time - latest_time;
	if( latest_dutarion.toSec() > tf_timeout){
		ROS_INFO("TF_TIMEOUT: %f", latest_dutarion.toSec());
		return TF_TIMEOUT;
	}

	//2 移動開始
	ros::Duration time_passed = start_time - now_time;
	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(robot_tf_, target_tf_name, ros::Time(0));
	double dx = tf_diff.transform.translation.x;
	double dy = tf_diff.transform.translation.y;
	double area_theta = atan2(dy,dx);
	double area_distance = sqrt(dx*dx + dy*dy);

	if(area_distance < distance_margin){
		return SUCCEED; 	//2.1 既にゴールしている場合
	}else if(motion_timeout < time_passed.toSec()){
		return MOTION_TIMEOUT; //2.2 モーションを開始してから一定時間経過した場合
	}
	//左右旋回
	if(fabs(area_theta)>area_angle_threth_ && fabs(area_theta)< (3.1416 - area_angle_threth_)){
		if(area_distance < 0.7){
			if( (area_theta < 0. && dx > 0.0)
			||(area_theta > 0. && dx < 0.0)
			){
				ROS_INFO("Battle Planner: Send Command: TURN_RIGHT");
				motion_command_.data = "TURN_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_INFO("Battle Planner: Send Command: TURN_LEFT");
				motion_command_.data = "TURN_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}else{
			if(area_theta < 0.){
				ROS_INFO("Battle Planner: Send Command: WALK_RIGHT");
				motion_command_.data = "WALK_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_INFO("Battle Planner: Send Command: WALK_LEFT");
				motion_command_.data = "WALK_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}
		return CONTINUE;
	}
	//前進・後退
	if(area_distance > area_distance_threth_){
		if(dx < 0.){
			ROS_FATAL("Battle Planner: Send Command: WALK_BACK");
			motion_command_.data = "WALK_BACK";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: WALK_FRONT");
			motion_command_.data = "WALK_FRONT";	motion_command_pub_.publish(motion_command_);
		}
		return CONTINUE;
	}
	return ERROR;
}

int BattlePlanner::attackTarget(const std::string target_tf_name,
		const double distance_margin,
		const double tf_timeout,
		const double motion_timeout,
		bool init_flag){
	//0 時刻計測
	ros::Time now_time = ros::Time::now();
	static ros::Time start_time = now_time;
	if(init_flag == true){
		start_time = now_time;
		return INITIALIZED;
	}
	//1.1 TFが来ていない場合
	if( tfBuffer_ptr->canTransform(robot_tf_, target_tf_name,ros::Time(0)) == false){
		ROS_INFO("TF NOT FOUND");
		return TF_TIMEOUT;
	}
	//1.2 TFが古すぎる場合
	//measuring the time interval of the TF
	ros::Time latest_time;
	int err;
	std::string err_string;
	tf2::CompactFrameID compact_id;
	compact_id = tfBuffer_ptr->_lookupFrameNumber(target_tf_name);
	err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
	ros::Duration latest_dutarion = now_time - latest_time;
	if( latest_dutarion.toSec() > tf_timeout){
		ROS_INFO("TF_TIMEOUT: %f", latest_dutarion.toSec());
		return TF_TIMEOUT;
	}

	//2 攻撃開始

	ros::Duration time_passed = start_time - now_time;
	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(robot_tf_, target_tf_name, ros::Time(0));
	double dx = tf_diff.transform.translation.x;
	double dy = tf_diff.transform.translation.y;
	double area_theta = atan2(dy,dx);
	double area_distance = sqrt(dx*dx + dy*dy);

	if(area_distance < distance_margin){
		return SUCCEED; 	//2.1 既にゴールしている場合
	}else if(motion_timeout < time_passed.toSec()){
		return MOTION_TIMEOUT; //2.2 モーションを開始してから一定時間経過した場合
	}
	//攻撃モーション選択
	if(area_theta < 0.){//右
		if(area_theta > -0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Battle Planner: Send Command: ATK_R1");
			motion_command_.data = "ATK_R1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > -0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Battle Planner: Send Command: ATK_R2");
			motion_command_.data = "ATK_R2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > (-3.1416 + area_angle_threth_*0.5)){
			ROS_FATAL("Battle Planner: Send Command: ATK_RB2");
			motion_command_.data = "ATK_RB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: ATK_RB1");
			motion_command_.data = "ATK_RB1";	motion_command_pub_.publish(motion_command_);
		}
		return CONTINUE;
	}else{//左
		if(area_theta < 0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Battle Planner: Send Command: ATK_L1");
			motion_command_.data = "ATK_L1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < 0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Battle Planner: Send Command: ATK_L2");
			motion_command_.data = "ATK_L2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < (3.1416 - area_angle_threth_*0.5)){
			ROS_FATAL("Battle Planner: Send Command: ATK_LB2");
			motion_command_.data = "ATK_LB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: ATK_LB1");
			motion_command_.data = "ATK_LB1";	motion_command_pub_.publish(motion_command_);
		}
		return CONTINUE;
	}
	return ERROR;
}

int BattlePlanner::avoidTarget(const std::string target_tf_name,
		const double distance_margin,
		const double tf_timeout,
		const double motion_timeout,
		bool init_flag){
	//0 時刻計測
	ros::Time now_time = ros::Time::now();
	static ros::Time start_time = now_time;
	if(init_flag == true){
		start_time = now_time;
		return INITIALIZED;
	}
	//1.1 TFが来ていない場合
	if( tfBuffer_ptr->canTransform(robot_tf_, target_tf_name,ros::Time(0)) == false){
		ROS_INFO("TF NOT FOUND");
		return TF_TIMEOUT;
	}
	//1.2 TFが古すぎる場合
	//measuring the time interval of the TF
	ros::Time latest_time;
	int err;
	std::string err_string;
	tf2::CompactFrameID compact_id;
	compact_id = tfBuffer_ptr->_lookupFrameNumber(target_tf_name);
	err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
	ros::Duration latest_dutarion = now_time - latest_time;
	if( latest_dutarion.toSec() > tf_timeout){
		ROS_INFO("TF_TIMEOUT: %f", latest_dutarion.toSec());
		return TF_TIMEOUT;
	}

	//2 移動開始
	ros::Duration time_passed = start_time - now_time;
	geometry_msgs::TransformStamped tf_diff = tfBuffer_ptr->lookupTransform(robot_tf_, target_tf_name, ros::Time(0));
	double dx = -tf_diff.transform.translation.x;
	double dy = -tf_diff.transform.translation.y;
	double area_theta = atan2(dy,dx);
	double area_distance = sqrt(dx*dx + dy*dy);

	if(area_distance < distance_margin){
		return SUCCEED; 	//2.1 既にゴールしている場合
	}else if(motion_timeout < time_passed.toSec()){
		return MOTION_TIMEOUT; //2.2 モーションを開始してから一定時間経過した場合
	}
	//左右旋回
	if(fabs(area_theta)>area_angle_threth_ && fabs(area_theta)< (3.1416 - area_angle_threth_)){
		if(area_distance < 0.7){
			if( (area_theta < 0. && dx > 0.0)
			||(area_theta > 0. && dx < 0.0)
			){
				ROS_INFO("Battle Planner: Send Command: TURN_RIGHT");
				motion_command_.data = "TURN_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_INFO("Battle Planner: Send Command: TURN_LEFT");
				motion_command_.data = "TURN_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}else{
			if(area_theta < 0.){
				ROS_INFO("Battle Planner: Send Command: WALK_RIGHT");
				motion_command_.data = "WALK_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_INFO("Battle Planner: Send Command: WALK_LEFT");
				motion_command_.data = "WALK_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}
		return CONTINUE;
	}
	//前進・後退
	if(area_distance > area_distance_threth_){
		if(dx < 0.){
			ROS_FATAL("Battle Planner: Send Command: WALK_BACK");
			motion_command_.data = "WALK_BACK";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Battle Planner: Send Command: WALK_FRONT");
			motion_command_.data = "WALK_FRONT";	motion_command_pub_.publish(motion_command_);
		}
		return CONTINUE;
	}
	return ERROR;
}

int BattlePlanner::searchTarget(const std::string target_tf_name,
		const double tf_timeout,
		bool init_flag){
	static int search_motion_counter = 0;
	bool search_flag = false;
	//0 時刻計測
	ros::Time now_time = ros::Time::now();
	if(init_flag == true){
		search_flag = true;
	}
	//1.1 TFが来ていない場合
	if( tfBuffer_ptr->canTransform(robot_tf_, target_tf_name,ros::Time(0)) == false){
		search_flag = true;
	}
	//1.2 TFが古すぎる場合
	//measuring the time interval of the TF
	ros::Time latest_time;
	int err;
	std::string err_string;
	tf2::CompactFrameID compact_id;
	compact_id = tfBuffer_ptr->_lookupFrameNumber(target_tf_name);
	err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
	ros::Duration latest_dutarion = now_time - latest_time;
	if( latest_dutarion.toSec() > tf_timeout){
		search_flag = true;
	}
	if(true == search_flag){
		if(search_motion_counter % 2 ==0){
			motion_command_.data = "MOVE_URG2";
			motion_command_pub_.publish(motion_command_);
			sleep(2);
		}else{
			motion_command_.data = "MOVE_URG3";
			motion_command_pub_.publish(motion_command_);
			sleep(3);
		}
		search_motion_counter ++;
		return CONTINUE;
	}else{
		return SUCCEED;
	}
	return ERROR;
}


