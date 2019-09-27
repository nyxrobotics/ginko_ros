#include <regulation_planner.h>

//RegulationPlanner here
RegulationPlanner::RegulationPlanner(ros::NodeHandle main_nh){
	initTF2();
	readParams(main_nh);
	initSubscriber(main_nh);
	initPublisher(main_nh);
	regulation_state_ = "off";
}

RegulationPlanner::~RegulationPlanner() {
	ros::shutdown();
}

void RegulationPlanner::readParams(ros::NodeHandle node_handle_){
}

void RegulationPlanner::initSubscriber(ros::NodeHandle node_handle_){
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	regulation_command_sub_ = node_handle_.subscribe("regulation_command", 10,&RegulationPlanner::getRegulationCommandCallback, this,transport_hints);
	imu_quaternion_sub_ = node_handle_.subscribe("imu_quaternion_in", 10,&RegulationPlanner::getImuQuaternionCallback, this, transport_hints);
}

void RegulationPlanner::initTF2(){
	//クラス内での宣言時では引数をもつコンストラクタを呼べないので、boost::shared_ptrを使って宣言し、ここで初期化をする。
	//参考：https://answers.ros.org/question/315697/tf2-buffer-length-setting-problem/
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(10.0), true));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	//sleep(10);//TFが安定するまで待つ
}

void RegulationPlanner::initPublisher(ros::NodeHandle node_handle_){
	motion_command_pub_ = node_handle_.advertise<std_msgs::String>("motion_command_out", 10);
}

int RegulationPlanner::mainLoop(){
	if(imu_ready_ == 0){
		usleep(100000);
		return 0;
	}
//	ROS_FATAL("Regulation Planner: STANDBY ");
	if(regulation_state_ == "off"){
		if(regulation_command_.data == "on"){
			ROS_FATAL("Regulation Planner: Send Command: TORQUE_ON");
			motion_command_.data = "TORQUE_ON";	motion_command_pub_.publish(motion_command_);
			usleep(100000);
			ROS_FATAL("Regulation Planner: Send Command: STANDING");
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
			regulation_state_ = "standby";
			regulation_command_.data = "";
			sleep(5);
			motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
			// sleep(5);
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
		}else{
//			ROS_FATAL("Regulation Planner: Send Command: TORQUE_OFF");
			motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
			regulation_state_ = "off";
			regulation_command_.data = "";
			usleep(100000);
			return 0;
		}
	}
	ros::spinOnce();
	if(regulation_command_.data == "off"){
		ROS_FATAL("Regulation Planner: Send Command: TORQUE_OFF");
		motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
		regulation_state_ = "off";
		regulation_command_.data = "";
		usleep(100000);
		return 0;
	}


	ros::spinOnce();
	static int imu_fall_direction_prev = imu_fall_direction_;
	if(imu_fall_direction_prev != 0 && imu_fall_direction_ == 0){
		sleep(3);
		ros::spinOnce();
		if(imu_fall_direction_ == 0){
			motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
			imu_fall_direction_prev = imu_fall_direction_;
			sleep(1);
		}
	}
	imu_fall_direction_prev = imu_fall_direction_;

	if(imu_fall_direction_ == 0){
		if(regulation_command_.data == "regulation"){
			regulation_state_ = "regulation";
			regulation_command_.data = "";
		}
		regulationMotionSelect();
		ROS_FATAL("Regulation Planner: Status: BATTLE!!!");
		usleep(100000);
		return 0;
	}

	sleep(1);
	ros::spinOnce();
	if(regulation_command_.data == "off"){
		ROS_FATAL("Regulation Planner: Send Command: TORQUE_OFF");
		motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
		regulation_state_ = "off";
		regulation_command_.data = "";
		usleep(100000);
		return 0;
	}

	if(imu_fall_direction_ == 0){
		usleep(100000);
		return 0;
	}
	if(imu_fall_direction_ == 1){
		ROS_FATAL("Regulation Planner: Send Command: WAKEUP_FRONT");
		motion_command_.data = "WAKEUP_FRONT";	motion_command_pub_.publish(motion_command_);
		sleep(3);
		 motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
//		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		// sleep(5);
		return 0;
	}
	if(imu_fall_direction_ == 2){
		ROS_FATAL("Regulation Planner: Send Command: WAKEUP_BACK");
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



void RegulationPlanner::getRegulationCommandCallback(const std_msgs::String::ConstPtr& msg){
	regulation_command_ready_ = 1;
	regulation_command_update_ = 1;
	regulation_command_ = *msg;
}

void RegulationPlanner::getImuQuaternionCallback(const sensor_msgs::Imu::ConstPtr& msg){
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
//	ROS_FATAL("Regulation Planner: fall/stand: %d",imu_fall_direction_);
}

int RegulationPlanner::regulationMotionSelect(){
	//TFが来ていない場合は探索
	if( tfBuffer_ptr->canTransform(robot_tf_, target_tf_,ros::Time(0)) == false){
		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		return 0;
	}
	//measuring the time interval of the TF
	ros::Time latest_time;
	int err;
	std::string err_string;
	tf2::CompactFrameID compact_id;
	compact_id = tfBuffer_ptr->_lookupFrameNumber(target_tf_);
	err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
	ros::Duration latest_dutarion = ros::Time::now() - latest_time;
	//TFが古すぎる場合は探索
	ROS_FATAL("target passed time: %f", latest_dutarion.toSec());
	if( latest_dutarion.toSec() > 5.0){
		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
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
				ROS_FATAL("Regulation Planner: Send Command: TURN_RIGHT");
				motion_command_.data = "TURN_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_FATAL("Regulation Planner: Send Command: TURN_LEFT");
				motion_command_.data = "TURN_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}else{
			if(area_theta < 0.){
				ROS_FATAL("Regulation Planner: Send Command: WALK_RIGHT");
				motion_command_.data = "WALK_RIGHT";	motion_command_pub_.publish(motion_command_);
			}else{
				ROS_FATAL("Regulation Planner: Send Command: WALK_LEFT");
				motion_command_.data = "WALK_LEFT";	motion_command_pub_.publish(motion_command_);
			}
		}
		return 0;
	}
	//前進・後退
	if(area_distance > area_distance_threth_){
		if(dx < 0.){
			ROS_FATAL("Regulation Planner: Send Command: WALK_BACK");
			motion_command_.data = "WALK_BACK";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Regulation Planner: Send Command: WALK_FRONT");
			motion_command_.data = "WALK_FRONT";	motion_command_pub_.publish(motion_command_);
		}
		return 0;
	}
	//攻撃

	if(area_theta < 0.){//右
		if(area_theta > -0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Regulation Planner: Send Command: ATK_R1");
			motion_command_.data = "ATK_R1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > -0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Regulation Planner: Send Command: ATK_R2");
			motion_command_.data = "ATK_R2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta > (-3.1416 + area_angle_threth_*0.5)){
			ROS_FATAL("Regulation Planner: Send Command: ATK_RB2");
			motion_command_.data = "ATK_RB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Regulation Planner: Send Command: ATK_RB1");
			motion_command_.data = "ATK_RB1";	motion_command_pub_.publish(motion_command_);
		}
		//motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		return 0;
	}else{//左
		if(area_theta < 0.25 * (3.1416 - area_angle_threth_) ){
			ROS_FATAL("Regulation Planner: Send Command: ATK_L1");
			motion_command_.data = "ATK_L1";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < 0.5 * (3.1416 - area_angle_threth_)){
			ROS_FATAL("Regulation Planner: Send Command: ATK_L2");
			motion_command_.data = "ATK_L2";	motion_command_pub_.publish(motion_command_);
		}else if(area_theta < (3.1416 - area_angle_threth_*0.5)){
			ROS_FATAL("Regulation Planner: Send Command: ATK_LB2");
			motion_command_.data = "ATK_LB2";	motion_command_pub_.publish(motion_command_);
		}else{
			ROS_FATAL("Regulation Planner: Send Command: ATK_LB1");
			motion_command_.data = "ATK_LB1";	motion_command_pub_.publish(motion_command_);
		}
		//motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		return 0;
	}

	return 0;
}





