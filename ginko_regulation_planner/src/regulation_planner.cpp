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
	//"standby"コマンドを受け取った際の処理
	if(regulation_state_ == "off"){
		if(regulation_command_.data == "standby"){
			ROS_INFO("Regulation Planner: Send Command: TORQUE_ON");
			motion_command_.data = "TORQUE_ON";	motion_command_pub_.publish(motion_command_);
			usleep(100000);
			ROS_INFO("Regulation Planner: Send Command: STANDING");
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
			regulation_state_ = "standby";
			regulation_command_.data = "";
			sleep(5);
			motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
			sleep(0.5);
			motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
		}else{
			motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
			regulation_state_ = "off";
			regulation_command_.data = "";
			usleep(100000);
			return 0;
		}
	}

	//"off"コマンドを受け取った際の処理
	ros::spinOnce();
	if(regulation_command_.data == "off"){
		ROS_INFO("Regulation Planner: Send Command: TORQUE_OFF");
		motion_command_.data = "TORQUE_OFF";	motion_command_pub_.publish(motion_command_);
		regulation_state_ = "off";
		regulation_command_.data = "";
		usleep(100000);
		return 0;
	}

	//"start"コマンドを受け取った際の処理
	if(regulation_command_.data == "start" && regulation_state_ == "standby"){
		ROS_INFO("Regulation Planner: Status: START regulation test");
		motion_command_.data = "MOVE_URG2";	motion_command_pub_.publish(motion_command_);
		sleep(4);
//		ros::spinOnce();
		regulationMotionSelect();
		regulation_state_ = "standby";
		regulation_command_.data = "";
		usleep(100000);
		return 0;
	}

	return 1;
}



void RegulationPlanner::getRegulationCommandCallback(const std_msgs::String::ConstPtr& msg){
	regulation_command_ready_ = 1;
	regulation_command_update_ = 1;
	regulation_command_ = *msg;
}

int RegulationPlanner::regulationMotionSelect(){
	//それぞれのTFが有効か確認
	bool human_r_ready = checkTfAlive(human_r_tf_);
	bool human_l_ready = checkTfAlive(human_l_tf_);
	bool robot_standing_r_ready = checkTfAlive(robot_standing_r_tf_);
	bool robot_standing_l_ready = checkTfAlive(robot_standing_l_tf_);
	bool robot_laying_r_ready = checkTfAlive(robot_laying_r_tf_);
	bool robot_laying_l_ready = checkTfAlive(robot_laying_l_tf_);

	if(human_r_ready || human_l_ready){
		ROS_FATAL("Regulation Planner: DETECT_HUMAN");
		motion_command_.data = "DETECT_HUMAN";	motion_command_pub_.publish(motion_command_);
	}else if(robot_standing_r_ready || robot_standing_l_ready){
		ROS_FATAL("Regulation Planner: DETECT_ROBOT_STANDING");
		motion_command_.data = "DETECT_ROBOT_STANDING";	motion_command_pub_.publish(motion_command_);
	}else if(robot_laying_r_ready || robot_laying_l_ready){
		ROS_FATAL("Regulation Planner: DETECT_ROBOT_LAYING");
		motion_command_.data = "DETECT_ROBOT_LAYING";	motion_command_pub_.publish(motion_command_);
	}else{
		ROS_FATAL("Regulation Planner: NOT FOUND");
		motion_command_.data = "STANDING";	motion_command_pub_.publish(motion_command_);
	}
	return 0;
}


bool RegulationPlanner::checkTfAlive(std::string tf_name){
	//TFが有効か確認
	if( tfBuffer_ptr->canTransform(robot_tf_, tf_name,ros::Time(0)) == false){
		return false;
	}else{
		//measuring the time interval of the TF
		ros::Time latest_time;
		int err;
		std::string err_string;
		tf2::CompactFrameID compact_id;
		compact_id = tfBuffer_ptr->_lookupFrameNumber(tf_name);
		err = tfBuffer_ptr->_getLatestCommonTime(compact_id, compact_id, latest_time, &err_string);
		ros::Duration latest_dutarion = ros::Time::now() - latest_time;
		if( latest_dutarion.toSec() > 5.0){
			return false;
		}else{
			return true;
		}
	}
	return false;
}





