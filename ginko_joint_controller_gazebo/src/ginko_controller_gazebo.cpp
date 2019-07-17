#include <ginko_controller_gazebo.h>

//GinkoControler here
GinkoController::GinkoController(){
	initPublisher();
	initSubscriber();
	ROS_INFO("Ginko_controller : Init OK!");
}

GinkoController::~GinkoController() {
	ros::shutdown();
}

void GinkoController::initPublisher() {
	//	jointstates(リターン値)は、gazebo立ち上げ時にjoint state publisherがやってくれる
	//	joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
	for (int index = 0; index < GAZEBO_JOINT_NUM; index++){
		std::string topic_name = ginko_params_._gazebo_joint_name[index] + ginko_params_._gazebo_target_topic_footer;
		gazebo_joints_pub_[index] = node_handle_.advertise<std_msgs::Float64>(topic_name, 10);
	}
	//	body_joint1_pub_ = node_handle_.advertise<std_msgs::Float64>("body_joint1_position_controller/command", 10);
	//	arm_r_joint0_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_r_joint0_position_controller/command", 10);
	//	arm_r_joint1_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_r_joint1_position_controller/command", 10);
	//	arm_r_joint2_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_r_joint2_position_controller/command", 10);
	//	arm_r_joint3_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_r_joint3_position_controller/command", 10);
	//	arm_l_joint0_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_l_joint0_position_controller/command", 10);
	//	arm_l_joint1_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_l_joint1_position_controller/command", 10);
	//	arm_l_joint2_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_l_joint2_position_controller/command", 10);
	//	arm_l_joint3_pub_ = node_handle_.advertise<std_msgs::Float64>("arm_l_joint3_position_controller/command", 10);
	//	leg_r_joint0_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_r_joint0_position_controller/command", 10);
	//	leg_r_joint1_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_r_joint1_position_controller/command", 10);
	//	leg_r_joint4_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_r_joint4_position_controller/command", 10);
	//	leg_r_joint7_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_r_joint7_position_controller/command", 10);
	//	leg_r_joint8_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_r_joint8_position_controller/command", 10);
	//	leg_l_joint0_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_l_joint0_position_controller/command", 10);
	//	leg_l_joint1_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_l_joint1_position_controller/command", 10);
	//	leg_l_joint4_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_l_joint4_position_controller/command", 10);
	//	leg_l_joint7_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_l_joint7_position_controller/command", 10);
	//	leg_l_joint8_pub_ = node_handle_.advertise<std_msgs::Float64>("leg_l_joint8_position_controller/command", 10);
}

void GinkoController::initSubscriber() {
	goal_joint_states_sub_ = node_handle_.subscribe("goal_joint_position", 1,&GinkoController::goalJointPositionCallback, this);
	torque_enable_sub_ = node_handle_.subscribe("torque_enable",1, &GinkoController::torqueEnableCallback, this);
}

void GinkoController::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	// ROS_INFO("gazebo_joint_controller:TargetJointReceived");
	for (int index = 0; index < msg->position.size(); index++){
		std::string joint_name;
		for (int index2 = 0; index2 < SERVO_NUM; index2++){
			if(index == (ginko_params_._servo_id[index2] -1)){
				joint_name = ginko_params_._servo_joint_name[index2];
				index2=SERVO_NUM;
			}
		}
		for (int index2 = 0; index2 < GAZEBO_JOINT_NUM; index2++){
			if(ginko_params_._gazebo_joint_name[index2] == joint_name){
				// ROS_INFO("gazebo_joint_controller:Name matched");
				std_msgs::Float64 target_angle;
				target_angle.data = msg->position.at(index);
				gazebo_joints_pub_[index2].publish(target_angle);
				index2 = GAZEBO_JOINT_NUM;
			}
		}
	}

	//	for (int index = 0; index < GAZEBO_JOINT_NUM; index++){
	//		for (int index2 = 0; index2 < msg->name.size(); index2++){
	//			if(ginko_params_._gazebo_joint_name[index] == msg->name.at(index2) ){
	//				 ROS_INFO("gazebo_joint_controller:Name matched");
	//				std_msgs::Float64 target_angle;
	//				target_angle.data = msg->position.at(index2);
	//				gazebo_joints_pub_[index].publish(target_angle);
	//				index2 = SERVO_NUM;
	//			}
	//		}
	//	}

}

void GinkoController::torqueEnableCallback(const std_msgs::Int8 &msg) { //0:off, 1:on, 2:break
	//未実装。実装予定
	// ROS_INFO("torque on* %d",msg.data);
}
