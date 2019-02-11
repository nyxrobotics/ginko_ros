#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "imu_angle_correction_node");
	ImuRpy imu_rpy;

	ros::spin();
//	while (ros::ok()) {
//		ROS_FATAL("message:test loop");
//		sleep(1);
//	}

	ros::shutdown();
	return 0;
}
