#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "imu_drift_correction_node");

	while (ros::ok()) {
		ROS_INFO("message:test loop");
		sleep(1);
	}

	ros::shutdown();
	return 0;
}
