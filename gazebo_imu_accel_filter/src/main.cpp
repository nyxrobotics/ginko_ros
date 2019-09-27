#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "imu_angle_correction_node");
	ros::NodeHandle node_handle_("~");

	//ノードハンドラを渡さないと、rosparamで受け取ろうとする名前空間がノード名の一層上になってしまい、launchの中でかけない。
	//参考:http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle
	AccelOverwrite imu_drift_correction(node_handle_);
//		AccelOverwrite imu_drift_correction;

	ros::spin();

	ros::shutdown();
	return 0;
}
