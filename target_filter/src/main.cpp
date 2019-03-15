
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "target_filter_detector");
	ros::NodeHandle node_handle_("~");
	TargetFilter target_filter(node_handle_);
	ros::Rate rate_(100); // 100 hz
//	ros::spin();
	while (ros::ok()) {
		target_filter.mainLoop();
		ros::spinOnce();
		rate_.sleep();
	}

	return 0;
}
