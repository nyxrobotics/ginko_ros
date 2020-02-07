
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "urg_nearest_detector");
	ros::NodeHandle node_handle_("~");
	UrgNearest urg_nearest(node_handle_);

//	ros::spin();
	ros::Rate rate_(20); // 20 hz
	while (ros::ok()) {
		urg_nearest.mainLoop();
		ros::spinOnce();
		rate_.sleep();
	}

	return 0;
}
