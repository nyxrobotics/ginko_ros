#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "roboone_ring_tf_publisher");
	ros::NodeHandle node_handle_("~");
	//ノードハンドラを渡さないと、rosparamで受け取ろうとする名前空間がノード名の一層上になってしまい、launchの中でかけない。
	//参考:http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle

	RingTfPublisher ring_tf_publisher(node_handle_);
	ros::Rate rate_(10); // 10 hz
	while (ros::ok()) {
		ring_tf_publisher.mainLoop();
		ros::spinOnce();
		rate_.sleep();
	}
	ros::shutdown();
	return 0;
}



