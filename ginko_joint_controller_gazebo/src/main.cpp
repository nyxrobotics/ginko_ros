
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_controller");
	GinkoController ginko_controller_gazebo;
	 ros::spin();
//	ros::Rate loop_rate0(LOOP_FREQUENCY);
//	while (ros::ok()){
//		loop_rate0.sleep();
//		ros::spinOnce();
//	}
	return 0;
}
