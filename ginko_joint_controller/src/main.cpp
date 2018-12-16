
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_controller");
	GinkoController ginko_controller; //宣言のタイミングでginko_controllerの初期化が呼ばれる。
	ros::Rate loop_rate(LOOP_FREQUENCY);

	while (ros::ok()) {
		ginko_controller.control_loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
