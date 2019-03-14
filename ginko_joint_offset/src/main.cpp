
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_joint_offset");
	GinkoOffsets ginko_controller; //宣言のタイミングでginko_controllerのコンストラクタが呼ばれる。

	ros::spin();

	return 0;
}
