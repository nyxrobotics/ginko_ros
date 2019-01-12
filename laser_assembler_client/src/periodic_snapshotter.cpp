
//参考:https://qiita.com/nnn_anoken/items/029b326317d9fa0f3499
#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans.h"
// Messages
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace laser_assembler {

class PeriodicSnapshotter {
public:
	PeriodicSnapshotter() {
		// Create a publisher for the clouds that we assemble
//		pub_ = n_.advertise<sensor_msgs::PointCloud>("/sensor/urg/assembled_cloud_right", 1);
	    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("cloud", 1);

		// Create the service client for calling the assembler
		client_ = n_.serviceClient<AssembleScans>("assemble_scans");

		// Start the timer that will trigger the processing loop (timerCallback)
		timer_ = n_.createTimer(ros::Duration(0.2, 0),&PeriodicSnapshotter::timerCallback, this);

		// Need to track if we've called the timerCallback at least once
		first_time_ = true;
	}

	void timerCallback(const ros::TimerEvent& e) {

		// We don't want to build a cloud the first callback, since we we
		//   don't have a start and end time yet
		if (first_time_) {
			first_time_ = false;
			return;
		}

		// Populate our service request based on our timer callback times
		AssembleScans srv;
//    srv.request.begin = e.last_real;
		srv.request.end = e.current_real;
		srv.request.begin = srv.request.end - ros::Duration(5.0);

		// Make the service call
		if (client_.call(srv)) {
			ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size()));
//			pub_.publish(srv.response.cloud);
		  sensor_msgs::PointCloud2 cloud2;
		  sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud2);
		  pub_.publish(cloud2);
		} else {
			ROS_ERROR("Error making service call\n");
		}
	}

private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::ServiceClient client_;
	ros::Timer timer_;
	bool first_time_;
};

}

using namespace laser_assembler;

int main(int argc, char **argv) {
	ros::init(argc, argv, "periodic_snapshotter");
	ros::NodeHandle n;
	ROS_INFO("Waiting for [build_cloud] to be advertised");
	ros::service::waitForService("build_cloud");
	ROS_INFO("Found build_cloud! Starting the snapshotter");
	PeriodicSnapshotter snapshotter;
	ros::spin();
	return 0;
}
