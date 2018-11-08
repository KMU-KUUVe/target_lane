#include "ros/ros.h"
#include <iostream>
#include "target_lane/TargetLaneNode.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "target_lane");

#if 1	// using camera
	TargetLaneNode target_lane_node;

	ros::spin();

#else	// using mp4 file
	TargetLaneNode target_lane_node("../challenge.mp4");

	target_lane_node.run_test();

#endif

	return 0;
}
