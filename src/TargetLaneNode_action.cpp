#include "target_lane/TargetLaneNode.h"

using namespace std;
using namespace cv;

TargetLaneNode::TargetLaneNode()
	:as_(nh_, "target_lane", boost::bind(&TargetLaneNode::actionCallback, this, _1), false)
{
	nh_ = ros::NodeHandle("~");
	as_.start();

	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
	//control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &TargetLaneNode::imageCallback, this);

	getRosParamForUpdate();
}


void TargetLaneNode::actionCallback(const car_tracking::car_trackingGoalConstPtr& goal)
{
	cout << "lane detector actioniCallback called" << endl;
	mission_start = true;

	ros::Rate r(10);

	while(ros::ok()){
		if(mission_cleared){
			car_tracking::car_trackingResult result;
			as_.setSucceeded(result);
			mission_start = false;
			break;
		}
		r.sleep();
	}
}

void TargetLaneNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	if(!mission_start){
		try{
			parseRawimg(image, frame);
		} catch(const cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return ;
		} catch(const std::runtime_error& e) {
			cerr << e.what() << endl;
		}

		getRosParamForUpdate();
		steer_control_value = laneDetecting();
		car_tracking::car_trackingFeedback feedback;
		feedback.tracking_feedback = steer_control_value; //feedback name is steer and type should be double.
		as_.publishFeedback(feedback);
	}
}


void TargetLaneNode::getRosParamForUpdate()
{
	//nh_.getParam("throttle", throttle_);
	//nh_.getParam("angle_factor", angle_factor_);
}

int TargetLaneNode::laneDetecting()
{
	int ncols = frame.cols;
	int nrows = frame.rows;
	int left_x =0;
	int right_x =frame.cols;
	unsigned int zero_count = 0;

	int64 t1 = getTickCount();
	frame_count++;

	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));
	img_mask = targetlane.mask(lane_frame);
	//imshow("img_mask", img_mask);
	targetlane.filter_colors(img_mask, img_mask2);
	//imshow("img_mask2", img_mask2);
	img_denoise = targetlane.deNoise(img_mask2);
	//imshow("img_denoise", img_denoise);

	double angle = targetlane.steer_control(img_denoise, steer_height, 12, left_x, right_x, img_mask, zero_count);
	if(zero_count > 1500){
		mission_cleared= true;
	}
	else{
		mission_cleared = false;
	}

	int64 t2 = getTickCount();
	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum += ms;
	avg = sum / (double)frame_count;
	waitKey(3);

	return angle * angle_factor_;
}



void TargetLaneNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}
