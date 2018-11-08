#include "target_lane/TargetLaneNode.h"

using namespace std;
using namespace cv;

TargetLaneNode::TargetLaneNode()
	:as_(nh_, "target_lane", boost::bind(&TargetLaneNode::actionCallback, this, _1), false)
{
	nh_ = ros::NodeHandle("~");
	as_.start();

	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &TargetLaneNode::imageCallback, this);

	getRosParamForUpdate();
}


void TargetLaneNode::actionCallback(const state_cpp_msg::MissionPlannerGoalConstPtr& goal)
{
	cout << "lane detector actioniCallback called" << endl;
	mission_start = true;

	ros::Rate r(10);

	while(ros::ok()){
		if(mission_cleared){
			state_cpp_msg::MissionPlannerResult result;
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
		steer_control_value_ = laneDetecting();
		//state_cpp_msg::MissionPlannerFeedback feedback;
		//feedback.steer = steer_control_value_; //feedback name is steer and type should be double.
		//as_.publishFeedback(feedback);
		ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
		control_pub_.publish(control_msg);
	}
}


void TargetLaneNode::getRosParamForUpdate()
{
	nh_.getParam("throttle", throttle_);
	nh_.getParam("angle_factor", angle_factor_);
}


ackermann_msgs::AckermannDriveStamped TargetLaneNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
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
	//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;
	waitKey(3);
	//ROS_INFO("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);

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


