#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
 
using namespace sensor_msgs;
using namespace message_filters;

void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{
	cv::imshow("left", cv_bridge::toCvShare(msg1, "bgr8")->image);
	cv::imshow("right", cv_bridge::toCvShare(msg2, "bgr8")->image);
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;

	cv::namedWindow("left");
	cv::startWindowThread();

	cv::namedWindow("right");
	cv::startWindowThread();

	message_filters::Subscriber<Image> image1_sub(nh, "camera/left", 1);
	message_filters::Subscriber<Image> image2_sub(nh, "camera/right", 1);

	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	ros::spin();
	cv::destroyWindow("left");
	cv::destroyWindow("right");
}

