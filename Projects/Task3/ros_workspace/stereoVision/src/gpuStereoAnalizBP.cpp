#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
 
using namespace sensor_msgs;
using namespace message_filters;

static int StereoNDisp = 50;
static int StereoIters = 1;
static int StereoLevels = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{
	cv::Mat left_img_rgb;
	cv::Mat right_img_rgb;
	cv::Mat left;
	cv::Mat right;
	cv::gpu::GpuMat d_left, d_right;
	cv::gpu::GpuMat disp_dst;
	cv::gpu::StereoBeliefPropagation bp;
	
	bp.ndisp = StereoNDisp;
	bp.iters = StereoIters;
	bp.levels = StereoLevels;

	// Подготовка изображений
	cv_bridge::CvImagePtr left_img_ptr = cv_bridge::toCvCopy(msg1,sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr right_img_ptr = cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::BGR8);
	left_img_rgb = left_img_ptr->image;
	right_img_rgb = right_img_ptr->image;
	cv::Mat disp(left_img_rgb.size(), CV_8U);
	cv::gpu::GpuMat d_disp(left_img_rgb.size(), CV_8U);
	cv::cvtColor(left_img_rgb, left, CV_BGR2GRAY);
	cv::cvtColor(right_img_rgb, right, CV_BGR2GRAY);
	d_left.upload(left);
	d_right.upload(right);

	// Работа с BP
	bp(d_left, d_right, d_disp);

	// Визуализация
	d_disp.download(disp);
	cv::imshow("Stereo Control", disp);
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;

	cv::namedWindow("Stereo Control");
	cv::startWindowThread();

	// Интерфейс настройки параметров
	cv::createTrackbar("NDisp ", "Stereo Control", &StereoNDisp, 1024);
	cv::createTrackbar("Iters", "Stereo Control", &StereoIters, 10);
	cv::createTrackbar("Levels", "Stereo Control", &StereoLevels, 10);

	message_filters::Subscriber<Image> image1_sub(nh, "stereo/left/image_raw", 1);
	message_filters::Subscriber<Image> image2_sub(nh, "stereo/right/image_raw", 1);

	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	ros::spin();
	cv::destroyWindow("Stereo Control");
}

