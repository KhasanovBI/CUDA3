#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        clock_t start = clock();
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = img_ptr->image;

        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, CV_BGR2GRAY);
        cv::equalizeHist(img_gray,img_gray);
        
        // Загрузка изображение на GPU:
        cv::gpu::GpuMat img_gray_gpu;
        img_gray_gpu.upload(img_gray);

        // Выбор каскада
        std::string cascade_name = "../imageVideo/haarcascade_mcs_righteye.xml";
        //std::string cascade_name = "../imageVideo/haarcascade_mcs_nose.xml";
        cv::gpu::CascadeClassifier_GPU cascade_gpu;
        cascade_gpu.load(cascade_name);
        
        // Работа с классификатором
        cv::gpu::GpuMat objbuf;
        int detect = cascade_gpu.detectMultiScale(img_gray_gpu, objbuf, 1.1, 1, cv::Size(50, 50));

        // Визуализация работы классификатора
        cv::Mat obj_host;        
        objbuf.colRange(0, detect).download(obj_host);
        cv::Rect *fc = obj_host.ptr<cv::Rect>();
        for(int i = 0; i < detect; i++) {
            cv::rectangle(img_gray, fc[i], cv::Scalar(255));
        }
        clock_t stop = clock();
        std::cout <<  1000. * (stop - start) / CLOCKS_PER_SEC << std::endl;
        cv::imshow("view", img_gray);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gpu_image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}
