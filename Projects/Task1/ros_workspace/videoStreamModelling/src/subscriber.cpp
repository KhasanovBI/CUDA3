#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <image_transport/image_transport.h>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <map>
#include <fstream>

using namespace cv;
using namespace cv::gpu;
using namespace std;

CascadeClassifier_GPU cascade_gpu;
std::map<string, int> myMap;

template<class T>
void convertAndResize(const T& src, T& gray, T& resized, double scale)
{
    if (src.channels() == 3)
    {
        cvtColor( src, gray, CV_BGR2GRAY );
    }
    else
    {
        gray = src;
    }

    Size sz(cvRound(gray.cols * scale), cvRound(gray.rows * scale));

    if (scale != 1)
    {
        resize(gray, resized, sz);
    }
    else
    {
        resized = gray;
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Создание вспомогательных объектов Mat
        Mat ImageBuf, resized_cpu;

        // Создание объекта GpuMat и копирование изображения в память GPU
        GpuMat gpuImage(image);

        // Создание вспомогательных объектов Mat
        GpuMat gpuImageBuf, gray_gpu, resized_gpu;

        // Параметры каскада
        double scaleFactor = 1.0;
        bool findLargestObject = true;

        // Подготовка изображения
        convertAndResize(gpuImage, gray_gpu, resized_gpu, scaleFactor);

        // Установка параметра
        cascade_gpu.findLargestObject = findLargestObject;

        // Параметры искомого фрагмента
        Size minSize;
        minSize.height = 60;
        minSize.width = 60;

        // Поиск объекта
        int detections_num = cascade_gpu.detectMultiScale(resized_gpu, gpuImageBuf, 1.2, 1, minSize);

        // Выгрузка объектов из GPU
        gpuImageBuf.colRange(0,detections_num).download(ImageBuf);
        resized_gpu.download(resized_cpu);

        cout << "Num: " << detections_num << endl;
        if (detections_num > 0) {
            myMap["detected"] += detections_num;
        }
        else {
            myMap["undetected"] += 1;
        }

        // Выделение найденного объекта в прямоугольник
        for (int i = 0; i < detections_num; ++i) {
            rectangle(resized_cpu, ImageBuf.ptr<cv::Rect>()[i], Scalar(15));
        }

        // Отображение найденного фрагмента
        imshow("view", resized_cpu);

    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();

    cascade_gpu.load("../../OpenCV/data/cascade.xml");
    myMap["detected"] = 0;
    myMap["undetected"] = 0;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    std::ofstream file ("output.txt", std::ofstream::out);
    for (std::map<string, int>::iterator it=myMap.begin(); it!=myMap.end(); ++it)
        file << it->first << ": " << it->second << '\n';
    file.close();
    cv::destroyWindow("view");
}
