#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

int main(int argc, char** argv) {
    if(argv[1] == NULL) {
        return 1;
    }
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    std::cout << argv[1] << std::endl;
    std::ifstream imagesPathsFile;
    imagesPathsFile.open(argv[1]);
    std::string imagePath;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(10);
    while (nh.ok() && !imagesPathsFile.eof()) {
        imagesPathsFile >> imagePath;
        std::cout << imagePath << std::endl;
        cv::Mat frame = cv::imread(imagePath);

        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            ROS_INFO("%s", "Message published");
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
