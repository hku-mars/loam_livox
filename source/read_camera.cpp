#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <chrono>
int main( int argc, char **argv )
{
    std::cout << "Hello, this is read_camera" << std::endl;
    std::cout << "Build date " << __DATE__ << std::endl;
    std::cout << "Build time " << __TIME__ << std::endl;

    ros::init( argc, argv, "laserMapping" );
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);

    cv::VideoCapture cap( 0 ); // open the default camera
    if ( !cap.isOpened() )     // check if we succeeded
    {
      ROS_ERROR("Open Camera error! exit node");
        return -1;
    }
    cap.set(CV_CAP_PROP_SETTINGS, 1); //opens camera properties dialog
    
    cap.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
    cap.set( CV_CAP_PROP_FRAME_HEIGHT, 240 );
    std::cout << "MJPG: " << cap.set( cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc( 'M', 'J', 'P', 'G' ) ) << std::endl;
    std::cout << "~~~ Read camera OK ~~~" << std::endl;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    for ( ;; )
    {
        cap >> frame; // get a new frame from camera
        cv::flip(frame, frame, 0);
        cv::flip(frame, frame, 1);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = std::string("world");
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    return 0;
}
