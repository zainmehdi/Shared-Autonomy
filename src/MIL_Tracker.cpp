//
// Created by zain on 17. 11. 22.
//



/// Tracker with image coming from Mobile robot ///


#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "fstream"
#include <cstring>
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    Ptr<Tracker> tracker;
    Rect2d roi;
    Mat frame;
    bool first_run=true;

public:
    ImageConverter()
            : it_(nh_)
    {



        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);




        //    tracker = TrackerTLD::create();                   // not usable at all;
//            tracker = TrackerKCF::create();                 // works fine
//        tracker = TrackerBoosting::create();
//            tracker = TrackerMIL::create();
        //    tracker = TrackerMedianFlow::create();          //works bad
            tracker = TrackerGOTURN::create();

    }

    ~ImageConverter()
    {

    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

       if(first_run)
       {
           roi=selectROI("tracker",cv_ptr->image);
           if(roi.width==0 || roi.height==0)
               return ;
           tracker->init(cv_ptr->image,roi);
           printf("Start the tracking process, press ESC to quit.\n");
           first_run=false;
       }

        if(cv_ptr->image.rows==0 || cv_ptr->image.cols==0)
            return;
        // update the tracking result
        tracker->update(cv_ptr->image,roi);
        // draw the tracked object
        cv::rectangle(cv_ptr->image, roi, Scalar( 255, 0, 0 ), 2, 1 );
       // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        // show image with the tracked object
        imshow("tracker",cv_ptr->image);
        //quit on ESC button
        if(waitKey(1)==27)
            return;

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};


int main( int argc, char** argv ){
    // show help
    std::ifstream fs("/home/zain/local_autonomy/src/opencv_test/goturn.prototxt", std::ifstream::in);
    if (fs.is_open())
        cout << "WIN";
    else
        cout << "LOST";

    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
