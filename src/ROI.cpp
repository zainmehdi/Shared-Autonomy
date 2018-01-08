
/// This program reads live images from camera mounted on Pioneer and find + tracks good features using LK method + user drawn path

#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

const int MAX_COUNT = 30;
bool needToInit = false;
bool nightMode = false;

Mat gray, prevGray, image, frame;
vector<Point2f> points[3];

static void help()
{
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
            "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
            "\tn - switch the \"night\" mode on/off\n"
            "To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;
bool drag;
int circle_radius=2;
vector<Point> line_points;
vector<Point> transformed_points;
vector<Point> line_points_circle;
cv::Scalar color(0, 0, 255); //red
int camera_height=40;
int camera_pitch=45;
int Hfov=60;
int Vfov=47;
int m=640;
int n=480;



Point IPM(int u , int v)
{
    int x=(  camera_height/(atan(camera_pitch-Hfov)+  (u*(2*Hfov/(m-1))) )) *(cos(-Vfov+ (v*2*Vfov/(n-1))) );
    int y=(  camera_height/(atan(camera_pitch-Hfov)+  (u*(2*Hfov/(m-1))) )) *(sin(-Vfov+ (v*2*Vfov/(n-1))) );
    return Point(x,y);

}

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN && !drag )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
        drag=true;
    }

    else if( event == EVENT_MOUSEMOVE && drag )
    {

        line_points.push_back(CvPoint(x,y));

    }

    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }


}

void imagecb(const sensor_msgs::ImageConstPtr& msg)
{

//    for(;;)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_ptr->image.copyTo(frame);

        if (frame.empty())
            return;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        if (nightMode)
            image = Scalar::all(0);

        if (needToInit) {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
            addRemovePt = false;
        } else if (!points[0].empty()) {
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;

            for(auto y:points[0])
            {
                points[3].push_back(y);
            }

            /// Find rigid transformation between points of optical flow and use it to transform your own points

            Mat transformation=estimateRigidTransform(points[0],points[1],1);
            cout<<"Transform: "<< transformation<<"\n";
            for (i = k = 0; i < points[1].size(); i++) {
                if (addRemovePt) {
                    if (norm(point - points[1][i]) <= 5) {
                        addRemovePt = false;
                        continue;
                    }
                }

                if (!status[i])
                    continue;

                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);

            }

           if(points[1].size()>3)
           {
               transform(line_points,transformed_points,transformation);
           }else
           {
               transformed_points=line_points;
           }

            for(auto index:transformed_points)
            {

                circle(image,Point(index), circle_radius, CV_RGB(255, 0, 0),1,8,0);
                line_points=transformed_points;
            }

            for(int i = 0; i < points[3].size() - 1; ++i)
            {
                cv::line(image, points[3][i], points[3][i+1], color);
            }

            points[1].resize(k);

        }



        if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char) waitKey(10);
        if (c == 27)
            return;
        switch (c) {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                line_points.clear();
                line_points_circle.clear();
                points[3].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }

}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "local_autonomyy");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    image_sub = it.subscribe("/camera/image_raw", 1,
                             imagecb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );

    help();




    ros::spin();

    return 0;
}