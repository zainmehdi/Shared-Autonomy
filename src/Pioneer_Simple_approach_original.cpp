//
// Created by kari on 18. 1. 1.
//

//
// Created by zain on 17. 12. 17.
//


#include <iostream>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotPioneer.h>
#include "ctime"
#include "chrono"
#include "ros/ros.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
#  define TEST_COULD_BE_ACHIEVED
#endif

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

const int MAX_COUNT = 100;
bool needToInit = false;
bool nightMode = false;
int line_point_size;
int n=0;
double th_prev =0;
Mat gray, prevGray, image, frame;
vector<Point2f> points[2];
Point2f point;
Point2f point1;
Point2f desired_point;
Point2f previous,current;
bool addRemovePt = false;
bool feature_selected=false;
int circle_radius=2;
vector<Point> line_points;
vector<Point> line_points_world;
vector<Point> transformed_points;
vector<Point> line_points_circle;

Point Target_point;
Point Target_point_prev;
bool drag;

double camera_height=40;
double camera_pitch = 45;
ros::Publisher vel_pub;
bool path_drawn=false;
bool features_found=false;
geometry_msgs::Twist v;



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

double distance(Point a,Point b)
{
    return sqrt(pow(b.x-a.x,2)+(b.y-a.y,2));
}

double derivative(Point a, Point b)
{
    return ((b.y-a.y)/(b.x-a.x));
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){


    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;

        /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
        ///////////////////////////////////////////////////////////////////////////////////


    if (!path_drawn)  {

        if( frame.empty() )
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
                    cout << "Points Location: " << points[1][i] << "\n";
                }


                for(auto index:line_points)
                {

                    circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);
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

             circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

            needToInit = false;
            imshow("LK Demo", image);

            char c = (char) waitKey(10);
            if (c == 27)
                path_drawn=true;
            switch (c) {
                case 'r':
                    needToInit = true;
                    cout<<"Re Initialized \n";
                    break;

                case 'c':
                    points[0].clear();
                    points[1].clear();
                    line_points.clear();
                    line_points_circle.clear();
                    path_drawn= false;
                    cout<<"Points Cleared \n";
                    break;
                case 'n':
                    nightMode = !nightMode;
                    break;
            }



            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);

        }

        cout<<"While loop is broken here \n";
        line_point_size=line_points.size();

        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////


    if (path_drawn  ) {
        if( frame.empty() )
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

                /// Find rigid transformation between points of optical flow and use it to transform your own points
                Mat transformation=estimateRigidTransform(points[0],points[1],true);
//                Mat transformation=estimateRigidTransform(points[0],points[1],true);
//                cout<<"Transformnation:" <<transformation<<endl;

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
                    circle(image, desired_point, 5, Scalar(0, 150, 0), -1, 8);
                    point1 = points[1][i];

                }



                transform(line_points,transformed_points,transformation);
                line_points=transformed_points;



                for(auto index:line_points)
                {

                    circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);


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
            features_found = true;

            circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);
            imshow("LK Demo", image);
            waitKey(1);



            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);

            Target_point_prev=Target_point;


            if(points[1].size()<30)
            {
                needToInit=true;
            }

            //////////////////////////////OpenCV part ends here /////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////
        v.linear.y = (-line_points[n].x + desired_point.x) / 800;
        v.linear.x = (-line_points[n].y + desired_point.y) / 1000;
        v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));

//        v.angular.z=1;
        vel_pub.publish(v);



            if(distance(desired_point,line_points[n]) < 5)
            {
                n++;
                if(n==line_points.size())
                    return;
                // needToInit=true;

            }





            char c = (char) waitKey(10);
            if (c == 27)
                return;
            switch (c) {
                case 'r':
                    needToInit = true;
                    cout<<"Re Initialized \n";
                    break;

                case 'c':
                    points[0].clear();
                    points[1].clear();
                    line_points.clear();
                    line_points_circle.clear();
                    cout<<"Points Cleared \n";
                    break;
                case 'n':
                    nightMode = !nightMode;
                    break;
            }


        }


}


int main(int argc, char **argv) {


    ros::init(argc, argv, "unity_autonomy");
    ros::NodeHandle nh;
    desired_point = Point(320, 475);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    image_sub = it.subscribe("image_raw", 1,imageCb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    ros::spin();
    return 0;
}