//
// Created by zain on 17. 12. 17.
//

// Program that uses transformation concept

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
Size subPixWinSize(10,10), winSize(30,30);
vector<cv::Mat> transformation_bw_line;
Mat transformation_bw_goal_nf;
int MAX_COUNT = 200;
bool needToInit = false;
bool nightMode = false;
bool features_found=false;
double t_y;
int line_point_size;
int n=0;
int width=4;
int height=4;
double th_prev =0;
Mat gray, prevGray, image, frame;
Mat mask;
Mat roi;
vector<Point2f> points[2];
vector<Point2f> point_buffer;
int m=0;
Point2f point;
Point2f point1;
Point2f desired_point;
Point2f previous,current;
Point2f nearest_feature;
bool addRemovePt = false;
bool feature_selected=false;
bool first_run=true;
int circle_radius=2;
vector<Point> line_points;
vector<Point> line_points_temp;
vector<Point> line_points_world;
vector<Point> transformed_points;
vector<Point> line_points_circle;
vector<Point> selected_points;

Point Target_point;
Point Target_point_prev;
Point p;

vector<Point2f> pp;
int e=0;
bool drag;

double camera_height=40;
double camera_pitch = 45;



/*!
  \example tutorial_ros_node_pioneer_visual_servo.cpp

  Example that shows how to create a ROS node able to control the Pioneer mobile robot by IBVS visual servoing with respect to a blob.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point corresponding to the blob center of gravity measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is proportional to the depth Z.

  */

void points_selector(vector<Point> &all_points){


    for(auto i:all_points)
    {

        if(first_run)
        {
            p=i;
            first_run=false;
            selected_points.push_back(i);
            continue;
        }

        int distance=sqrt(pow(p.x-i.x,2)+pow(p.y-i.y,2));
//        if(distance>(2*circle_radius*25))
        if(distance>5)
        {

            selected_points.push_back(i);
            cout<<"Point pushed \n";
            p=i;
        }

    }
    line_points_temp.clear();

}

 void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN && !drag )
    {
//        point = Point2f((float)x, (float)y);
//        addRemovePt = true;
        drag=true;
    }

    else if( event == EVENT_MOUSEMOVE && drag )
    {

        line_points.push_back(CvPoint(x,y));
        line_points_temp.push_back(CvPoint(x,y));
        points_selector(line_points_temp);



    }

    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }

}

Mat transformation_calculate(Point2f x, Point2f y)
{
    double Tx=y.x-x.x;
    double Ty=y.y-x.y;
    double theta=atan((y.y-x.y)/(y.x-x.x));

    Mat T=Mat_<double>(3,3);
    T.at<double>(0,0)=cos(theta*3.1428/180);
    T.at<double>(0,1)=-sin(theta*3.1428/180);
    T.at<double>(0,2)=Tx;
    T.at<double>(1,0)=sin(theta*3.1428/180);
    T.at<double>(1,1)=cos(theta*3.1428/180);
    T.at<double>(1,2)=Ty;
    T.at<double>(2,0)=0;
    T.at<double>(2,1)=0;
    T.at<double>(2,2)=1;

    return T;
}

double distance(Point a,Point b)
{
    return sqrt(pow(b.x-a.x,2)+(b.y-a.y,2));
}

double derivative(Point a, Point b)
{
    return ((b.y-a.y)/(b.x-a.x));
}

int main(int argc, char **argv) {
    try {
        vpImage<unsigned char> I; // Create a gray level image container
        double depth = 1;
        double lambda = 0.6;
        double coef = 1. / 6.77; // Scale parameter used to estimate the depth Z of the blob from its surface
        desired_point = Point(300, 475);

        namedWindow("LK Demo", 1);
        setMouseCallback("LK Demo", onMouse, 0);
        vpROSRobotPioneer robot;
        robot.setCmdVelTopic("/RosAria/cmd_vel");
        robot.init(argc, argv);

        // Wait 3 sec to be sure that the low level Aria thread used to control
        // the robot is started. Without this delay we experienced a delay (arround 2.2 sec)
        // between the velocity send to the robot and the velocity that is really applied
        // to the wheels.
        vpTime::sleepMs(3000);

        std::cout << "Robot connected" << std::endl;

        // Camera parameters. In this experiment we don't need a precise calibration of the camera
        vpCameraParameters cam;

        // Create a grabber based on libdc1394-2.x third party lib (for firewire cameras under Linux)
        vpROSGrabber g;
        g.setCameraInfoTopic("/usb_cam/camera_info");
        g.setImageTopic("/usb_cam/image_raw");
        g.setRectify(true);
        g.open(argc, argv);
        // Get camera parameters from /camera/camera_info topic
        if (g.getCameraInfo(cam) == false)
            cam.initPersProjWithoutDistortion(600, 600, I.getWidth() / 2, I.getHeight() / 2);



        /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
        ///////////////////////////////////////////////////////////////////////////////////


        cout << "\n ****************************************\n"
             << "\nDraw the desired path\n"
             << "\n*****************************************\n";


        while (1) {

            g.acquire(I);
            vpImageConvert::convert(I, image);
            image.copyTo(frame);
            circle(image, desired_point, 7, Scalar(0, 150, 0), -1, 8);


            if (frame.empty())
                break;

            //  frame.copyTo(image);
            //  cvtColor(image, gray, COLOR_BGR2GRAY);

            frame.copyTo(gray);
            if (nightMode)
                image = Scalar::all(0);

            for (auto index:line_points) {

                circle(image, Point(index), 5, CV_RGB(255, 255, 0), 1, 8, 0);
            }

            for (auto g:selected_points) {
                rectangle(image, Point(g.x - width / 2, g.y - height / 2), Point(g.x + width / 2, g.y + height / 2),
                          CV_RGB(255, 0, 0), 1, 8, 0);

            }

            char c = (char) waitKey(10);
            if (c == 27)
                break;
            switch (c) {
                case 'c':
                    line_points.clear();
                    selected_points.clear();
                    break;
            }

            imshow("LK Demo", image);

        }


        cout << "\n ***************************************************\n"
             << "\nClick on the goal position to select nearest feature\n"
             << "\nPress any key to proceed\n"
             << "\n****************************************************\n";


        while (1) {


            waitKey(10);


            g.acquire(I);
            vpImageConvert::convert(I, image);
            image.copyTo(frame);
            circle(image, desired_point, 7, Scalar(0, 150, 0), -1, 8);


            if (frame.empty())
                break;


            frame.copyTo(gray);
            if (nightMode)
                image = Scalar::all(0);


            if (needToInit) {


                for (auto it:selected_points) {

                    g.acquire(I);
                    vpImageConvert::convert(I, image);
                    mask = Mat::zeros(image.size(), CV_8U);
                    roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
                    roi = Scalar(255, 255, 255);

                    // automatic initialization
                    goodFeaturesToTrack(gray, point_buffer, MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
//                    cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);
                    imshow("Mask", mask);

                    for (auto index:point_buffer) {

                        points[1].push_back(index);

                    }
                }


                addRemovePt = false;
                features_found = true;
            }


            if (features_found) {
                for (auto q:points[1]) {
                    circle(image, q, 3, Scalar(0, 255, 0), -1, 8);
                    imshow("LK Demo", image);

                }

            }

            needToInit = false;
            for (auto index:line_points) {

                circle(image, Point(index), 8, CV_RGB(255, 255, 0), 0.5, 8, 0);
            }

            needToInit = false;
            imshow("LK Demo", image);

            char c = (char) waitKey(10);
            if (c == 27)
                break;
            switch (c) {
                case 'r':
                    needToInit = true;
                    pp.clear();
                    e = 0;
                    cout << "Re Initialized \n";
                    break;

                case 'c':
                    points[0].clear();
                    points[1].clear();
                    line_points.clear();
                    line_points_circle.clear();
                    selected_points.clear();
                    features_found = false;
                    cout << "Points Cleared \n";
                    break;
                case 'n':
                    nightMode = !nightMode;
                    break;
            }

            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);

        }

        cout << "While loop is broken Entering Visual Servo loop \n";
        line_point_size = line_points.size();





        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////


        // Create an image viewer
        vpDisplayX d(I, 10, 10, "Current frame");
        vpDisplay::display(I);
        vpDisplay::flush(I);


        while (1) {

            cout << "\n**********************************\n"
                 << "\n Visual Servoing Loop has started\n"
                 << "\n**********************************\n"
                 << "\nPress any key to continue\n";


            g.acquire(I);


            /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
            ///////////////////////////////////////////////////////////////////////////////////

            vpImageConvert::convert(I, image);
            image.copyTo(frame);


            if (frame.empty())
                break;


            frame.copyTo(gray);
            if (nightMode)
                image = Scalar::all(0);

            if (needToInit) {


                for (auto it:selected_points) {

                    g.acquire(I);
                    vpImageConvert::convert(I, image);
                    mask = Mat::zeros(image.size(), CV_8U);
                    roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
                    roi = Scalar(255, 255, 255);

                    // automatic initialization
                    goodFeaturesToTrack(gray, point_buffer, MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
                    cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);
                    imshow("Mask", mask);

                    for (auto index:point_buffer) {

                        points[1].push_back(index);

                    }
                }


                addRemovePt = false;
                features_found = true;
            }


         else if (!points[0].empty()) {
                vector<uchar> status;
                vector<float> err;
                if (prevGray.empty())
                    gray.copyTo(prevGray);
                calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                     3, termcrit, 0, 0.001);


//
//             if(!first_run)
//             {
//                 Mat rig_transformation= estimateRigidTransform(prevGray,gray,1);
//                t_y =rig_transformation.at<double>(1,2);
//             }
                size_t i, k;
                for( i = k = 0; i < points[1].size(); i++ )
                {

                    points[1][k++] = points[1][i];
                    circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
                }
                points[1].resize(k);


                circle(image, desired_point, 5, Scalar(0, 150, 0), -1, 8);

            }
                needToInit = false;

                std::swap(points[1], points[0]);
                cv::swap(prevGray, gray);


                //////////////////////////////OpenCV part ends here /////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////

                vpColVector v(2);


                cout << "Point 0 size: " << points[0].size() << endl;

//            map <double,Point2f> distance_points;
//            vector<double> dist;
//            vector<Point2f> sorted_points;
//
//
//            for(auto t:points[1])
//            {
//                dist.push_back(distance(desired_point,t));
//                distance_points[distance(desired_point,t)]=t;
//                cout<<"unsorted points:"<<distance(desired_point,t)<<"\n";
//            }
//
//
//            sort(dist.begin(),dist.end());
//
//
//            for(auto f:dist)
//            {
//                sorted_points.push_back(distance_points.at(f));
//                cout<<"sorted points:"<<f<<"\n";
//            }



//
//            v[1] = (-transformed_points[n].x + desired_point.x)/400;
//            v[0] = (-transformed_points[n].y+280 + desired_point.y)/800;


//            v[1] = (-line_points[n].x + desired_point.x)/400;
//            v[0] = (-line_points[n].y + desired_point.y)/800;

                v[1] = (-points[0][n].x + desired_point.x) / 400;
                v[0] = (-points[0][n].y + desired_point.y) / 800;

                circle(image, points[0][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);
//            v[0] = (-line_points_world[n].y + desired_point.y)/600;

//            cout<<"Current Point image : "<<transformed_points[n]<<endl;
//            cout<<"Current Point robot: "<<line_points_world[n]<<endl;



//            auto done = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double> elapsed = done - started;
//            std::cout <<"Time :"<< elapsed.count()<<endl;

//            v[1] = (-line_points[n].x + desired_point.x)*0.002+ 2*((current.x-previous.x)/1200);
//            v[0] = (-line_points[n].y + desired_point.y)*0.002 +0.5*((current.y-previous.y)/1200);



                imshow("LK Demo", image);

                robot.setVelocity(vpRobot::REFERENCE_FRAME, v);


                first_run = false;



//            if(distance(desired_point,line_points[n]) < 5)
            if(distance(desired_point,points[0][n]) < 3)
            {
//
                n++;
                if(n==points[0].size())
                    waitKey();

               // needToInit=true;

            }





                char c = (char) waitKey(10);
                if (c == 27)
                    break;
                switch (c) {
                    case 'r':
                        needToInit = true;
                        pp.clear();
                        e = 0;
                        cout << "Re Initialized \n";
                        break;

                    case 'c':
                        points[0].clear();
                        points[1].clear();
                        line_points.clear();
                        line_points_circle.clear();
                        features_found = false;
                        cout << "Points Cleared \n";
                        break;
                    case 'n':
                        break;
                }

        }
        return 0;
    }

    catch (vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
        return 1;
    }
}