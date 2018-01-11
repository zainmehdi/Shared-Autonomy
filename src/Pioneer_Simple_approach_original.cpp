

//
// Created by zain on 17. 12. 17.
//


// This is the first implementation in which I track using all keypoints

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

int main(int argc, char **argv) {
    try {
        vpImage<unsigned char> I; // Create a gray level image container
        double depth = 1;
        double lambda = 0.6;
        double coef = 1. / 6.77; // Scale parameter used to estimate the depth Z of the blob from its surface
        desired_point = Point(300, 450);

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

            needToInit = false;
            imshow("LK Demo", image);

            char c = (char) waitKey(10);
            if (c == 27)
                break;
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



            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);

        }

        cout<<"While loop is broken here \n";
        line_point_size=line_points.size();

        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////


        // Create an image viewer
        vpDisplayX d(I, 10, 10, "Current frame");
        vpDisplay::display(I);
        vpDisplay::flush(I);


        while (1) {
            // Acquire a new image
//            auto started = std::chrono::high_resolution_clock::now();

            g.acquire(I);


            /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
            ///////////////////////////////////////////////////////////////////////////////////

            vpImageConvert::convert(I, image);
            image.copyTo(frame);


            if (frame.empty())
                break;

//                frame.copyTo(image);
//                cvtColor(image, gray, COLOR_BGR2GRAY);

            frame.copyTo(gray);
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
                cout<<"Transformnation:" <<transformation<<endl;

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
//                    cout << "Points Location: " << points[1][i] << "\n";
                }



                transform(line_points,transformed_points,transformation);
                line_points=transformed_points;
//
//                if(points[1].size()>3)
//                {
//                    transform(line_points,transformed_points,transformation);
//                }else
//                {
//                    transformed_points=line_points;
//                }



                for(auto index:line_points)
                {

                    circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);


//                   line_points=transformed_points;


                }


//                for(auto r:line_points_world)
//                {
//
//                  r.y=r.y-70;
//
//                }


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

            vpColVector v(2);
//
//            v[1] = (-transformed_points[n].x + desired_point.x)/400;
//            v[0] = (-transformed_points[n].y+280 + desired_point.y)/800;


            v[1] = (-line_points[n].x + desired_point.x)/400;
            v[0] = (-line_points[n].y + desired_point.y)/800;

//            v[0] = (-line_points_world[n].y + desired_point.y)/600;

            cout<<"Desired Point: "<<desired_point<<endl;
//            cout<<"Current Point image : "<<transformed_points[n]<<endl;
//            cout<<"Current Point robot: "<<line_points_world[n]<<endl;



//            auto done = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double> elapsed = done - started;
//            std::cout <<"Time :"<< elapsed.count()<<endl;

//            v[1] = (-line_points[n].x + desired_point.x)*0.002+ 2*((current.x-previous.x)/1200);
//            v[0] = (-line_points[n].y + desired_point.y)*0.002 +0.5*((current.y-previous.y)/1200);





            robot.setVelocity(vpRobot::REFERENCE_FRAME, v);






            if(distance(desired_point,line_points[n]) < 5)
            {
                n++;
                if(n==line_points.size())
                    break;
                // needToInit=true;

            }



//            cout<<"Set velocity: "<<v[1].<<" "<<v[2]<endl;

            th_prev=v[1];

            char c = (char) waitKey(10);
            if (c == 27)
                break;
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
        return 0;

    }

    catch (vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
        return 1;
    }
}