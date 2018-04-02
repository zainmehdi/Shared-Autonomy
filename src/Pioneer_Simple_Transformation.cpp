//
// Created by kari on 18. 1. 3.
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
vector<cv::Mat> transformation_bw_line;
Mat transformation_bw_goal_nf;
const int MAX_COUNT = 1;
bool needToInit = false;
bool nightMode = false;
bool features_found=false;
int line_point_size;
int n=0;
double th_prev =0;
Mat gray, prevGray, image, frame;
vector<Point2f>points[2];
Point2f point;
Point2f point1;
Point2f desired_point;
Point2f previous,current;
Point2f nearest_feature;
bool addRemovePt = false;
bool feature_selected=false;
int circle_radius=2;
vector<Point> line_points;
vector<Point> line_points_world;
vector<Point> transformed_points;
vector<Point> line_points_circle;
int run=1;

Point Target_point;
Point Target_point_prev;
bool drag;

double camera_height=40;
double camera_pitch = 45;
ros::Publisher vel_pub;
bool path_drawn=false;
bool path_feature_found=false;
geometry_msgs::Twist v;




void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{

    if( event == EVENT_LBUTTONDOWN && !drag )
    {
        drag=true;
    }

  if( event == EVENT_LBUTTONDOWN && path_drawn )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
        cout<<"Point added \n";
    }



    else if( event == EVENT_MOUSEMOVE && !path_drawn && drag)
    {

        line_points.push_back(CvPoint(x,y));

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
    double theta=atan(Ty/Tx);

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
    return sqrt(pow(b.x-a.x,2)+pow(b.y-a.y,2));
}

double derivative(Point a, Point b)
{
    return ((b.y-a.y)/(b.x-a.x));
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {

     /////////////////




    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;



    if (!path_drawn)
        {

            cout<<"\n ****************************************\n"
                <<"\nDraw the desired path\n"
                <<"\n*****************************************\n";


            if( frame.empty() )
                return;

            frame.copyTo(image);
            cvtColor(image, gray, COLOR_BGR2GRAY);
            if (nightMode)
                image = Scalar::all(0);


            for(auto index:line_points)
            {

                circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);
            }



            char c = (char) waitKey(10);
            if (c == 27)
                path_drawn=true;
            switch (c) {
                case 'c':
                    line_points.clear();
                    cout<<"Points Cleared \n";
                    break;
            }

            imshow("LK Demo", image);

            v.linear.y = 0;
            v.linear.x = 0;
            v.angular.z= 0;

            vel_pub.publish(v);


        }





    if (path_drawn  && !path_feature_found ) {


        cout<<"\n ***************************************************\n"
            <<"\nClick on the goal position to select nearest feature\n"
            <<"\nPress any key to proceed\n"
            <<"\n****************************************************\n";


        waitKey(10);


        if( frame.empty() )
            return;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        if (nightMode)
            image = Scalar::all(0);

        if (needToInit)
            {
                // automatic initialization
                goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
                cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                addRemovePt = false;
                features_found=true;
            }


            if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
                vector<Point2f> tmp;
                tmp.push_back(point);
                cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
                points[1].push_back(tmp[0]);

                addRemovePt = false;
                features_found=true;
            }


//        if(features_found)
//            circle(image,points[1].back() , 3, Scalar(0, 255, 0), -1, 8);


            cout<<"SIze: "<<points[1].size()<<"\n";
            for(auto index:line_points)
            {

                circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);
            }

            needToInit = false;
            imshow("LK Demo", image);

            char c = (char) waitKey(10);
            if (c == 27)
                path_feature_found=true;
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
                    features_found=false;
                    cout<<"Points Cleared \n";
                    break;
                case 'n':
                    nightMode = !nightMode;
                    break;
            }

            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);



        }


    if (path_feature_found && run==1) {


        cout << "While loop is broken Entering Visual Servo loop \n";
        line_point_size = line_points.size();


        double distance_current;
        double distance_previous = 2000000;
        int nearest_index;
        if (features_found) {

            transformation_bw_goal_nf = transformation_calculate(line_points.back(), points[0].back());

            cout << "\nTFG:" << transformation_bw_goal_nf << "\n";
        }

        waitKey();

        transformation_bw_line.resize(line_points.size());
        for (int i = 0; i < line_points.size(); i++) {
            transformation_bw_line[i].push_back(
                    transformation_calculate(line_points[line_points.size() - 1], line_points[i]));
        }

        run++;

    }

        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////



    if(path_drawn && path_feature_found) {

        if( frame.empty() )
            return;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        if (nightMode)
            image = Scalar::all(0);


        if (!points[0].empty()) {
                vector<uchar> status;
                vector<float> err;
                if (prevGray.empty())
                    gray.copyTo(prevGray);
                calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                     3, termcrit, 0, 0.001);



                circle(image, points[1][0], 3, Scalar(0, 255, 0), -1, 8);
                circle(image, desired_point, 5, Scalar(0, 150, 0), -1, 8);



                Mat temp,temp_feature,line;
                temp.create(3,1,cv::DataType<double>::type) ;
                temp_feature.create(3,1,cv::DataType<double>::type) ;
                line.create(3,1,cv::DataType<double>::type) ;

                temp.at<double>(0,0)=points[1][0].x;
                temp.at<double>(1,0)=points[1][0].y;
                temp.at<double>(2,0)=1;


                cout<<"\nTFG_INV:"<<transformation_bw_goal_nf.inv(DECOMP_LU)<<"\n";
                cout<<"\ntemp:"<<temp<<"\n";

                temp_feature=transformation_bw_goal_nf.inv(DECOMP_LU)*temp;

                for(int i=line_points.size()-1;i>=0;i--)
                {
                    line=transformation_bw_line[i]*temp_feature;
                    line_points[i].x=line.at<double>(0,0);
                    line_points[i].y=line.at<double>(1,0);

                }






                for(auto index:line_points)
                {

                    circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);


//                   line_points=transformed_points;

                }


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




            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);



            //////////////////////////////OpenCV part ends here /////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////


        v.linear.y = (desired_point.x-line_points[n].x) / 800;
        v.linear.x = (desired_point.y-line_points[n].y) / 1000;
        v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));

//        v.angular.z=1;
        vel_pub.publish(v);









            if(distance(desired_point,line_points[n]) < 5)
            {
                n++;
                if(n==line_points.size()-3)
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
                    features_found=false;
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