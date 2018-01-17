////
//// Created by kari on 17. 12. 24.
////
//
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv/cv.hpp"
//
//#include <iostream>
//#include <ctype.h>
//
//using namespace cv;
//using namespace std;
//
//static void help()
//{
//    // print a welcome message, and the OpenCV version
//    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
//            "Using OpenCV version " << CV_VERSION << endl;
//    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
//    cout << "\nHot keys: \n"
//            "\tESC - quit the program\n"
//            "\tr - auto-initialize tracking\n"
//            "\tc - delete all the points\n"
//            "\tn - switch the \"night\" mode on/off\n"
//            "To add/remove a feature point click it\n" << endl;
//}
//
//Point2f point;
//bool addRemovePt = false;
//float h=0.72;
//float theta=0.52;
//float a_u=0.38;
//float a_v=0.68;
//float projected_point[2];
//Mat gray, prevGray, image, frame;
//Mat projection_matrix[3][4],camera_matrix,rot_matrix,trans;
//int m=640;
//int n=480;
//
//void IPM(int u , int v)
//{
//    double x = h/tan((theta-a_u)+u*2.0*a_u/(frame.rows-1))*cos(-a_v+v*2.0*a_v/(frame.cols-1));
//    double y = -h/tan((theta-a_u)+u*2.0*a_u/(frame.rows-1))*sin(-a_v+v*2.0*a_v/(frame.cols-1));
//
//    cout<<"Projected Point:" << x<<" "<<y<<endl;
//    cout<<"Point:" << point.x<<" "<<point.y<<endl;
//
//}
//
//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);
//
//        addRemovePt = true;
//    }
//}
//
//int main( int argc, char** argv )
//{
//
//    cv::Mat P(3,4,cv::DataType<float>::type);
//    P.at<float>(0,0) = 828.32;
//    P.at<float>(1,0) = 0;
//    P.at<float>(2,0) = 0;
//
//    P.at<float>(0,1) = 0;
//    P.at<float>(1,1) = 837.72;
//    P.at<float>(2,1) = 0;
//
//    P.at<float>(0,2) = 329.66;
//    P.at<float>(1,2) = 266.30;
//    P.at<float>(2,2) = 1;
//
//    P.at<float>(0,3) = 0;
//    P.at<float>(1,3) = 0;
//    P.at<float>(2,3) = 0;
//
//    std::cout << "P: " << P << std::endl;
//
//    // Decompose the projection matrix into:
//    cv::Mat K(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
//    cv::Mat R(3,3,cv::DataType<float>::type); // rotation matrix
//    cv::Mat T(4,1,cv::DataType<float>::type); // translation vector
//
//    cv::decomposeProjectionMatrix(P, K, R, T);
//    std::cout << "K: " << K << std::endl;
//    std::cout << "R: " << R << std::endl;
//    std::cout << "T: " << T << std::endl;
//
//
//    Mat tmp=-1*(Mat_<float>(3,1)<<T.at<float>(0,0),T.at<float>(1,0),T.at<float>(2,0));
//    Mat RT = R*tmp;
//
//    std::cout << "RT: " << RT << std::endl;
////    Mat H= K*RT;
////
////    std::cout << "H: " << H << std::endl;
//
//
////    H=K* cv::Mat(R.col(0),R.col(1),T.colRange(0,2));
//
//    VideoCapture cap(0);
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//    Size subPixWinSize(10,10), winSize(31,31);
//
//    const int MAX_COUNT = 500;
//    bool needToInit = false;
//    bool nightMode = false;
//
//    help();
//
//    if( !cap.isOpened() )
//    {
//        cout << "Could not initialize capturing...\n";
//        return 0;
//    }
//
//    namedWindow( "LK Demo", 1 );
//    setMouseCallback( "LK Demo", onMouse, 0 );
//
//    Mat gray, prevGray, image, frame;
//    vector<Point2f> points[2];
//
//    for(;;)
//    {
//        cap >> frame;
//        if( frame.empty() )
//            break;
//
//        frame.copyTo(image);
//        cvtColor(image, gray, COLOR_BGR2GRAY);
//
//        if( nightMode )
//            image = Scalar::all(0);
//
//        if( needToInit )
//        {
//            // automatic initialization
//            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
//            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
//            addRemovePt = false;
//        }
//        else if( !points[0].empty() )
//        {
//            vector<uchar> status;
//            vector<float> err;
//            if(prevGray.empty())
//                gray.copyTo(prevGray);
//            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                                 3, termcrit, 0, 0.001);
//            size_t i, k;
//            for( i = k = 0; i < points[1].size(); i++ )
//            {
//                if( addRemovePt )
//                {
//                    if( norm(point - points[1][i]) <= 5 )
//                    {
//                        addRemovePt = false;
//                        continue;
//                    }
//                }
//
//                if( !status[i] )
//                    continue;
//
//                points[1][k++] = points[1][i];
//                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//                circle( image, point, 3, Scalar(0,255,255), -1, 8);
//                addRemovePt = true;
//            }
//            points[1].resize(k);
//        }
//
//        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//        {
//            vector<Point2f> tmp;
//            tmp.push_back(point);
//            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
//            points[1].push_back(tmp[0]);
//            addRemovePt = false;
//        }
//
//        needToInit = false;
//        imshow("LK Demo", image);
//
//        char c = (char)waitKey(10);
//        if( c == 27 )
//            break;
//        switch( c )
//        {
//            case 'r':
//                needToInit = true;
//                break;
//            case 'c':
//                points[0].clear();
//                points[1].clear();
//                break;
//            case 'n':
//                nightMode = !nightMode;
//                break;
//        }
//
////        cv::Mat p(3,1,cv::DataType<float>::type);
////        p.at<float>(0,0)=point.x;
////        p.at<float>(1,0)=point.y;
////        p.at<float>(2,0)=1;
////
////       Mat d= R*p;
////
////        float X= -RT.at<float>(0,2)/d.at<float>(0,2) + RT.at<float>(0,0);
////        float Y= -RT.at<float>(0,2)/d.at<float>(0,2) + RT.at<float>(0,1);
////
////
////        Mat projected_point;
////        projected_point.create(2,1,cv::DataType<float>::type);
////        projected_point.at<float>(0,0)=X;
////        projected_point.at<float>(1,0)=Y;
////
////
////        cout<<"Image Point:" << point<<endl;
////        cout<<"Projected Point:" << projected_point<<endl;
//
//        IPM(point.y,point.x);
//
//        std::swap(points[1], points[0]);
//        cv::swap(prevGray, gray);
//    }
//
//    return 0;
//}



#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
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


#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

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
Mat img;
Mat gray, prevGray, image, frame;
vector<Point2f> points[2];
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

const int MAX_COUNT = 500;
bool needToInit = false;
bool nightMode = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;



        if( frame.empty() )
            return;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        if( nightMode )
            image = Scalar::all(0);

        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 10 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }

        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char)waitKey(10);
        if( c == 27 )
            return;
        switch( c )
        {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);


}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "LKDEMO");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_sub = it.subscribe("image_raw", 1, imageCb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    ros::spin();
    return 0;
}

