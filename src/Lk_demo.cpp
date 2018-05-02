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
#include <eigen3/Eigen/Dense>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <geometry_msgs/Twist.h>


#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;
using namespace Eigen;
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
Mat img,src;
Mat gray, prevGray, image, frame;
vector<Point2f> points[2],v;
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);
float left_sum,right_sum,up_sum,down_sum;
vector<uchar> status;
geometry_msgs::Twist velocity;
ros::Publisher vel_pub;
bool turn=false;
bool turn_prev=false;


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


void calculate_FOE(vector<Point2f> prev_pts, vector<Point2f> next_pts)
{

    MatrixXf A(next_pts.size(),2);
    MatrixXf b(next_pts.size(),1);
    Point2f tmp;

    for(int i=0;i<next_pts.size();i++)
    {

        if(!status[i])
            continue;
        tmp= next_pts[i]-prev_pts[i];
        A.row(i)<<next_pts[i].x-prev_pts[i].x,next_pts[i].y-prev_pts[i].y;
        b.row(i)<<(prev_pts[i].x*tmp.x)-(prev_pts[i].y*tmp.y);

    }


    Matrix<float,2,1> FOE;
    FOE=((A.transpose()*A).inverse())*A.transpose()*b;

    Point2f c;
    c.x=FOE(0,0);
    c.y=FOE(1,0);
    circle( image, c, 8, Scalar(255,255,0), -1, 8);


}

void Draw_flowVectors(vector<Point2f> prev_pts, vector<Point2f> next_pts)
{

    right_sum=0;
    left_sum=0;
    up_sum=0;
    down_sum=0;

   for(int i=0;i<next_pts.size();i++)
   {
       CvPoint p,q; /*  "p" is the point where the line begins.
/*  "q" is the point where the line stops.
/*  "CV_AA" means antialiased drawing.
/*  "0" means no fractional bits in the center cooridinate or radius.
*/

       if(!status[i])
           continue;

       p.x = (int) prev_pts[i].x;
       p.y = (int) prev_pts[i].y;
       q.x = (int) next_pts[i].x;
       q.y = (int) next_pts[i].y;
       double angle;
       angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
       double hypotenuse;  hypotenuse = sqrt( pow(p.y - q.y,2) + pow(p.x - q.x,2 ));
       q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
       q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
       arrowedLine( image, p, q, Scalar(255,255,255), 1, CV_AA, 0 );




       int mag= sqrt(pow(next_pts[i].x-prev_pts[i].x,2)+pow(next_pts[i].y-prev_pts[i].y,2));

       if(next_pts[i].x > 320)
           right_sum+=mag;
       else
           left_sum+=mag;

       if(next_pts[i].y <329)
           up_sum+=mag;
       else
           down_sum+=mag;



   }

    cout<<"Left Flow "<<left_sum<<" Right Flow "<<right_sum
        <<" Up Flow "<<up_sum<<" Down Flow "<<down_sum<<endl;
    cout<<".........................................."<<endl;


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

        Rect ROI=Rect(300,1,100,220);
        Mat mask;
        Mat mask_image;
        mask = Mat::zeros(gray.size(), CV_8U);
        mask_image=Mat(mask,ROI);
        mask_image=Scalar(255,255,255);
        if( nightMode )
            image = Scalar::all(0);

        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {

            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);


            if(!points[0].empty() && !points[1].empty())
            {
                Draw_flowVectors(points[0],points[1]);
               // calculate_FOE(points[0],points[1]);
            }

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


    if(right_sum>left_sum && (abs(right_sum-left_sum)>15))
    {
        velocity.angular.z=25;

    }

    else if(left_sum>right_sum && (abs(right_sum-left_sum)>15))
    {


        velocity.angular.z=-25;

    }

    else
    {
        velocity.angular.z=0;

    }



    vel_pub.publish(velocity);




}

//void imageCb(const sensor_msgs::ImageConstPtr& msg);
//
int main( int argc, char** argv ) {

    ros::init(argc, argv, "LKDEMO");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_sub = it.subscribe("image_raw", 1, imageCb);
    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
 //   namedWindow( "LK Demo", 1 );
 //   setMouseCallback( "LK Demo", onMouse, 0 );
    ros::spin();

    return 0;
}
//
//
//
//void imageCb(const sensor_msgs::ImageConstPtr& msg)
//
//{
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
//    }
//    catch (cv_bridge::Exception &e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//
//    src=cv_ptr->image;
//
//
//
//    if( src.empty() )
//        return;
//
//
//    // Change the background from white to black, since that will help later to extract
//    // better results during the use of Distance Transform
//    for( int x = 0; x < src.rows; x++ ) {
//        for( int y = 0; y < src.cols; y++ ) {
//            if ( src.at<Vec3b>(x, y) == Vec3b(255,255,255) ) {
//                src.at<Vec3b>(x, y)[0] = 0;
//                src.at<Vec3b>(x, y)[1] = 0;
//                src.at<Vec3b>(x, y)[2] = 0;
//            }
//        }
//    }
//    // Show output image
//    imshow("Black Background Image", src);
//    // Create a kernel that we will use for accuting/sharpening our image
//    Mat kernel = (Mat_<float>(3,3) <<
//                                   1,  1, 1,
//            1, -8, 1,
//            1,  1, 1); // an approximation of second derivative, a quite strong kernel
//    // do the laplacian filtering as it is
//    // well, we need to convert everything in something more deeper then CV_8U
//    // because the kernel has some negative values,
//    // and we can expect in general to have a Laplacian image with negative values
//    // BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
//    // so the possible negative number will be truncated
//    Mat imgLaplacian;
//    Mat sharp = src; // copy source image to another temporary one
//    filter2D(sharp, imgLaplacian, CV_32F, kernel);
//    src.convertTo(sharp, CV_32F);
//    Mat imgResult = sharp - imgLaplacian;
//    // convert back to 8bits gray scale
//    imgResult.convertTo(imgResult, CV_8UC3);
//    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
//    // imshow( "Laplace Filtered Image", imgLaplacian );
//    imshow( "New Sharped Image", imgResult );
//    src = imgResult; // copy back
//    // Create binary image from source image
//    Mat bw;
//    cvtColor(src, bw, CV_BGR2GRAY);
//    threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//    imshow("Binary Image", bw);
//    // Perform the distance transform algorithm
//    Mat dist;
//    distanceTransform(bw, dist, CV_DIST_L2, 3);
//    // Normalize the distance image for range = {0.0, 1.0}
//    // so we can visualize and threshold it
//    normalize(dist, dist, 0, 1., NORM_MINMAX);
//    imshow("Distance Transform Image", dist);
//    // Threshold to obtain the peaks
//    // This will be the markers for the foreground objects
//    threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
//    // Dilate a bit the dist image
//    Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
//    dilate(dist, dist, kernel1);
//    imshow("Peaks", dist);
//    // Create the CV_8U version of the distance image
//    // It is needed for findContours()
//    Mat dist_8u;
//    dist.convertTo(dist_8u, CV_8U);
//    // Find total markers
//    vector<vector<Point> > contours;
//    findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//    // Create the marker image for the watershed algorithm
//    Mat markers = Mat::zeros(dist.size(), CV_32SC1);
//    // Draw the foreground markers
//    for (size_t i = 0; i < contours.size(); i++)
//        drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
//    // Draw the background marker
//    circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
//    imshow("Markers", markers*10000);
//    // Perform the watershed algorithm
//    watershed(src, markers);
//    Mat mark = Mat::zeros(markers.size(), CV_8UC1);
//    markers.convertTo(mark, CV_8UC1);
//    bitwise_not(mark, mark);
////    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
//    // image looks like at that point
//    // Generate random colors
//    vector<Vec3b> colors;
//    for (size_t i = 0; i < contours.size(); i++)
//    {
//        int b = theRNG().uniform(0, 255);
//        int g = theRNG().uniform(0, 255);
//        int r = theRNG().uniform(0, 255);
//        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
//    }
//    // Create the result image
//    Mat dst = Mat::zeros(markers.size(), CV_8UC3);
//    // Fill labeled objects with random colors
//    for (int i = 0; i < markers.rows; i++)
//    {
//        for (int j = 0; j < markers.cols; j++)
//        {
//            int index = markers.at<int>(i,j);
//            if (index > 0 && index <= static_cast<int>(contours.size()))
//                dst.at<Vec3b>(i,j) = colors[index-1];
//            else
//                dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
//        }
//    }
//    // Visualize the final image
//    imshow("Final Result", dst);
//
//}