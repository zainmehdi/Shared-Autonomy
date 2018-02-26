//
// Created by zain on 17. 12. 17.
//

// Program that uses images coming from unity

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
#include "geometry_msgs/Twist.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/features2d.hpp"


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv::xfeatures2d;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(30,30);
vector<cv::Mat> transformation_bw_line;
Mat transformation_bw_goal_nf;
int MAX_COUNT = 500;
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
geometry_msgs::Twist v;

bool path_drawn=false;
bool feature_on_path_found=false;
bool path_feature_found=false;

ros::Publisher vel_pub;
Point Target_point;
Point Target_point_prev;
Point p;

vector<Point2f> pp;
int e=0;
bool drag;

double camera_height=40;
double camera_pitch = 45;
Ptr<FeatureDetector> Surf;
Ptr<SURF> Surf_detector;
std::vector<KeyPoint> keypoints[2];
std::vector<Mat> surf_descriptor[2];



/// Function that selects points to be drawn on image as path. It takes as input
/// all points that user draws using mouse and then selects certain number of points
/// using distance as threshold. In the first run distance criteria is overlooked and first
/// point is directly added. In the upcmoing iterations whenever the distance is greater then 5 the reference point
/// i.e. the point from where the distance is calculated is updated. This is done beacuse we are drawing rectangles
/// and we dont want them to overlap. Distance criteria is decided based on the size of rectangle we want to draw.
/// Though this rectangle drawing is not needed here it was used in previous version (SURF version) to draw regions
/// and extract features in those regions (Bounding boxes).
/// Update: I sudden revelation came to me that these rectangles are used to extract a mask which is later used
/// to extract features. (This is what happens when you make dozen versions of your code none of which works and you
/// look at your code after a gap of few weeks. Me and my procrastination :p


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

/// Mouse callback function (Takes care of mouse clicks etc)

 void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN && !drag )
    {

        drag=true;
    }

    else if( event == EVENT_MOUSEMOVE && drag )
    {

        line_points.push_back(CvPoint(x,y));

        // I am not sure why I am using line_points_temp here. Maybe because I want to keep my buffer free for every iteration
        // while I also want to retain the actual line points for path reconstruction until I clear them out.

        line_points_temp.push_back(CvPoint(x,y));
        points_selector(line_points_temp);



    }

    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }

}

/// Function that calculates transformation matrix given two 2D points in image

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

/// Function for calculating distance between two 2D points

double distance(Point a,Point b)
{
    return sqrt(pow(b.x-a.x,2)+(b.y-a.y,2));
}

/// Function for calculatinf derivative

double derivative(Point a, Point b)
{
    return ((b.y-a.y)/(b.x-a.x));
}

/// This function is used to find features/reinitialize features as per will
/// This is the part from opencv good features to track algorithm which can be found in the contrib module
/// I have modified it to suit my needs.
void find_features(Mat colr ,Mat gr)
{

    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    for (auto it:selected_points) {


        // The mask that we created using rectangles is used to created a ROI and features are extracted in that ROI.
        // I dont create a single mask using all the points because it ends up my algorithm with just a single feature
        // Instead I use one rectangle at a time and iterate through all rectangles to find features. This also allows me
        // to avoid skipping and ROI and find greater number of points.
        mask = Mat::zeros(gray.size(), CV_8U);
        roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
        roi = Scalar(255, 255, 255);


        // finding features
        goodFeaturesToTrack(gray, point_buffer, MAX_COUNT, 0.01, 1, mask, 3, 5, 0, 0.04);


        // its used to refine corner search and get even better approximate but somehow it doesnt work in my case.

//                cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);


        for (auto index:point_buffer) {

            points[1].push_back(index);

        }
    }


    // This is for drawing mask. ( Not needed just shashka)

    mask = Mat::zeros(gray.size(), CV_8U);
    for (auto it:selected_points) {

        roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
        roi = Scalar(255, 255, 255);
    }
    imshow("Mask", mask);


    features_found = true;
    needToInit=false;


}

/// This function acquires image from incoming ros topic and uses cv_bridge to convert
/// it into Mat

void acquire_image(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;
}

/// This is the visual servoing loop that calculates optical flow and performs the task of following the path.

void visual_servo()
{


    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    if (nightMode)
        image = Scalar::all(0);

    if (needToInit) {

      find_features(frame,gray);
    }


    else if (!points[0].empty()) {

        vector<uchar> status;
        vector<float> err;
        if (prevGray.empty())
            gray.copyTo(prevGray);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                             3, termcrit, 0, 0.001);


        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {

            points[1][k++] = points[1][i];
            circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);



    }
    needToInit = false;

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);



    circle(image, points[0][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

    v.linear.y = (-points[0][n].x + desired_point.x) / 1800;
    v.linear.x = (-points[0][n].y + desired_point.y) / 1200;
//  v.linear.z = (-points[0][n].y + desired_point.y) / 2000;
    v.angular.z= (atan2(v.linear.y,v.linear.x));
    vel_pub.publish(v);


    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

    imshow("LK Demo", image);

    //   robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

//        cout << "n: " << n << endl;
    first_run = false;



    if(distance(desired_point,points[0][n]) < 1)
    {

        n++;
        if(n==points[0].size())
            waitKey();


    }





    char c = (char) waitKey(20);
    if (c == 27)
        return;
    switch (c) {



        case 's':
        {
            imwrite("/home/kari/surf_test1.jpg",gray);
            imwrite("/home/kari/surf_test_mask.jpg",mask);
            cout<<"Image 1 saved \n";
            break;

        }

        case 'd':
        {
            imwrite("/home/surf_test2.jpg",gray);
            cout<<"Image 2 saved \n";
            break;

        }

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
            n=0;
            features_found = false;
            cout << "Points Cleared \n";
            path_drawn= false;
            feature_on_path_found=false;
            break;
        case 'n':
            n++;
    }

}


/// This function tries to find features on the drawn path

void path_drawn_features_nishta()
{


    waitKey(10);

    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);
    if (nightMode)
        image = Scalar::all(0);


    // needtoinit variable decided if user wants to reinitialize the features. In the first run it will always be true

    if (needToInit) {

        find_features(frame,gray);
    }


    // Drawing features that were found

    if (features_found) {
        for (auto q:points[1]) {
            circle(image, q, 3, Scalar(0, 255, 0), -1, 8);
            imshow("LK Demo", image);

        }

    }


    // Drawing reference point

    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);


   // Drawing indicator circle that shows which point is being followed. Variable n decides that and is updated in button press bu user
    if (!points[1].empty())
        circle(image, points[1][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

    imshow("LK Demo", image);

    char c = (char) waitKey(30);
    if (c == 27)
    {

        feature_on_path_found=true;
        return;

    }

    switch (c) {


        case 's':
        {
            imwrite("/home/kari/surf_test1.jpg",gray);
            imwrite("/home/kari/surf_test_mask.jpg",mask);
            cout<<"Image 1 saved \n";
            break;

        }

        case 'r':
            needToInit = true;
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
            path_drawn= false;
            n=0;
            feature_on_path_found=false;
            break;
        case 'n':
            //  nightMode = !nightMode;
            n++;
            break;
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
}



/// Function that takes care of drawing path on the incoming image and then transfers control to the part

void path_not_yet_drawn()
{
    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);
    if (nightMode)
        image = Scalar::all(0);

 // Drawing circles to show the path user drew

    for (auto index:line_points) {

        circle(image, Point(index), 5, CV_RGB(255, 255, 0), 1, 8, 0);
    }

 // Drawing rectangles around the path. Kind of redundant here but who cares its my code :p

    for (auto g:selected_points) {
        rectangle(image, Point(g.x - width / 2, g.y - height / 2), Point(g.x + width / 2, g.y + height / 2),
                  CV_RGB(255, 0, 0), 1, 8, 0);

    }

    char c = (char) waitKey(10);
    if (c == 27)
    {
        path_drawn=true;

    }

    switch (c) {
        case 'c':
            line_points.clear();
            points[0].clear();
            points[1].clear();
            selected_points.clear();
            path_drawn= false;
            break;
    }


    // This is the desired point of reference point that shows position of the robot in image
    // and tries to follow the points we draw on screen

    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);
    imshow("LK Demo", image);

    // Making sure that the velocity sent to robot is zero. It may look useless here but when we are in
    // later cycles if execution and clear points and come bacj to this thread then v variable will have
    // velocity value of that thread (non zero) and will be continuously sent to the robot which is not
    // what we want as during this stage of execution robot should be stationary so we are inforcing our
    // static robot state here.

    v.linear.y = 0;
    v.linear.x = 0;
    v.angular.z= 0;

    vel_pub.publish(v);
}


/// Callback function for incoming images

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{



    acquire_image(msg);


    if (!path_drawn) {

        cout << "\n ****************************************\n"
             << "\nDrawing path on the image\n"
             << "\n*****************************************\n";

        path_not_yet_drawn();

    }



    if (path_drawn  && !feature_on_path_found ) {


        cout << "\n ****************************************\n"
             << "\nFinding Features on Drawn Path\n"
             << "\n*****************************************\n";

        path_drawn_features_nishta();
    }




    if(path_drawn && feature_on_path_found) {



           cout << "\n**********************************\n"
                << "\n Visual Servoing Loop has started\n"
                << "\n**********************************\n";


            visual_servo();

    }
}



int main(int argc, char **argv) {


    ros::init(argc, argv, "unity_autonomy");
    ros::NodeHandle nh;
    desired_point = Point(320, 475);
    image_transport::ImageTransport it(nh);

    Surf_detector=SURF::create(400);
    image_transport::Subscriber image_sub;
    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    image_sub = it.subscribe("image_raw", 1,imageCb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    ros::spin();
    return 0;
}