//
// Created by kari on 18. 5. 20.
//

#ifndef PROJECT_PIONEER_SIMPLE_APPROACH_H
#define PROJECT_PIONEER_SIMPLE_APPROACH_H


#include <iostream>
#include <math.h>
#include "ctime"
#include "chrono"
#include "ros/ros.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/tracking.hpp>
#include <iostream>
#include <ctype.h>
#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include "geometry_msgs/Twist.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/features2d.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <std_msgs/String.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv::xfeatures2d;
using namespace message_filters;

Ptr<Tracker> tracker;
Rect2d roii;

enum line_direction{left,right,up};

int w=640;
int h=480;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(30,30);
vector<cv::Mat> transformation_bw_line;
Mat transformation_bw_goal_nf;
int MAX_COUNT = 500;
bool needToInit = false;
bool nightMode = false;
bool features_found=false;
bool tracker_ROI=false;
double t_y;
int line_point_size;
int n=0;
int width=4;
int height=4;
double th_prev =0;
Mat gray, prevGray, image, frame;
Mat mask;
Mat roi;
Rect ROI;
vector<Point2f> points[2];
vector<Point2f> point_buffer;
int m=0;
Point2f point;
Point2f point_of_discontinuity;
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
ros::Publisher mask_pub;
Point Target_point;
Point Target_point_prev;
Point p;

vector<Point2f> pp;
int e=0;
bool alpha,beta;
bool drag;
bool discontinuity=false;


double camera_height=40;
double camera_pitch = 45;
Ptr<FeatureDetector> Surf;
Ptr<SURF> Surf_detector;
std::vector<KeyPoint> keypoints[2];
std::vector<Mat> surf_descriptor[2];
double roll,pitch,yaw;
int yaw_degrees;
float left_sum,right_sum,up_sum,down_sum;
vector<uchar> status;
geometry_msgs::Twist velocity;
geometry_msgs::PoseStamped pose_from_unity;
int decision;
bool obstacle=false;
double c; // arc variable
int test=0;
bool case_1_2_3_4=false;
bool case_5_up_1=false;
bool case_5_up_2=false;

double errror,changeError,totalError,error_prev;


void clear_all()
{
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
    discontinuity=false;
    first_run=true;
}

int picture_counter=1;

/// Function that selects points to be drawn on image as path. It takes as input
/// all points that user draws using mouse and then selects certain number of points
/// using distance as threshold. In the first run distance criteria is overlooked and first
/// point is directly added. In the next iterations whenever the distance is greater then 5 the reference point
/// i.e. the point from where the distance is calculated is updated. This is done because we are drawing rectangles
/// and we dont want them to overlap. Distance criteria is decided based on the size of rectangle we want to draw.
/// Though this rectangle drawing is not needed here it was used in previous version (SURF version) to draw regions
/// and extract features in those regions (Bounding boxes).
/// Update: A sudden revelation came to me that these rectangles are used to extract a mask which is later used
/// to extract features. (This is what happens when you make dozen versions of your code none of which works and you
/// look at your code after a gap of few weeks. Need to cut my sleep ... pheww !!! :p


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
        if(distance>5)
        {
            selected_points.push_back(i);
            p=i;
        }

    }
    line_points_temp.clear();

}


float angle_bw_lines(Point2f a, Point2f b, Point2f c)
{

    float dist1 = sqrt(  pow((a.x-c.x),2) + pow((a.y-c.y),2) );
    float dist2 = sqrt(  pow((b.x-c.x),2) + pow((b.y-c.y),2) );


    float Ax, Ay;
    float Bx, By;
    float Cx, Cy;

    //find closest point to C
    //printf("dist = %lf %lf\n", dist1, dist2);

    Cx = c.x;
    Cy = c.y;
    if(dist1 < dist2)
    {
        Bx = a.x;
        By = a.y;
        Ax = b.x;
        Ay = b.y;


    }else{
        Bx = b.x;
        By = b.y;
        Ax = a.x;
        Ay = a.y;
    }


    float Q1 = Cx - Ax;
    float Q2 = Cy - Ay;
    float P1 = Bx - Ax;
    float P2 = By - Ay;


    float A = acos( (P1*Q1 + P2*Q2) / ( sqrt(P1*P1+P2*P2) * sqrt(Q1*Q1+Q2*Q2) ) );

    A = A*180/M_PI;

    if(c.x>a.x)
        return -1*A;

    return A;
}

void arc_direction(vector<Point> line){

    int mid = line.size()/2;
    Point A = line.front();
    Point O = line[mid];
    Point B = line.back();

    c = (A.x-O.x)*(B.y-O.y)-(A.y-O.y)*(B.x-O.x);

    if(c==0)
        cout<<"Up"<<endl;
    if(c>0)
        cout<<"right"<<endl;
    if(c<0)
        cout<<"left"<<endl;


}


void mask_generator(){

    mask = Mat::zeros(gray.size(), CV_8U);
    for (auto it:selected_points) {

        roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
        roi = Scalar(255, 255, 255);
    }
    imshow("Mask", mask);
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

/// Function for calculating derivative

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

    mask_generator();

    sensor_msgs::ImagePtr mask_image;
    mask_image= cv_bridge::CvImage(std_msgs::Header(),"mono8",mask).toImageMsg();
    mask_pub.publish(mask_image);

    features_found = true;
    needToInit=false;


}


void find_features_1(Mat colr ,Mat gr)
{

    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    for (int it=n+2;it<points[0].size();it++) {



        // The mask that we created using rectangles is used to created a ROI and features are extracted in that ROI.
        // I dont create a single mask using all the points because it ends up my algorithm with just a single feature
        // Instead I use one rectangle at a time and iterate through all rectangles to find features. This also allows me
        // to avoid skipping and ROI and find greater number of points.
        mask = Mat::zeros(gray.size(), CV_8U);
        roi = Mat(mask, Rect(points[0][it].x - width / 2, points[0][it].y - height / 2, width, height));
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


void find_features_2(Mat colr ,Mat gr)
{

    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    int d=3;
    int m=n;

    n+=points[1].size()-n;

    for (int it=0;it<3;it++) {



        // The mask that we created using rectangles is used to created a ROI and features are extracted in that ROI.
        // I dont create a single mask using all the points because it ends up my algorithm with just a single feature
        // Instead I use one rectangle at a time and iterate through all rectangles to find features. This also allows me
        // to avoid skipping and ROI and find greater number of points.
        mask = Mat::zeros(gray.size(), CV_8U);
        roi = Mat(mask, Rect((points[0][m].x+d) - width / 2,points[0][m].y - height / 2, width+4, height+4));
        roi = Scalar(255, 255, 255);


        // finding features
        goodFeaturesToTrack(gray, point_buffer, 2, 0.01, 1, mask, 3, 5, 0, 0.04);


        // its used to refine corner search and get even better approximate but somehow it doesnt work in my case.

//                cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);


        for (auto index:point_buffer) {

            points[1].push_back(index);

        }

        d+=5;

    }
    n+=points[1].size()-1;


    // This is for drawing mask. ( Not needed just shashka)
//
//    mask = Mat::zeros(gray.size(), CV_8U);
//    for (auto it:selected_points) {
//
//        roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
//        roi = Scalar(255, 255, 255);
//    }
//    imshow("Mask", mask);


    features_found = true;
    needToInit=false;


}



void Image_Initialization()
{

    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);
    if (nightMode)
        image = Scalar::all(0);
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


void acquire_pose(const geometry_msgs::PoseStampedConstPtr& pose)
{
    tf::Quaternion quater;
    tf::quaternionMsgToTF(pose->pose.orientation,quater);
    tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
    yaw_degrees=-1*angles::to_degrees(pitch);
}


void Draw_flowVectors(vector<Point2f> prev_pts, vector<Point2f> next_pts)
{

    right_sum=0;
    left_sum=0;
    up_sum=0;
    down_sum=0;

    for(int i=0;i<next_pts.size();i++)
    {
        CvPoint p,q;

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

    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);



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




/// This is the visual servoing loop that calculates optical flow and performs the task of following the path.

void visual_servo() {


    if(decision==5 && n==2) {
        case_5_up_1=true;
        case_1_2_3_4=true;

    }

    if(decision==5 && n>2) {
        case_5_up_1 = false;
        case_1_2_3_4=false;

    }
    if(decision==5 && n == points[0].size()-1){
        case_5_up_2=true;
        case_1_2_3_4=true;
    }

    if (n == points[0].size() - 1) {

        case_1_2_3_4 = true;
    }

    if((decision ==1 || decision==3) && case_1_2_3_4)
    {

        Image_Initialization();

        v.linear.x=0.0;
        v.linear.y=0.0;
        v.angular.z=50;
        vel_pub.publish(v);

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);
        cout<<"In turn left \n";
        char c= (char)waitKey(20);
        switch(c){

            case 'c':
                case_1_2_3_4=false;

            case 'n':
                n++;
        }




    }

    if((decision ==2 || decision==4) && case_1_2_3_4)
    {

        Image_Initialization();

        v.linear.x=0.0;
        v.linear.y=0.0;
        v.angular.z=-25;
        vel_pub.publish(v);

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);
        cout<<"In turn right \n";
        char c= (char)waitKey(20);
        switch(c){

            case 'c':
                case_1_2_3_4=false;

            case 'n':
                n++;
        }
    }

    if(decision ==5 && case_5_up_1)
    {

        Image_Initialization();

        v.linear.x=0.0;
        v.linear.y=0.0;
        v.angular.y=-60;
        v.angular.z=0;
        v.angular.x=0;
        vel_pub.publish(v);

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);
        cout<<"In turn up_1 \n";
        char c= (char)waitKey(20);
        switch(c){

            case 'c':
                case_1_2_3_4=false;
                case_5_up_1=false;
                clear_all();


            case 'n':
                n++;
        }


    }

    if(decision ==5 && case_5_up_2)
    {

        Image_Initialization();

        v.linear.x=0.05;
        v.linear.y=0.0;
        v.angular.y=0;
        v.angular.z=0;
        v.angular.x=0;
        vel_pub.publish(v);

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);
        cout<<"In turn up_2 \n";
        char c= (char)waitKey(20);
        switch(c){

            case 'c':
                case_1_2_3_4=false;
                case_5_up_2=false;
                clear_all();



            case 'n':
                n++;
        }

    }




    if (!obstacle && !case_1_2_3_4) {


       Image_Initialization();

     //  cout<<"In visual servo \n";

        ////////// Tracker Part/////////////

        // update the tracking result
        tracker->update(image,roii);
        // draw the tracked object
        cv::rectangle(image, roii, Scalar( 255, 0, 0 ), 2, 1 );
        // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        // show image with the tracked object
        //  imshow("tracker",image);

        ///////////////////////////////////




        if (needToInit) {

            find_features(frame, gray);

        } else if (!points[0].empty()) {

            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);

            calcOpticalFlowPyrLK(prevGray,
                                 gray,
                                 points[0],
                                 points[1],
                                 status,
                                 err,
                                 winSize,
                                 3,
                                 termcrit,
                                 0,
                                 0.001);


            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++) {

                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);


        }
        needToInit = false;

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);



         // The idea was to update the mask as the robot moves by removing the points that have been followed and consider
         // only future points. Then some interpolation could have been used to account for gaps that are created as the
         // robot travels

        //// Dynamic Mask Routine /////

//        mask = Mat::zeros(gray.size(), CV_8U);
//        for (int it=n+2;it<points[0].size();it++) {
//
//            roi = Mat(mask, Rect(points[0][it].x - 2 / 2, points[0][it].y - 2 / 2, width, height));
//            roi = Scalar(255, 255, 255);
//        }
//        imshow("Mask", mask);




//        if (n == points[0].size()/2){
//            ROI=roii;
//        }


        // Now I am activating obstacle routing when only three points are left in points buffer
        // its better to add a trigger than can be manually controlled ny user to toggle it on or off
//        if (n == points[0].size() - 1) {
//
//            obstacle=true;
//            needToInit=true;
//            points[0].clear();
//            points[1].clear();
//
//        } else
//            obstacle=false;



        ////////////////Checking Discontinuity in Points //////////////////

 //if(distance(points[0][n+1],points[0][n])>50)




//        if (n == points[0].size() - 2)
//    {
//
//        case_1_2_3_4=true;
       // points[0][n].y-=5;             // turning radius ( tukka :p)
       // cout<<"Discontinuity_left\n";



//      if(c>0)
//          v.angular.z=-3;
//      if(c<0)
//          v.angular.z=3;



       //  while(1)
//         {
//             char c = (char) waitKey(20);
//
//             if (c == 27)
//                // break;
//
//             vel_pub.publish(v);
//         }
//
//    }

//        if(distance(points[0][n+1],points[0][n])>10 && distance(points[0][n+1],points[0][n]) < 20)
//        {
//            points[0][n].x-=10;
//            cout<<"Discontinuity_right\n";
//
//            v.linear.x=0;
//            v.linear.y=0;
//            v.angular.z=0;
//            vel_pub.publish(v);
//
//            while(1)
//            {
//                char c = (char) waitKey(20);
//
//                if (c == 27)
//                    break;
//
//                vel_pub.publish(v);
//            }
//
//        }


        if(!points[0].empty()) {


            float desired_angle;
            float current_angle;

            circle(image, points[0][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

//            current_angle = atan2(points[0][n].y - desired_point.y, -points[0][n].x + desired_point.x);
//            current_angle=(current_angle*180)/M_PI + 90;
//            if (current_angle < -1.5)
//                current_angle = current_angle + 3.14;
//            else if (current_angle > 1.5)
//                current_angle = current_angle - 3.14;


//            Point2f P2;
//
//            P2.x =  (int)round(desired_point.x + 150* cos(current_angle * CV_PI / 180.0));
//            P2.y =  (int)round(desired_point.y + 150 * sin(current_angle * CV_PI / 180.0));

            arrowedLine( image, desired_point,Point_<double>(desired_point.x,desired_point.y-50) , Scalar(255,255,255), 1, CV_AA, 0 );
            arrowedLine( image, desired_point,points[0][n+1] , Scalar(255,255,255), 1, CV_AA, 0 );

            double angle = angle_bw_lines(desired_point,Point_<double>(desired_point.x,desired_point.y+50),points[0][n+1]);

          //  cout<<angle<<endl;
            cout<<yaw_degrees<<endl;



            //  v.linear.y =( (-points[0][n].x + desired_point.x)*cos(pitch ) + (-points[0][n].y + desired_point.y)*sin(pitch)) /1500;       // X direction for unity
            //  v.linear.x = (-(-points[0][n].x + desired_point.x)*sin(pitch)+ (-points[0][n].y + desired_point.y)*cos(pitch)) /1800;       // Z direction for unity




//            if (current_angle != 0)
//                v.angular.z =(2* (current_angle/abs(current_angle)));


//            errror= desired_point.x - points[0][n].x;


//            cout<<"Error: "<<errror<<endl;
//            v.angular.z = (0.5*errror);



                v.angular.z=angle + yaw_degrees;
                v.linear.y = (-points[0][n].x + desired_point.x) / 1500;       // X direction for unity
                v.linear.x = (-points[0][n].y + desired_point.y) / 1800;       // Z direction for unity

                vel_pub.publish(v);












        }

        // Drawing desired point on image
        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);

        //   robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

//        cout << "n: " << n << endl;
        first_run = false;


        if (distance(desired_point, points[0][n]) < 0.5) {

            cout<<points[0][n]<<endl;
            n++;
            if (n == points[0].size())
                waitKey();


        }


        char c = (char) waitKey(20);
        if (c == 27)
            return;
        switch (c) {


            case 's': {
                imwrite("/home/kari/surf_test1.jpg", gray);
                imwrite("/home/kari/surf_test_mask.jpg", mask);
                cout << "Image 1 saved \n";
                break;

            }

            case 'd': {
                imwrite("/home/surf_test2.jpg", gray);
                cout << "Image 2 saved \n";
                break;

            }

            case 'r':
                needToInit = true;

                pp.clear();
                e = 0;
                cout << "Re Initialized \n";
                break;

            case 'c':
               clear_all();

                // Making sure that the velocity sent to robot is zero. It may look useless here but when we are in
                // later cycles of execution and clear points and come back to this thread then v variable will have
                // velocity value of that thread (non zero) and will be continuously sent to the robot which is not
                // what we want as during this stage of execution robot should be stationary so we are enforcing our
                // static robot state here.

                v.linear.y = 0;
                v.linear.x = 0;
                v.angular.z= 0;
                break;
            case 'o':
                obstacle=true;
                needToInit=true;
                break;
            case 'n':
                n++;
                test+=5;

        }
    }

//    if(obstacle){
//
//
//        frame.copyTo(image);
//        cvtColor(image, gray, COLOR_BGR2GRAY);
//        ROI=Rect(200,1,250,300);
//        //    ROI=roii;
//        Mat mask;
//        Mat mask_image;
//        mask = Mat::zeros(gray.size(), CV_8U);
//        mask_image=Mat(mask,ROI);
////        mask_image=Mat(mask,roii);
//        mask_image=Scalar(255,255,255);
//        imshow("tracker",image);
//
//
//        ///////// Updating Desired point //////////
//        //  desired_point.x= -(w/2)*cos(yaw-M_PI/2)+w/2;                                //
//        //  desired_point.y= (-h/2)*sin(yaw-M_PI/2)+h/2;
//
//        //////////////////////////////////////////
//
//
//
//        if( nightMode )
//            image = Scalar::all(0);
//
//        if( needToInit )
//        {
//            // automatic initialization
//            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
//            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
//            addRemovePt = false;
//        }
//        else if( !points[0].empty() )
//        {
//
//            vector<float> err;
//            if(prevGray.empty())
//                gray.copyTo(prevGray);
//            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                                 3, termcrit, 0, 0.001);
//
//
//            if(!points[0].empty() && !points[1].empty())
//            {
//                Draw_flowVectors(points[0],points[1]);
//                // calculate_FOE(points[0],points[1]);
//            }
//
//            size_t i, k;
//            for( i = k = 0; i < points[1].size(); i++ )
//            {
//                if( addRemovePt )
//                {
//                    if( norm(point - points[1][i]) <= 10 )
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
//            return;
//        switch( c )
//        {
//            case 'r':
//                needToInit = true;
//                break;
//            case 'c':
//                points[0].clear();
//                points[1].clear();
//                obstacle=false;
//                break;
//            case 'n':
//                nightMode = !nightMode;
//                break;
//        }
//
//
//
//        std::swap(points[1], points[0]);
//        cv::swap(prevGray, gray);
//
//        velocity.linear.x=0.01;
//
//        if(right_sum>left_sum && (abs(right_sum-left_sum)>50))
//        {
//
//            //  velocity.linear.x=0.01;
//            velocity.angular.z=25;
//            vel_pub.publish(velocity);
//        }
//
//        else if(left_sum>right_sum && (abs(right_sum-left_sum)>50))
//        {
//
//
//            // velocity.linear.x=0.01;
//            velocity.angular.z=-25;
//            vel_pub.publish(velocity);
//        }
//
//        else
//        {
//            velocity.linear.y=0;
//            velocity.angular.z=0;
//            //   velocity.linear.x=1;
//            vel_pub.publish(velocity);
//        }
//
//    }



}




void Draw_Features()
{
    if (features_found) {
        for (auto q:points[1]) {
            circle(image, q, 3, Scalar(0, 255, 0), -1, 8);
            imshow("LK Demo", image);

        }

    }
}

/// This function tries to find features on the drawn path

void path_drawn_features_nishta()
{


    Image_Initialization();


    // needToInit variable decides if user wants to reinitialize the features. In the first run it will always be true

    if (needToInit) {

        find_features(frame,gray);
    }


    // Drawing features that were found

     Draw_Features();



    ///////// Updating Desired point //////////

    desired_point.x= -(w/2)*cos(yaw-M_PI/2)+w/2;                                //
    desired_point.y= (-h/2)*sin(yaw-M_PI/2)+h/2;

    //////////////////////////////////////////

  //  cout<<"desired_point "<<desired_point<<"  Yaw: "<<yaw_degrees<<endl;

    // Drawing reference point
    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);


    // Drawing indicator circle that shows which point is being followed. Variable n decides that and is updated in button press by the user
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
//            points[0].clear();
//            points[1].clear();
//            line_points.clear();
//            line_points_circle.clear();
//            selected_points.clear();
//            features_found = false;
//            cout << "Points Cleared \n";
//            path_drawn= false;
//            n=0;
//            feature_on_path_found=false;
//            discontinuity=false;
//            first_run=true;
            clear_all();


            break;
        case 'n':
            //  nightMode = !nightMode;
            n++;
            break;
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
}


void select_tracker_ROI()
{



    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);
    if (nightMode)
        image = Scalar::all(0);

    // Tracker Part

    roii=selectROI("tracker",image);
    if(roii.width==0 || roii.height==0)
        return ;
    tracker->init(image,roii);
    printf("Start the tracking process, press ESC to quit.\n");
    tracker_ROI=true;



}


/// Function that takes care of drawing path on the incoming image and then transfers control to the part

void path_not_yet_drawn()
{
   Image_Initialization();

    // Drawing circles to show the path user drew

    for (auto index:line_points) {

        circle(image, Point(index), 5, CV_RGB(255, 255, 0), 1, 8, 0);
    }

    // Drawing rectangles around the path. Kind of redundant here but who cares its my code :p

    for (auto g:selected_points) {
        rectangle(image, Point(g.x - width / 2, g.y - height / 2), Point(g.x + width / 2, g.y + height / 2),
                  CV_RGB(255, 0, 0), 1, 8, 0);

    }

    mask_generator();

    char c = (char) waitKey(10);
    if (c == 27)
    {
        path_drawn=true;

    }

    switch (c) {
        case 'c':

            ostringstream filename;
            filename<<"/home/kari/catkin_ws/tensorflow-image-classifier/Training_images/lup_"<<picture_counter++<<".jpg";
            // Only use for creating training set

          //  cout<<filename.str()<<endl;c

            imwrite( filename.str(), mask );
            line_points.clear();
            points[0].clear();
            points[1].clear();
            selected_points.clear();
            path_drawn= false;
            discontinuity=false;
            first_run=true;
            tracker_ROI=false;
            v.linear.x=0;


            break;
    }


    // This is the desired point or reference point that shows position of the robot in image
    // and tries to follow the points we draw on screen

    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);
    imshow("LK Demo", image);

    vel_pub.publish(v);
}



//////////////////////////////////////////////////////////////////////
// Relation for calculating polar to rectangular screen coordinates//
// in order to shift desired point                                //
//                                                               //
// x= (w/2)*cos(theta-pi/2)+w/2;                                //
// y= (-h/2)*sin(theta-pi/2)+h/2;                              //
//                                                            //
// w= screen width ; h = screen height ;                     //
// theta = angle in radians from unity                      //
/////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Relation for Angular compensation i.e to make robot achieve desired pose as drawn by the user.       //
// Calculates angular velocity                                                                         //
//                                                                                                    //
// Velocity is calculated for Z because Z in ROS is Y in Unity                                       //
//                                                                                                  //
// angular velocity.z = (desired_point.x- point.x)*alpha + (desired_point.y - point.y)*beta        //
//                                                                                                //
// desired_point = point on screen which will be followed                                        //
// point = features that are being servoed                                                      //
// alpha = binary number (o or 1)  =>    { alpha =1   DP.y == max || DP.y == min               //
//                                         alpha =0   min < DP.y < max       }                //
//                                                                                           //
// beta = binary number (0 or 1)         { beta =1   DP.x == max || DP.x == min             //
//                                         beta =0   min < DP.x < max       }              //
////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////
//                                                                  //
// Decision == 1  => approach from left and turn left              //
// Decision == 2  => approach from left and turn right            //
// Decision == 3  => approach from left and turn left            //
// Decision == 4  => approach from left and turn right          //
//                                                             //
///////////////////////////////////////////////////////////////



#endif //PROJECT_PIONEER_SIMPLE_APPROACH_H
