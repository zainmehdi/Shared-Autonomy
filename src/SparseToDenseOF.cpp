
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
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
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);
Mat gray, prevGray, image, frame,img;
vector<Point2f> points[3];

const int MAX_COUNT = 500;
bool needToInit = false;
bool nightMode = false;
bool drag=false;
vector<Point> line_points;
vector<Point> line_points_circle;
vector<Point> selected_points;
int width =50;
int height=50;
bool first_run=false;
int circle_radius=2;
Point p;


void draw_circle(Mat &imgg,vector<Point> &lp){
    for(auto index:lp)
    {
        circle(imgg,Point(index), circle_radius, CV_RGB(255, 0, 0),1,8,0);
    }

}

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
        if(distance>50)
        {

            selected_points.push_back(i);
            p=i;
        }

    }
    line_points.clear();

}


static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{

    if  ( event == EVENT_LBUTTONDOWN && !drag)
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
        drag=true;
    }

    else if( event == EVENT_MOUSEMOVE && drag )
    {

        line_points.push_back(CvPoint(x,y));
        line_points_circle.push_back(CvPoint(x,y));
        draw_circle(img,line_points_circle);
        points_selector(line_points);


        for(auto g:selected_points)
        {
            rectangle(img,Point(g.x- width/2,g.y-height/2),Point(g.x+width/2,g.y+height/2),CV_RGB(255, 0, 0),1,8,0);

        }

        imshow("My Window",img);
        waitKey(1);
    }

    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }
}


void imagecb(const sensor_msgs::ImageConstPtr& msg)
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
    cv_ptr->image.copyTo(img);


    if( frame.empty() )
        return;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    if( nightMode )
        image = Scalar::all(0);

    if( needToInit )
    {
        // automatic initialization

       for(auto it:selected_points) {

           Mat mask=Mat::zeros(gray.size(),CV_8U);
           cout<<"Point1 Size: "<<points[1].size();
           Mat roi=Mat(mask,Rect(it.x-width/2,it.y-height/2,width,height));
           roi= Scalar(255, 255, 255);
           goodFeaturesToTrack(gray, points[3], MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
           cornerSubPix(gray, points[3], subPixWinSize, Size(-1, -1), termcrit);
           points[1].insert(std::end(points[1]),std::begin(points[3]),std::end(points[3]));


       }


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
                if( norm(point - points[1][i]) <= 2 )
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
            cout<<"Re initialized \n";
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            selected_points.clear();
            line_points.clear();
            line_points_circle.clear();
            cout<<"All points cleared \n";
            break;
        case 'n':
            nightMode = !nightMode;
            break;
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
}

int main( int argc, char** argv )
{

    ros::init(argc, argv, "local_autonomyy");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    image_sub = it.subscribe("/usb_cam/image_raw", 1,
                             imagecb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );

    help();




    ros::spin();

    return 0;
}