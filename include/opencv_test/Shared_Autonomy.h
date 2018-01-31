//
// Created by kari on 18. 1. 31.
//

#ifndef PROJECT_SHARED_AUTONOMY_H
#define PROJECT_SHARED_AUTONOMY_H

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

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

class Shared_Autonomy_Vine{
public:

    /***********************************************
                       Public Functions
    ***********************************************/

    /**
     * Constructor for basic initializations
     */

    Shared_Autonomy_Vine();

    /**
     * Function for selecting points in order to draw squares/ ROI for creating
     * mask that will be used to extract features
     */

     static void points_selector(vector<Point> &all_points);

    /**
     * Mouse callback function
     */

     static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ );

    /**
     * Function for calculating transformation matrix
     */

     static Mat transformation_calculate(Point2f x, Point2f y);

    /**
     * Function for calculating distance between two points
     */

     static double distance(Point a,Point b);

    /**
     * ROS callback function for images coming from camera / unity3D
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);



    /***********************************************
                    Public Variables
    ***********************************************/




    static int MAX_COUNT ;
    static int line_point_size;
    static int n;
    static int width;
    static int height;
    static int m;
    static int circle_radius;
    static int e;

    static bool needToInit;
    static bool nightMode;
    static bool features_found;
    static bool addRemovePt;
    static bool feature_selected;
    static bool first_run;

    static bool path_drawn;
    static bool feature_on_path_found;
    static bool path_feature_found;
    static bool drag;

    static Mat gray, prevGray, image, frame;
    static Mat mask;
    static Mat roi;
    static Mat transformation_bw_goal_nf;




    static double th_prev;
    static double t_y;
    static double camera_height;
    static double camera_pitch;


    static vector<Point2f> points[2];
    static vector<Point2f> point_buffer;
    static vector<Point2f> pp;
    static vector<cv::Mat> transformation_bw_line;
    static vector<Point> line_points;
    static vector<Point> line_points_temp;
    static vector<Point> line_points_world;
    static vector<Point> transformed_points;
    static vector<Point> line_points_circle;
    static vector<Point> selected_points;

    static Point2f point;
    static Point2f point1;
    static Point2f desired_point;
    static Point2f previous,current;
    static Point2f nearest_feature;


    static Point Target_point;
    static Point Target_point_prev;
    static Point p;



    ros::NodeHandle nh;
    ros::Publisher vel_pub;

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    static geometry_msgs::Twist v;



};

enum path_feature_state{
    PATH_DRAWING, FEATURE_FINDING,VISUAL_SERVOING
};
path_feature_state pstate;

enum strategy_state{
    GLOBAL_FEATURES,FEATURES_ON_LINE,RELATIVE_TRANSFORMATION
};
strategy_state sstate;

#endif //PROJECT_SHARED_AUTONOMY_H
