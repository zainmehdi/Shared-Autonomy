//
// Created by zain on 17. 11. 30.
//

#ifndef PROJECT_FEATURE_EXTRACTOR_H
#define PROJECT_FEATURE_EXTRACTOR_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <iostream>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
namespace enc = sensor_msgs::image_encodings;

/**
 * Class that handles ROI and feature extraction
 */

class feature_extractor {
public:

    /***********************************************
                       Public Functions
    ***********************************************/

    /**
     * Constructor for the class
     * Start video acquisition from the camera and sets various parameters
     */

    feature_extractor();

    /**
     *    Function for mouse callback
     */

    static void CallBackFunc(int event, int x, int y, int flags, void* userdata);

    /**
     * Function for selecting specific points from point_lines
     */

    static void points_selector(vector<Point> &all_points);

    /**
     * Function for drawing circle at mouse pointer's position on screen
     */

    static void draw_circle(Mat &img,vector<Point> &lp);


    /*
     * Function for finding keypoints and descriptors using SURF
     */

    void find_Surf_features(Mat &im);

    /**
     * Creating a feature detector
     */

    static Ptr<FeatureDetector> Surf;

    /**
     * Subscriber function for image callback from PC on mobile robot
     */

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Function for finding Shi-Tomasi Goof features to track
     */

    void find_GoodFeatures(Mat &im);



    /***********************************************
                        Public Variables
     ***********************************************/

   /**
    * ROS related initializations
    */

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    /**
     * VideoCapture object for acquiring video
     */

    VideoCapture cap;

    /**
     * Mat element for storing image data
     */

    static Mat img,gray,prevGray,main_disp;
    Mat img_surf_final;
    Scalar color ;
    static bool start;

    /**
    * Vector for storing all the points of mouse pointer when dragging
    */

    static vector<Point> line_points;           // this will be cleared in each iteration
    static vector<Point> line_points_circle;    // it will be maintained to draw all circles

    /**
     * Vector for storing selected points which will be used for rectangles formation
     */

    static vector<Point> selected_points;

    /**
     * Vector for storing points for LK tracker
     */

    static vector<Point2f> points[2];
    static Point2f point;

    /**
     * Pointer to SURF detector
     */

    Ptr<SURF> Surf_detector;


    /**
     * Keypoints and descriptors for SURF detector
     */

    std::vector<std::vector<KeyPoint>> surf_keypoints;
    std::vector<Mat> surf_descriptor;
    std::vector<KeyPoint> concatenated_keypoints;



    /**
     * Mask and ROI variables used with SURF detector
     */

    Mat mask;
    Mat roi;


    /**
     * Variable for checking mouse drag
     */

    static bool drag;


    /**
     * Random variable for logic implementation
     */

    /// *** For SURF ***
    static bool first_run;
    static int circle_radius;
    static int width;             // width of rectangles drawn on screen
    static int height;            // width of rectangles drawn on screen
    static int minHessian;        // hessian value to be used in SURF detector
    static Point p;               // for storing last pivot point
    static bool SURF_selector;

    ///*** For LK Tracker ***
    const int MAX_COUNT = 500;
    bool needToInit = false;
    bool nightMode = false;
    static bool addRemovePt ;
    static bool LK_selector;
    int c;


};



#endif //PROJECT_FEATURE_EXTRACTOR_H
