////
//// Created by kari on 18. 6. 4.
////
//
///* This program tries to use optical flow to compensate command mapping when camera is rotating
// * The concept is to use optical flow to track if the mapping has changed or not.
// *
// * Normal / Starting Conditions :
// * We move left => Flow directed right
// * We move right => Flow directed left
// *
// * Check for:
// * We move right => flow directed right
// * We move left => flow directed left
// *
// * Above conditions if true imply that left and right have been swapped
// * In other words camera has rotated 180 degrees and command compensation
// * needs to be carried out.
// *
// * I will use the node I created for avoiding obstacle and compensate for mappings
// * ROS Sharp version will be used to communicate with Unity node.
// *
// * Cases : Camera rotates body stationary
// *         Body rotates camera stationary (h//as been compensated)
// *         Both rotate i.e relative motion between two
// *
//*/
//
// Also a very good video stabilizer for live videos but the draw back is it needs some reference/ reinitialization
//
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//#include <iostream>
//#include "ctime"
//#include "chrono"
//#include "ros/ros.h"
//#include <eigen3/Eigen/Dense>
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//
//#include <iostream>
//#include <ctype.h>
//#include "ros/ros.h"
//#include "image_transport/image_transport.h"
//#include "cv_bridge/cv_bridge.h"
//#include <geometry_msgs/Twist.h>
//
//
//#include <iostream>
//#include <ctype.h>
//#include <std_msgs/String.h>
//
//
//using namespace cv;
//using namespace std;
//using namespace Eigen;
//namespace enc = sensor_msgs::image_encodings;
//
//
//
//Point2f point;
//bool addRemovePt = false;
//Mat img,src;
//Mat gray, prevGray, image, frame;
//vector<Point2f> points[2],v;
//TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//Size subPixWinSize(10,10), winSize(31,31);
//float left_sum,right_sum,up_sum,down_sum;
//vector<uchar> status;
//geometry_msgs::Twist velocity;
//ros::Publisher map_pub;
//std_msgs::String mapping_rcv,mapping_snd;
//
//
//const int MAX_COUNT = 100;
//bool needToInit = false;
//bool nightMode = false;
//
//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);
//        addRemovePt = true;
//    }
//}
//
//
//// Function to calculate Focus of Expansion.
//// Helpful resources
//// https://www.dgp.toronto.edu/~donovan/stabilization/opticalflow.pdf
//// https://stackoverflow.com/questions/18245076/optical-flow-and-focus-of-expansion-in-opencv?rq=1
//
//void calculate_FOE(vector<Point2f> prev_pts, vector<Point2f> next_pts)
//{
//
//    MatrixXf A(next_pts.size(),2);
//    MatrixXf b(next_pts.size(),1);
//    Point2f tmp;
//
//    for(int i=0;i<next_pts.size();i++)
//    {
//
//        if(!status[i])
//            continue;
//        tmp= next_pts[i]-prev_pts[i];
//        A.row(i)<<next_pts[i].x-prev_pts[i].x,next_pts[i].y-prev_pts[i].y;
//        b.row(i)<<(prev_pts[i].x*tmp.x)-(prev_pts[i].y*tmp.y);
//
//    }
//
//
//    Matrix<float,2,1> FOE;
//    FOE=((A.transpose()*A).inverse())*A.transpose()*b;
//
//    Point2f c;
//    c.x=FOE(0,0);
//    c.y=FOE(1,0);
//    circle( image, c, 8, Scalar(255,255,0), -1, 8);
//
//
//}
//
//// Function to draw flow vector and calculate magnitude
//
//void Draw_flowVectors(vector<Point2f> prev_pts, vector<Point2f> next_pts)
//{
//
//    right_sum=0;
//    left_sum=0;
//    up_sum=0;
//    down_sum=0;
//
//    for(int i=0;i<next_pts.size();i++)
//    {
//        CvPoint p,q;
//
//        if(!status[i])
//            continue;
//
//        p.x = (int) prev_pts[i].x;
//        p.y = (int) prev_pts[i].y;
//        q.x = (int) next_pts[i].x;
//        q.y = (int) next_pts[i].y;
//        double angle;
//        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
//        double hypotenuse;  hypotenuse = sqrt( pow(p.y - q.y,2) + pow(p.x - q.x,2 ));
//
//        // Scaling of arrow vectors
//        q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
//        q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
//        arrowedLine( image, p, q, Scalar(255,255,255), 1, CV_AA, 0 );
//
//
//
//        // Calculating magnitude
//
//        int mag= sqrt(pow(next_pts[i].x-prev_pts[i].x,2)+pow(next_pts[i].y-prev_pts[i].y,2));
//
//        // Flow vectors are categorized according to there position in image
//        // We can choose horizontal and vertical boundary as per our choice
//
//        if(next_pts[i].x > 335)
//            right_sum+=mag;
//        else if(next_pts[i].x < 295)
//            left_sum+=mag;
//
//        if(next_pts[i].y <329)
//            up_sum+=mag;
//        else
//            down_sum+=mag;
//
//
//
//    }
//
////    cout<<"Left Flow "<<left_sum<<" Right Flow "<<right_sum
////        <<" Up Flow "<<up_sum<<" Down Flow "<<down_sum<<endl;
////    cout<<".........................................."<<endl;
//
//
//}
//
//
//
//
//
//
//void imageCb(const sensor_msgs::ImageConstPtr& msg)
//{
//
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
//    }
//    catch (cv_bridge::Exception &e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//
//    frame=cv_ptr->image;
//
//
//
//    if( frame.empty() )
//        return;
//
//    frame.copyTo(image);
//    cvtColor(image, gray, COLOR_BGR2GRAY);
//
//    Rect ROI=Rect(300,1,20,475);
//    Mat mask;
//    Mat mask_image;
//    mask = Mat::zeros(gray.size(), CV_8U);
//    mask_image=Mat(mask,ROI);
//    mask_image=Scalar(255,255,255);
//    if( nightMode )
//        image = Scalar::all(0);
//
//    if( needToInit )
//    {
//        // automatic initialization
//        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
//        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
//        addRemovePt = false;
//    }
//    else if( !points[0].empty() )
//    {
//
//        vector<float> err;
//        if(prevGray.empty())
//            gray.copyTo(prevGray);
//        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                             3, termcrit, 0, 0.001);
//
//
//        if(!points[0].empty() && !points[1].empty())
//        {
//            Draw_flowVectors(points[0],points[1]);
//            // calculate_FOE(points[0],points[1]);       // wasnt working properly so omitted it
//        }
//
//        size_t i, k;
//        for( i = k = 0; i < points[1].size(); i++ )
//        {
//            if( addRemovePt )
//            {
//                if( norm(point - points[1][i]) <= 10 )
//                {
//                    addRemovePt = false;
//                    continue;
//                }
//            }
//
//            if( !status[i] )
//                continue;
//
//            points[1][k++] = points[1][i];
//            circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//        }
//        points[1].resize(k);
//    }
//
//    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//    {
//        vector<Point2f> tmp;
//        tmp.push_back(point);
//        cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
//        points[1].push_back(tmp[0]);
//        addRemovePt = false;
//    }
//
//    needToInit = false;
//    imshow("Obstacle_Avoidance", image);
//
//    char c = (char)waitKey(10);
//    if( c == 27 )
//        return;
//    switch( c )
//    {
//        case 'r':
//            needToInit = true;
//            break;
//        case 'c':
//            points[0].clear();
//            points[1].clear();
//            break;
//        case 'n':
//            nightMode = !nightMode;
//            break;
//    }
//
//
//
//    std::swap(points[1], points[0]);
//    cv::swap(prevGray, gray);
//
//
//    // Here we decide when and in which direction to avoid obstacle
//    // I use a specific magnitude threshold which when exceeded
//    // triggers velocity commands to be published to Unity or actual robot
//    // Have to tweak this values to make it work depends on how fast robot
//    // is moving. The faster the robot higher will b the mag value
//    // Todo: create a launch file and parameters for all these
//
//
//   if((mapping_rcv.data=="left" && left_sum>right_sum && abs(right_sum-left_sum)>60) || (mapping_rcv.data =="right"&& right_sum>left_sum && abs(right_sum-left_sum)>60))
//   {
//       mapping_snd.data="swap";
//       map_pub.publish(mapping_snd);
//
//   } else
//   {
//       mapping_snd.data="";
//       map_pub.publish(mapping_snd);
//   }
//
////    cout<<"left_sum : "<<left_sum<<"  ";
////    cout<<"right_sum: "<<right_sum<<endl;r
//
//
//
//}
//
//
//void mapping_cb(std_msgs::StringConstPtr msg)
//{
//    mapping_rcv.data=msg->data;
//   // cout<<mapping_rcv.data<<endl;
//}
//
//int main( int argc, char** argv ) {
//
//    ros::init(argc, argv, "Command_Compensation");
//    ros::NodeHandle nh;
//    ros::Subscriber mapping_sub;
//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber image_sub;
//    image_transport::Publisher image_pub;
//    image_sub = it.subscribe("image_raw", 1, imageCb);
//    mapping_sub =nh.subscribe("/command_status",1,mapping_cb);
//    map_pub = nh.advertise<std_msgs::String>("/mapping", 1000);
//
//    ros::spin();
//
//    return 0;
//}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


/*
 * Focal_length = ImageSizeX /(2 * tan(CameraFOV * π / 360))
 * Center_X = ImageSizeX / 2
 * Center_Y = ImageSizeY / 2
 *
 * Intrinsic Camera Matrix K
 *
 * K = [[f, 0, Cu],
       [0, f, Cv],
       [0, 0, 1 ]]
 *
 * Useful links
 * https://dsp.stackexchange.com/questions/25936/obtain-motion-between-image-features-by-means-of-the-homography-matrix/25939?noredirect=1#comment48551_25939
 *
 * https://github.com/carla-simulator/carla/issues/56
 *
 * This link describes very well how we solve for homography
 * https://stackoverflow.com/questions/23619269/calculating-translation-value-and-rotation-angle-of-a-rotated-2d-image
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>


using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;
namespace enc = sensor_msgs::image_encodings;
int average_counter=0;

bool first_run=true;
int n=0;

Mat descriptors_image1, descriptors_image2, intrinsic;
std::vector<KeyPoint> keypoints_image1, keypoints_image2;


Mat frame,image,image1,image2,gray,test;
Vec3f euler;
float angle_curr,angle_prev,threshold;
float reinitialize_angle=0;
int minHessian = 200;

//SURF Detector
Ptr<SURF> detector= SURF::create(minHessian);

// Fast feature extractor
//Ptr<FastFeatureDetector> detector= FastFeatureDetector::create();


// BRIEF descriptor
//Ptr<BriefDescriptorExtractor> extractor= BriefDescriptorExtractor::create();

// image1= First image in grayscale
// image2= Second image in grayscale

ImageFeatures aFeatures, bFeatures;


bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).

Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    if(isRotationMatrix(R)) {


        float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

        bool singular = sy < 1e-6; // If

        float x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            y = atan2(-R.at<double>(2, 0), sy);
            z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        } else {
            x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
            y = atan2(-R.at<double>(2, 0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
    }


}

// This function applies rotation to the image

Mat rotate(Mat src, double angle)
{
    Mat dst;
    Point2f pt(src.cols/2., src.rows/2.);
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}



void image_initialization(const sensor_msgs::ImageConstPtr &im)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(im, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;
    frame.copyTo(image);

    if( frame.empty() )
        return;

    cvtColor(image, image1, COLOR_BGR2GRAY);
//    imshow("check",image1);


}

double find_average_vector(std::vector<Vec3f> val)
{
   double avg;
    for(auto i:val)
    {
        avg+=i[2];
    }

    avg=avg/val.size();
    return avg;
}


void image_cb(const sensor_msgs::ImageConstPtr &im) {

    image_initialization(im);

    if(first_run){


//        image2 = imread("/home/kari/template_image_compensation.jpg", CV_LOAD_IMAGE_GRAYSCALE);
//        detector->detect(image2, keypoints_image2);
//        detector->compute(image2, keypoints_image2, descriptors_image2);
//        imshow("check",image2);
//        waitKey(1);



         image2=image1;

        detector->detect(image2, keypoints_image2);
        detector->compute(image2, keypoints_image2, descriptors_image2);

    }

    detector->detect(image1,keypoints_image1);
    detector->compute(image1, keypoints_image1, descriptors_image1);

    if (first_run == false) {


         // Need to use it with binary descriptors
         // FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));


        FlannBasedMatcher matcher;
        std::vector<std::vector<DMatch>> matches;
        descriptors_image1.convertTo(descriptors_image1, CV_32F);
        descriptors_image2.convertTo(descriptors_image2, CV_32F);
        matcher.knnMatch(descriptors_image1, descriptors_image2, matches,2);

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches[i].size() > 1 && matches[i][0].distance / matches[i][1].distance <= ratio_thresh)
            {
                good_matches.push_back(matches[i][0]);
            }
        }



        Mat img_matches;
        drawMatches(image1, keypoints_image1, image2, keypoints_image2,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        imshow("Good Matches", img_matches);

        //-- Localize the object
        std::vector<Point2f> one;   // this is new image keypoints
        std::vector<Point2f> two;   // this is old image keypoints

        for (int i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            one.push_back(keypoints_image1[good_matches[i].queryIdx].pt);
            two.push_back(keypoints_image2[good_matches[i].trainIdx].pt);
        }


        if (!one.empty() && !two.empty()) {
            Mat H = findHomography(one, two, CV_RANSAC);

            std::vector<Mat> Rt,R, t, n;
            int solutions = decomposeHomographyMat(H, intrinsic, R, t, n);

            euler=(rotationMatrixToEulerAngles(R[0]));
            angle_curr=(euler[2]*180/M_PI) +reinitialize_angle;

           // todo: need to implement code to extract correct solution out of four solutions

          //   std::cout << "Rotation from homography decomposition: " << angle_curr << std::endl;
          //  std::cout << "Translation from homography decomposition: " << t[1] << std::endl;




            if(angle_prev!=0 && abs(angle_curr)>(abs(angle_prev)-2) && abs(angle_curr)<(abs(angle_prev)+2))
            {
                Mat rot2 = getRotationMatrix2D(CvPoint2D32f(320,240),angle_curr,1);
                warpAffine(image1,test,rot2,image1.size(), INTER_CUBIC | WARP_INVERSE_MAP);
                imshow("Rectified Image",test);
                waitKey(1);
            }


            angle_prev=angle_curr;


        }


    }
//
    char c=(char)waitKey(1);
    if (c == 'f')

    if(!image1.empty())
    {
        image2 = image1;
        keypoints_image2 = keypoints_image1;
        descriptors_image2 = descriptors_image1;
        reinitialize_angle=angle_prev;

    }


        //   this is used when we use sequential images

//        image2 = image1;
//        keypoints_image2 = keypoints_image1;
//        descriptors_image2 = descriptors_image1;

        if (first_run)
            first_run = false;



}


int main( int argc, char** argv ) {

    ros::init(argc, argv, "Command_Compensation");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_sub = it.subscribe("image_raw", 1, image_cb);


    //Cameras intrinsic matrix created for unity's simulated camera'

    intrinsic.create(3,3,cv::DataType<double>::type);
    intrinsic.at<double>(0,0)=640/(2*tan(60*M_PI/360));
    intrinsic.at<double>(0,1)=0;
    intrinsic.at<double>(0,2)=640/2;
    intrinsic.at<double>(1,0)=0;
    intrinsic.at<double>(1,1)=640/(2*tan(60*M_PI/360));
    intrinsic.at<double>(1,2)=480/2;
    intrinsic.at<double>(2,0)=0;
    intrinsic.at<double>(2,1)=0;
    intrinsic.at<double>(2,2)=1;

    std::cout<<"intrinsic :"<<intrinsic<<std::endl;


    ros::spin();
    return  0;
}