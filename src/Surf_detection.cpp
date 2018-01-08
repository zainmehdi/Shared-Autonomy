////
//// Created by zain on 17. 11. 23.
////
//
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
\

using namespace cv::xfeatures2d;
//void readme();
///* @function main */
//
//
//int main( int argc, char** argv ) {
//
//
//  VideoCapture cap(0);
//  int minHessian = 400;
////  Ptr<SURF> detector = SURF::create(minHessian);
//    Ptr<FastFeatureDetector> fD= FastFeatureDetector::create();
//  RNG rng(12345);
//  Mat img_1;
//  Mat cropped;
//  Mat img_curr;
//  Mat img_prev;
//  Mat dest;
//  Mat flowMat;
//  Size win=Size(50,50);
//  std::vector<KeyPoint> keypoints_curr, keypoints_prev;
//  Mat descriptor_1, descriptor_2;
//  Point pt1_1;
//  Point pt2_1;
//  Point pt1_2;
//  Point pt2_2;
//  bool first_run=true;
//  std::vector<uchar> status;
//  std::vector<float> err;
//  std::vector<cv::Point2f> features_prev, features_next;
//
//  cap.read(img_curr);
//  if (!img_curr.data) {
//    std::cout << " --(!) Error reading images " << std::endl;
//    return -1;
//  }
//  Rect2d r = selectROI(img_curr);
//  Mat mask = Mat::zeros(img_curr.size(), CV_8U);
//  Mat roi(mask, r);
//  roi = Scalar(255, 255, 255);
//
////  detector->detectAndCompute(img_curr, mask, keypoints_curr, descriptor_1);
//
//  fD->detect(img_curr,keypoints_curr,mask);
//  //  fD->detectAndCompute(img_curr, mask, keypoints_curr, descriptor_1);
//  drawKeypoints(img_curr, keypoints_curr, img_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
//  rectangle(img_1, r, CV_RGB(255, 0, 0), 1, 8, 0);
//  imshow("SURF POINTS",img_1);
//  for(auto u:keypoints_curr){
//    features_next.push_back(u.pt);
//  }
//
//
//
//waitKey();
//  while(1) {
//    img_prev=img_curr.clone();
//    features_prev=features_next;
//
//
//    cap.read(img_curr);
//
//    if (!img_curr.data) {
//      std::cout << " --(!) Error reading images " << std::endl;
//      return -1;
//    }
//
//    calcOpticalFlowFarneback(img_prev, img_curr, flowMat, 0.4, 1, 12, 2, 8, 1.2, 0);
////    cv::calcOpticalFlowPyrLK(
////            img_prev, img_curr, // 2 consecutive images
////            features_prev, // input point positions in first im
////            features_next, // output point positions in the 2nd
////            status,    // tracking success
////            err,
////            win,
////            0);      // tracking errorq
//
//// track points based on dense optical flow field and bilinear interpolation
//    for( unsigned int n = 0; n <features_prev.size(); ++n )
//    {
//      float ix = floor(features_prev[n].x);
//      float iy = floor(features_prev[n].y);
//      float wx = features_prev[n].x - ix;
//      float wy = features_prev[n].y - iy;
//      float w00 = (1.f - wx) * (1.f - wy);
//      float w10 = (1.f - wx) * wy;
//      float w01 = wx * (1.f - wy);
//      float w11 = wx * wy;
//      if( features_prev[n].x >= flowMat.cols - 1 || features_prev[n].y >= flowMat.rows - 1)
//      {
//        // these points are out of the image roi and cannot be tracked.
//        features_next[n] = features_prev[n];
//      }
//      else
//      {
//        /*
//        bilinear interpolation of the flow vector from the flow field at a given location.
//        The bilinear interpolation has to be applied since the points to track can be given at subpixel level
//        */
//        features_next[n] = features_next[n]
//                        + flowMat.at<cv::Point2f>(iy, ix) * w00
//                        + flowMat.at<cv::Point2f>(iy+1, ix) * w10
//                        + flowMat.at<cv::Point2f>(iy, ix+1) * w01
//                        + flowMat.at<cv::Point2f>(iy+1, ix+1) * w11;
//
//        for( int i = 0; i < features_next.size(); i++ )
//        { circle( img_curr,features_next[i], 3, Scalar(rng.uniform(0,255), rng.uniform(0,255),
//                                                       rng.uniform(0,255)), -1, 8, 0 );
//
//        }
//
//        imshow("SURF POINTSss",img_curr);
//
//
//
//      }
//    }
//
//
//
//    waitKey(10);
//
//  }
//
//
//}
//
//
//
//
///* @function readme */
//void readme()
//{ std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl; }


#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
  cout <<
       "\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
               "Mainly the function: calcOpticalFlowFarneback()\n"
               "Call:\n"
               "./fback\n"
               "This reads from video camera 0\n" << endl;
}
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                           double, const Scalar& color)
{
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
    {
      const Point2f& fxy = flow.at<Point2f>(y, x);
      line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
           color);
      circle(cflowmap, Point(x,y), 2, color, -1);
    }
}

int main(int argc, char** argv)
{
  cv::CommandLineParser parser(argc, argv, "{help h||}");
  if (parser.has("help"))
  {

    help();
    return 0;
  }
  VideoCapture cap(0);
  help();
  if( !cap.isOpened() )
    return -1;

  Mat flow, cflow, frame;
  UMat gray, prevgray, uflow;
  namedWindow("flow", 1);

  for(;;)
  {
    cap >> frame;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    if( !prevgray.empty() )
    {
      calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
      cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
      uflow.copyTo(flow);
      drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
      imshow("flow", cflow);
    }
    (waitKey(10));
//      break;
    std::swap(prevgray, gray);
  }
  return 0;
}



