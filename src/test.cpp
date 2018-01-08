#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/video/tracking.hpp"

#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;
using namespace std;
using namespace xfeatures2d;

IplImage* frame, * img1;
CvPoint point;
int drag = 0;
int x_point,width_point,y_point,height_point;
CvCapture *capture = 0;
int key = 0;
CvRect rect;
Rect region_of_interest;
int test;
Mat src,src_gray,image,src_gray_prev,src1,src_gray1,copy,copy1,frames,copy2;
int maxCorners = 10;
RNG rng(12345);
vector<Point2f> corners,corners_prev,corners_temp;
double qualityLevel = 0.01;
double minDistance = 10;
int blockSize = 3;
bool useHarrisDetector = false;
double k = 0.04;
vector<uchar> status;
vector<float> err;
float x_cord[100];
float y_cord[100];
int mean_y=0;
int mean_x=0;
vector<Point2f> corners_final;
Ptr<SURF> Surf_detector;
vector<KeyPoint> surf_keypoints,surf_keypoints_prev;
Mat descriptor;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        point = cvPoint(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        img1 = cvCloneImage(frame);
        cvRectangle(img1,point,cvPoint(x, y),CV_RGB(255, 0, 0),1,8,0);
        cvShowImage("result", img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        rect = cvRect(point.x,point.y,x-point.x,y-point.y);
        x_point = point.x;
        y_point = point.y;
        width_point = x-point.x;
        height_point = y-point.y;
        cvShowImage("result", frame);
        drag = 0;
    }


    if (event == CV_EVENT_RBUTTONUP)
    {
        drag = 0;
    }
}

int main(int argc, char *argv[])
{
    capture = cvCaptureFromCAM( 0 );
    if ( !capture ) {
        printf("Cannot open initialize webcam!\n" );
        exit(0);
    }

    int small,big; //declares integer

    int x = 1;

    while( key != 'q' )
    {
        frame = cvQueryFrame( capture );
        if (rect.width>0)
        {
            if(corners.size() == 0 || x==0)
            {
                Mat frames(cv::cvarrToMat(frame));
                src = frames.clone();
                cvtColor( src, src_gray, CV_BGR2GRAY );
                cv::Mat mask1 = cv::Mat::zeros(src.size(), CV_8UC1);
                cv::Mat roi(mask1, cv::Rect(x_point,y_point,width_point,height_point));
                roi = cv::Scalar(255, 255, 255);
                copy1 = src.clone();
                goodFeaturesToTrack( src_gray,
                                     corners,
                                     maxCorners,
                                     qualityLevel,
                                     minDistance,
                                     mask1,
                                     blockSize,
                                     useHarrisDetector,
                                     k );
               // Surf_detector->detectAndCompute(src_gray,mask1,surf_keypoints,descriptor);

                for(auto in:corners)
                {
                    mean_x=in.x+mean_x;
                    mean_y-in.y+mean_y;
                }
                mean_x=mean_x/corners.size();
                mean_y=mean_y/corners.size();

                Point2d mean;
                mean.x=mean_x;
                mean.y=mean_y;



                for(auto in:corners)
                {
                    int distance=sqrt(pow(mean.x-in.x,2)+pow(mean.y-in.y,2));
                    std::cout<<"distance:"<<distance<<endl;
                    if(distance<265)
                    {
                        corners_final.push_back(in);
                    }

                }

                int rad = 3;
                for( int i = 0; i < corners_final.size(); i++ )
                { circle( copy1, corners_final[i], rad, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                                                         rng.uniform(0,255)), -1, 8, 0 );

                }
                IplImage test1 = copy1;
                IplImage* test2 = &test1;
                x = 1;

                cvShowImage("result", test2);
            }
            else
            {
               surf_keypoints_prev=surf_keypoints;
                src_gray_prev = src_gray.clone();
                corners_prev = corners_final;
                Mat framess(cv::cvarrToMat(frame));
                src = framess.clone();
                cvtColor( src, src_gray, CV_BGR2GRAY );
                cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
                cv::Mat roi(mask, cv::Rect(x_point,y_point,width_point,height_point));
                roi = cv::Scalar(255, 255, 255);
                Mat copy;
                copy = src.clone();
                goodFeaturesToTrack( src_gray,
                                     corners,
                                     maxCorners,
                                     qualityLevel,
                                     minDistance,
                                     mask,
                                     blockSize,
                                     useHarrisDetector,
                                     k );
              //  Surf_detector->detectAndCompute(src_gray,mask,surf_keypoints,descriptor);
                for(auto in:corners)
                {
                    mean_x=in.x+mean_x;
                    mean_y-in.y+mean_y;
                }
                mean_x=mean_x/corners.size();
                mean_y=mean_y/corners.size();

                Point2d mean;
                mean.x=mean_x;
                mean.y=mean_y;



                for(auto in:corners)
                {
                    int distance=sqrt(pow(mean.x-in.x,2)+pow(mean.y-in.y,2));
                    std::cout<<"distance:"<<distance<<endl;
                    if(distance<265)
                    {
                        corners_final.push_back(in);
                    }

                }

                calcOpticalFlowPyrLK(src_gray_prev, src_gray, corners_prev, corners_final, status, err);
             //   calcOpticalFlowPyrLK(src_gray_prev, src_gray, surf_keypoints_prev, surf_keypoints, status, err);

                int r = 3;
                for( int i = 0; i < corners_final.size(); i++ )
                { circle( copy, corners_final[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                                                      rng.uniform(0,255)), -1, 8, 0 );
                    x_cord[i] = corners_final[i].x;
                    y_cord[i] = corners_final[i].y;

                }

                IplImage test3 = copy;
                IplImage* test4 = &test3;




                Mat final=cvarrToMat(test4);
                rectangle(final, boundingRect(corners_final),Scalar( 255, 0, 0 ),2, LINE_8);
                imshow("result", final);
            }



        }

        cvSetMouseCallback("result", mouseHandler, NULL);
        key = cvWaitKey(10);
        if( (char) key== 'r' )
        {
            rect = cvRect(0,0,0,0); cvResetImageROI(frame);
            x = 0;
        }
        cvShowImage("result", frame);
//	cout << x_point << " " << y_point <<  " " << width_point << " " << height_point<< endl;

    }
    cvDestroyWindow("result");
    cvReleaseImage(&img1);
    return 0;
}
