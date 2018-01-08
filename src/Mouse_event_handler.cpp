//
// Created by zain on 17. 11. 24.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <iostream>
#include "opencv_test/Draw_many_images_function.h"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/features2d.hpp"


/// This code is a part development done under my thesis " Local Autonomy" image processing section
/// The aim is to draw a region using mouse in the image and extract and track keypoints int the locations
/// marked by those mouse position coordinates. My approach here is to draw a line, make regions of interest
/// at points marked by the line, extract templates (cropped images) at those points and track them sequentially
/// that will be use later in visual servoing (VISP section).


using namespace std;
using namespace cv;
vector<Point> line_points;
vector<Point> selected_points;
bool drag;
Mat img;
bool first_run=true;
Point p;
int circle_radius=2;
Mat selected_image;
vector<Mat> cropped_images;
Rect2d cropped_rect;
typedef std::vector<KeyPoint> keypoints;
int minHessian = 400;
using namespace cv::xfeatures2d;



/// This function selects a list of points from all the points that constitute the circles
/// I used a distance metric to select points that will be used to ultimately draw rectangles
/// for the cropped section for feature locations. D= (p2-p1) where p1 is the starting point
/// and p2 is the next point. I keep first point in this case p1 as the pivot and keep calculating
/// distance from incoming next points p2,p3,p4 ... until distance condition is satisfied. Once the
/// condition is satisfied I select that point as the location for drawing rectangle and then start
/// measuring distance from this point onwards. for example I started from p1 which will definitely
/// a rectangle but I need the next location. When distance condition is fulfilled lets say at p6, It
/// will be our new pivot point and a rectangle will be drawn here, but now all the distances will be
/// measured from p6 onwards

void points_selector(vector<Point> &all_points){

    for(auto i:all_points)
    {

        if(first_run)
        {
            p=i;
            first_run=false;
            cout<<"condition became true";
            continue;
        }

        int distance=sqrt(pow(p.x-i.x,2)+pow(p.y-i.y,2));
        cout<<"distance:"<<distance<<endl;
        if(distance>2*circle_radius*5)
        {


            selected_points.push_back(i);
            p=i;
        }

    }

}


/// This function draws rectangles at the points we pass it. usually the center point of circles
/// are passed to it and then I add desired width and hieght of rectangle to that center point
/// and construct a rectangle.

void draw_segmented_rectrangles(Mat i,int width,int height,vector<Point> &k){

    Mat mask=Mat::zeros(img.size(),CV_8U);
    std::vector<keypoints> surf_keypoints;
    std::vector<Mat> descriptors;
    Mat img_with_keypoints;
    Ptr<SURF> detector = SURF::create( minHessian );
    int y=0;

    for(auto g:k)
   {
       rectangle(i,Point(g.x-width,g.y-height),Point(g.x+width,g.y+height),CV_RGB(255, 0, 0),1,8,0);
       Rect2d r(g.x,g.y,width,height);
       Mat roi(mask,r);
       roi = Scalar(255, 255, 255);
       detector->detectAndCompute( img,mask, surf_keypoints[y],descriptors[y]);
       drawKeypoints( img, surf_keypoints[y], img_with_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
       y++;

   }
    line_points.clear();
    namedWindow("Original Image", 1);
    imshow("Original Image", img_with_keypoints);
}

/// This function simply draws a circle at points that are passed via mouse callback function

void draw_circle(Mat img,vector<Point> &lp)
{
    for(auto index:lp)
    {
        circle(img,Point(index), circle_radius, CV_RGB(255, 0, 0),1,8,0);
    }


}

/// This function generates cropped images (templates) from the rectangles I draw on the image
/// that will be used later in feature extraction and tracking

void cropped_image_extractor(Mat &t,vector<Point> &w)
{
    for(auto q:w)
    {

        cropped_rect.x=q.x;
        cropped_rect.y=q.y;
        cropped_rect.width=10;
        cropped_rect.height=10;
        cropped_images.push_back(t(cropped_rect));
        cout<<"Image pushed"<<endl;
    }

}

/// This is the mouse callback function that process pixel information that cursor is pointing to
/// I use this information for drawing circles.

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

    if  ( event == EVENT_LBUTTONDOWN && !drag)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        drag=true;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;


    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

    }
    else if ( event == EVENT_MOUSEMOVE && drag)
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        line_points.push_back(CvPoint(x,y));


    }
    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }
}






///////////////////////////// Main Function ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{


    namedWindow("My Window", 1);
    VideoCapture cap(0);


   while(1) {
       cap>>img;


       //if fail to read the image
       if (img.empty()) {
           cout << "Error loading the image" << endl;
           return -1;
       }




     /// If number of points are not zero then proceed
     /// and select points and then draw rectangles at those points.
     /// Also extract images in accord with the region bounded by those
     /// rectangles

       if(!line_points.empty())
      {
          selected_image=img;
          points_selector(line_points);
          draw_segmented_rectrangles(selected_image,10,10,selected_points);


      }

       imshow("My Window", img);

       /// Set the callback function for any mouse event
       setMouseCallback("My Window", CallBackFunc, NULL);



     /// Interrupts for termination a
     /// Wait until user press some key
      int key = waitKey(1);
       if( (char) key== 'r' )
       {
           line_points.clear();       // clears all circle points
           selected_points.clear();   // clears circle points that were selected
           destroyWindow("Cropped Images");
       }

      /// When d is pressed images will be extracted from rectangular regions and displayed on screen

        if ( (char )key=='d')
       {

//           cropped_image_extractor(selected_image,selected_points);
//
//              try{
//                  ShowManyImages("Cropped Sections",12,cropped_images[0],cropped_images[1],cropped_images[2],
//                                 cropped_images[3],cropped_images[4],cropped_images[5],cropped_images[6],
//                                 cropped_images[7],cropped_images[8],cropped_images[9],cropped_images[10],cropped_images[11]);
//                                // cropped_images[12],cropped_images[13]);
//              }
//              catch (invalid_argument &e)
//              {
//                  cerr<<e.what()<<endl;
//                  return -1;
//              }
       }


   }

    return 0;

}