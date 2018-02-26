/*
 * @file SURF_FlannMatcher
 * @brief SURF detector + descriptor + FLANN Matcher
 * @author A. Huaman
 */
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
void readme();
/*
 * @function main
 * @brief Main function
 */
int main( int argc, char** argv )
{
    if( argc != 4 )
    { readme(); return -1; }
    Mat img_1 = imread( argv[1], IMREAD_GRAYSCALE );
    Mat img_2 = imread( argv[2], IMREAD_GRAYSCALE );
    Mat mask= imread( argv[3], IMREAD_GRAYSCALE );
    Mat kp;
    if( !img_1.data || !img_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors


    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create();

    Ptr<DescriptorExtractor> extractor= SIFT::create();

//    detector->setHessianThreshold(minHessian);
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    Ptr<FeatureDetector> detector= ORB::create();
    detector->detect(img_1,keypoints_1,mask);
    detector->detect( img_2, keypoints_2);

    extractor->compute(img_1,keypoints_1,descriptors_1);
    extractor->compute(img_2,keypoints_2,descriptors_2);

    drawKeypoints(img_1,keypoints_1,kp);
    imshow("Keypoints in original image",kp);

    //-- Step 2: Matching descriptor vectors using FLANN matcher

    Ptr<DescriptorMatcher> matcher =DescriptorMatcher::create("FlannBased");
    std::vector<vector< DMatch> > matches;
    matcher->knnMatch(descriptors_1,descriptors_2,matches,2);
    double max_dist = 0; double min_dist = 10;

    //-- Quick calculation of max and min distances between keypoints

    vector<DMatch> goodmatches;
    goodmatches.reserve(matches.size());

    for( int i = 0; i < matches.size(); i++ )
    {
        if(matches[i].size()<2)
            continue;
        const DMatch &m1=matches[i][0];
        const DMatch &m2=matches[i][1];

        if(m1.distance <= 0.9* m2.distance)
            goodmatches.push_back(m1);
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                 goodmatches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    imshow( "Good Matches", img_matches );
    for( int i = 0; i < (int)goodmatches.size(); i++ )
    { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, goodmatches[i].queryIdx, goodmatches[i].trainIdx ); }
    waitKey(0);
    return 0;
}
/*
 * @function readme
 */
void readme()
{ std::cout << " Usage: ./SURF_FlannMatcher <img1> <img2>" << std::endl; }
