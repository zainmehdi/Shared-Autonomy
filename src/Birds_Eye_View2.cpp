//
// Created by zain on 17. 12. 25.
//

//Example 19-1. Bird’s - eye view
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv/cv.hpp"

#include <iostream>
using namespace std;
using namespace cv;
Point2f point;
void help(char *argv[]) {
    cout	<< "\nExample 19-01, using homography to get a bird's eye view."
            << "\nThis file relies on you having created an intrinsic file via example_18-01_from_disk"
            << "\n   but here, that file is already stored in ../birdseye/intrinsics.xml"
            << "\nCall:"
            << "\n./example_19-01 <chessboard_width> <chessboard_height> <path/camera_calib_filename> <path/chessboard_image>"
            << "\n\nExample:"
            << "\n./example_19-01 12 12 ../birdseye/intrinsics.xml ../birdseye/IMG_0215L.jpg\n"
            << "\nPress 'd' for lower birdseye view, and 'u' for higher (it adjusts the apparent 'Z' height), Esc to exit\n"
            << endl;
}


void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
    }
}
// args: [board_w] [board_h] [intrinsics.xml] [checker_image]
//
int main(int argc, char *argv[]) {

    // Input Parameters:
    //
    int board_w = 8;
    int board_h = 6;
    int board_n = board_w * board_h;
    cv::Size board_sz(board_w, board_h);
    cv::Mat intrinsic, distortion;
    int i=1;


   intrinsic.create(3,3,cv::DataType<double>::type);
    intrinsic.at<double>(0,0)=8.4028771356606751e+02;
    intrinsic.at<double>(0,1)=0;
    intrinsic.at<double>(0,2)=320;
    intrinsic.at<double>(1,0)=0;
    intrinsic.at<double>(1,1)=8.4282050609881685e+02;
    intrinsic.at<double>(1,2)=240;
    intrinsic.at<double>(2,0)=0;
    intrinsic.at<double>(2,1)=0;
    intrinsic.at<double>(2,2)=1;

   distortion.create(1,5,cv::DataType<double>::type);
    distortion.at<double>(0,0)=0.1185028650130305;
    distortion.at<double>(0,0)=-0.5150168858561729;
    distortion.at<double>(0,0)=0.0127408188974029;
    distortion.at<double>(0,0)=0.006831715564244691;
    distortion.at<double>(0,0)=0;

    VideoCapture cap(0);
    namedWindow("Birds_Eye", 1);
    setMouseCallback( "Birds_Eye", onMouse, 0 );

    cv::Mat gray_image, image, image0 ;
    cap>>image0;
    if (image0.empty()) {
        cout << "Error: Couldn't load image " << argv[1] << endl;
        return -1;
    }



    // UNDISTORT OUR IMAGE
    //
    cv::undistort(image0, image, intrinsic, distortion, intrinsic);
    cv::cvtColor(image, gray_image, cv::COLOR_BGRA2GRAY);

    // GET THE CHECKERBOARD ON THE PLANE
    //
    vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners( // True if found
            image,                              // Input image
            board_sz,                           // Pattern size
            corners,                            // Results
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (!found) {
        cout << "Couldn't acquire checkerboard on " << argv[1] << ", only found "
             << corners.size() << " of " << board_n << " corners\n";
        return -1;
    }

    // Get Subpixel accuracy on those corners
    //
    cv::cornerSubPix(
            gray_image,       // Input image
            corners,          // Initial guesses, also output
            cv::Size(11, 11), // Search window size
            cv::Size(-1, -1), // Zero zone (in this case, don't use)
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30,
                             0.1));

    // GET THE IMAGE AND OBJECT POINTS:
    // Object points are at (r,c):
    // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1)
    // That means corners are at: corners[r*board_w + c]
    //
    cv::Point2f objPts[4], imgPts[4];
    objPts[0].x = 0;
    objPts[0].y = 0;
    objPts[1].x = board_w - 1;
    objPts[1].y = 0;
    objPts[2].x = 0;
    objPts[2].y = board_h - 1;
    objPts[3].x = board_w - 1;
    objPts[3].y = board_h - 1;
    imgPts[0] = corners[0];
    imgPts[1] = corners[board_w - 1];
    imgPts[2] = corners[(board_h - 1) * board_w];
    imgPts[3] = corners[(board_h - 1) * board_w + board_w - 1];

    // DRAW THE POINTS in order: B,G,R,YELLOW
    //
    cv::circle(image, imgPts[0], 9, cv::Scalar(255, 0, 0), 3);
    cv::circle(image, imgPts[1], 9, cv::Scalar(0, 255, 0), 3);
    cv::circle(image, imgPts[2], 9, cv::Scalar(0, 0, 255), 3);
    cv::circle(image, imgPts[3], 9, cv::Scalar(0, 255, 255), 3);

    // DRAW THE FOUND CHECKERBOARD
    //
    cv::drawChessboardCorners(image, board_sz, corners, found);
    cv::imshow("Checkers", image);

    // FIND THE HOMOGRAPHY
    //
    cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);

    // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
    //
    cout << "\nPress 'd' for lower birdseye view, and 'u' for higher (it adjusts the apparent 'Z' height), Esc to exit" << endl;
    double Z = 15;
    cv::Mat birds_image;
    for (;;) {

       if(i>1)
       {
           cap>>image0;
           cv::undistort(image0, image, intrinsic, distortion, intrinsic);
           cv::cvtColor(image, gray_image, cv::COLOR_BGRA2GRAY);
       }


        // escape key stops
        H.at<double>(2, 2) = Z;
        // USE HOMOGRAPHY TO REMAP THE VIEW
        //
        cv::warpPerspective(image,			// Source image
                            birds_image, 	// Output image
                            H,              // Transformation matrix
                            image.size(),   // Size for output image
                            cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT, cv::Scalar::all(0) // Fill border with black
        );

        circle( birds_image, point, 6, Scalar(0,0,255), -1, 8);
        cv::imshow("Birds_Eye", birds_image);
        cv::imshow("Checkers", image);
        int key = cv::waitKey(1);
        if (key == 'u')
            Z += 0.5;
        if (key == 'd')
            Z -= 0.5;
        if (key == 27)
            break;

        i++;
    }

    // SHOW ROTATION AND TRANSLATION VECTORS
    //
    vector<cv::Point2f> image_points;
    vector<cv::Point3f> object_points;
    for (int i = 0; i < 4; ++i) {
        image_points.push_back(imgPts[i]);
        object_points.push_back(cv::Point3f(objPts[i].x, objPts[i].y, 0));
    }
    cv::Mat rvec, tvec, rmat;
    cv::solvePnP(object_points, 	// 3-d points in object coordinate
                 image_points,  	// 2-d points in image coordinates
                 intrinsic,     	// Our camera matrix
                 cv::Mat(),     	// Since we corrected distortion in the
            // beginning,now we have zero distortion
            // coefficients
                 rvec, 			// Output rotation *vector*.
                 tvec  			// Output translation vector.
    );
    cv::Rodrigues(rvec, rmat);

    // PRINT AND EXIT
    cout << "rotation matrix: " << rmat << endl;
    cout << "translation vector: " << tvec << endl;
    cout << "homography matrix: " << H << endl;
    cout << "inverted homography matrix: " << H.inv() << endl;

    return 1;
}