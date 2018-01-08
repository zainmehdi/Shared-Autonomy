//
// Created by zain on 17. 11. 22.
//

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
using namespace std;
using namespace cv;
int main( int argc, char** argv ){
    // show help

    VideoCapture cap(0);

    // declares all required variables
    Rect2d roi;
    Mat frame;
    Mat fram_gray;

    cap.read(frame);
    // create a tracker object
//    Ptr<Tracker> tracker = TrackerTLD::create();                   // not usable at all;
    //   Ptr<Tracker> tracker = TrackerKCF::create();                 // works fine
//   Ptr<Tracker> tracker = TrackerBoosting::create();
    Ptr<Tracker> tracker = TrackerMIL::create();
//    Ptr<Tracker> tracker = TrackerMedianFlow::create();          //works bad
//    Ptr<Tracker> tracker = TrackerGOTURN::create();


    // set input video

    // get bounding box
    cap >> frame;
    cvtColor(frame, fram_gray,CV_BGR2GRAY);
    roi=selectROI("tracker",fram_gray);
    //quit if ROI was not selected
    if(roi.width==0 || roi.height==0)
        return 0;
    // initialize the tracker
    tracker->init(fram_gray,roi);
    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    for ( ;; ){
        // get frame from the video
        cap >> frame;

        // stop the program if no more images
        if(frame.rows==0 || frame.cols==0)
            break;
        cvtColor(frame, fram_gray, CV_BGR2GRAY);
        // update the tracking result
        tracker->update(fram_gray,roi);
        // draw the tracked object
        rectangle( fram_gray, roi, Scalar( 255, 0, 0 ), 2, 1 );
        // show image with the tracked object
        imshow("tracker",fram_gray);
        //quit on ESC button
        if(waitKey(1)==27)break;
    }
    return 0;
}
//
// Created by zain on 17. 12. 5.
//

