
#include <iostream>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotPioneer.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
#  define TEST_COULD_BE_ACHIEVED
#endif

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

const int MAX_COUNT = 30;
bool needToInit = false;
bool nightMode = false;

Mat gray, prevGray, image, frame;
vector<Point2f> points[2];
Point2f point;
Point2f point1;
bool addRemovePt = false;
bool feature_selected=false;

/*!
  \example tutorial_ros_node_pioneer_visual_servo.cpp

  Example that shows how to create a ROS node able to control the Pioneer mobile robot by IBVS visual servoing with respect to a blob.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point corresponding to the blob center of gravity measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is proportional to the depth Z.

  */


static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
        feature_selected=true;
    }
}



int main(int argc, char **argv)
{
    try {
        vpImage<unsigned char> I; // Create a gray level image container
        double depth = 1;
        double lambda = 0.6;
        double coef = 1./6.77; // Scale parameter used to estimate the depth Z of the blob from its surface

        namedWindow( "LK Demo", 1 );
        setMouseCallback( "LK Demo", onMouse, 0 );
        vpROSRobotPioneer robot;
        robot.setCmdVelTopic("/RosAria/cmd_vel");
        robot.init(argc, argv);

        // Wait 3 sec to be sure that the low level Aria thread used to control
        // the robot is started. Without this delay we experienced a delay (arround 2.2 sec)
        // between the velocity send to the robot and the velocity that is really applied
        // to the wheels.
        vpTime::sleepMs(3000);

        std::cout << "Robot connected" << std::endl;

        // Camera parameters. In this experiment we don't need a precise calibration of the camera
        vpCameraParameters cam;

        // Create a grabber based on libdc1394-2.x third party lib (for firewire cameras under Linux)
        vpROSGrabber g;
        g.setCameraInfoTopic("/usb_cam/camera_info");
        g.setImageTopic("/usb_cam/image_raw");
        g.setRectify(true);
        g.open(argc, argv);
        // Get camera parameters from /camera/camera_info topic
        if (g.getCameraInfo(cam) == false)
            cam.initPersProjWithoutDistortion(600,600,I.getWidth()/2, I.getHeight()/2);

        g.acquire(I);


        /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
        ///////////////////////////////////////////////////////////////////////////////////

        vpImageConvert::convert(I, image);
        image.copyTo(frame);


      while(1)
      {
          if (frame.empty())
              break;

        //  frame.copyTo(image);
        //  cvtColor(image, gray, COLOR_BGR2GRAY);

          frame.copyTo(gray);
          if (nightMode)
              image = Scalar::all(0);

          if (needToInit) {
              // automatic initialization
              goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
              cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
              addRemovePt = false;
          } else if (!points[0].empty()) {
              vector<uchar> status;
              vector<float> err;
              if (prevGray.empty())
                  gray.copyTo(prevGray);
              calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                   3, termcrit, 0, 0.001);
              size_t i, k;
              for (i = k = 0; i < points[1].size(); i++) {
                  if (addRemovePt) {
                      if (norm(point - points[1][i]) <= 5) {
                          addRemovePt = false;
                          continue;
                      }
                  }

                  if (!status[i])
                      continue;

                  points[1][k++] = points[1][i];
                  circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
                  cout << "Points Location: " << points[1][i] << "\n";
              }
              points[1].resize(k);
          }

          if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
              vector<Point2f> tmp;
              tmp.push_back(point);
              cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
              points[1].push_back(tmp[0]);
              addRemovePt = false;
          }

          needToInit = false;
          imshow("LK Demo", image);

          char c = (char) waitKey(10);
          if (c == 27)
              break;
          switch (c) {
              case 'r':
                //r  needToInit = true;
                  break;
              case 'c':
                  points[0].clear();
                  points[1].clear();
                  break;
              case 'n':
                  nightMode = !nightMode;
                  break;
          }

          std::swap(points[1], points[0]);
          cv::swap(prevGray, gray);

      }


        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////


        // Create an image viewer
        vpDisplayX d(I, 10, 10, "Current frame");
        vpDisplay::display(I);
        vpDisplay::flush(I);

//        // Create a blob tracker
//        vpDot2 dot;
//        dot.setGraphics(true);
//        dot.setComputeMoments(true);
//        dot.setEllipsoidShapePrecision(0.);  // to track a blob without any constraint on the shape
//        dot.setGrayLevelPrecision(0.5);  // to set the blob gray level bounds for binarisation
//        dot.setEllipsoidBadPointsPercentage(0.5); // to be accept 50% of bad inner and outside points with bad gray level
//        dot.initTracking(I);
//        vpDisplay::flush(I);




        vpServo task;
        task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
        task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;
        task.setLambda(lambda) ;
        vpVelocityTwistMatrix cVe ;
        cVe = robot.get_cVe() ;
        task.set_cVe(cVe) ;

        std::cout << "cVe: \n" << cVe << std::endl;

        vpMatrix eJe;
        robot.get_eJe(eJe) ;
        task.set_eJe(eJe) ;
        std::cout << "eJe: \n" << eJe << std::endl;

        // Current and desired visual feature associated to the x coordinate of the point
        vpFeaturePoint s_x, s_xd,s_x1,s_xd1;

        // Create the current x visual feature
        vpFeatureBuilder::create(s_x, cam, vpImagePoint(point.x,point.y));
        vpFeatureBuilder::create(s_x1, cam, vpImagePoint(point.x,point.y));
        cout<<" Coordinates in image plane \n"<<s_x.get_x()<<" "<<s_x.get_y();

        // Create the desired x* visual feature
        s_xd.buildFrom(0, 0, depth);
//        s_xd1.buildFrom(0, 0, depth);

        // Add the feature
        task.addFeature(s_x, s_xd) ;
//        task.addFeature(s_x1, s_xd1) ;

        // Create the current log(Z/Z*) visual feature
        vpFeatureDepth s_Z, s_Zd;
        // Surface of the blob estimated from the image moment m00 and converted in meters
      //  double surface = 1./sqrt(dot.m00/(cam.get_px()*cam.get_py()));
        double Z, Zd;
        // Initial depth of the blob in from of the camera
     //   Z = coef * surface ;
        Z=3;
        // Desired depth Z* of the blob. This depth is learned and equal to the initial depth
        Zd = 3;

        std::cout << "Z " << Z << std::endl;
        s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
        s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0

        // Add the feature
//        task.addFeature(s_Z, s_Zd) ;
        task.addFeature(s_Z, s_Zd) ;

        vpColVector v; // vz, wx

        while(1)
        {
            // Acquire a new image
            g.acquire(I);


            /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
            ///////////////////////////////////////////////////////////////////////////////////

            vpImageConvert::convert(I, image);
            image.copyTo(frame);



                if (frame.empty())
                    break;

//                frame.copyTo(image);
//                cvtColor(image, gray, COLOR_BGR2GRAY);

                frame.copyTo(gray);
                if (nightMode)
                    image = Scalar::all(0);

                if (needToInit) {
                    // automatic initialization
                    goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
                    cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                    addRemovePt = false;
                } else if (!points[0].empty()) {
                    vector<uchar> status;
                    vector<float> err;
                    if (prevGray.empty())
                        gray.copyTo(prevGray);
                    calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                         3, termcrit, 0, 0.001);
                    size_t i, k;
                    for (i = k = 0; i < points[1].size(); i++) {
                        if (addRemovePt) {
                            if (norm(point - points[1][i]) <= 5) {
                                addRemovePt = false;
                                continue;
                            }
                        }

                        if (!status[i])
                            continue;

                        points[1][k++] = points[1][i];
                        circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
                        point1=points[1][i];
                        cout << "Points Location: " << points[1][i] << "\n";
                    }
                    points[1].resize(k);
                }

                if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
                    vector<Point2f> tmp;
                    tmp.push_back(point);
                    cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
                    points[1].push_back(tmp[0]);
                    addRemovePt = false;
                }

                needToInit = false;
                imshow("LK Demo", image);
                waitKey(1);

                std::swap(points[1], points[0]);
                cv::swap(prevGray, gray);


            //////////////////////////////OpenCV part ends here /////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////





            // Set the image as background of the viewer
            vpDisplay::display(I);
            vpDisplay::flush(I);
//
            // Does the blob tracking
//            dot.track(I);
            // Update the current x feature
            vpFeatureBuilder::create(s_x, cam, vpImagePoint(point1.x,point1.y));
            cout<<" Coordinates in image plane "<<s_x.get_x()<<" "<<s_x.get_y()<<endl;
//            vpFeatureBuilder::create(s_x1, cam, vpImagePoint(point1.x,point1.y));
         //   cout << "Points Location: " << s_x.get_x() << "\t"<<s_x.get_y()<<endl;
            // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
          //  surface = 1./sqrt(dot.m00/(cam.get_px()*cam.get_py()));
//            Z = coef * surface ;
//            Z=Zd;
            s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)) ;

            robot.get_cVe(cVe) ;
            task.set_cVe(cVe) ;

            robot.get_eJe(eJe) ;
            task.set_eJe(eJe) ;

            // Compute the control law. Velocities are computed in the mobile robot reference frame
            v = task.computeControlLaw() ;

            std::cout << "Send velocity to the pionner: " << v[0] << " m/s "
                      << vpMath::deg(v[1]) << " deg/s" << std::endl;

            // Send the velocity to the robot
            robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

            // Draw a vertical line which corresponds to the desired x coordinate of the dot cog
            vpDisplay::displayLine(I, 0, 320, 479, 320, vpColor::red);
            vpDisplay::flush(I);

            // A click in the viewer to exit
            if ( vpDisplay::getClick(I, false) )
                break;
        }

        std::cout << "Ending robot thread..." << std::endl;

        // Kill the servo task
        task.print() ;
        task.kill();
    }
    catch(vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
        return 1;
    }
}


