//
// Created by kari on 18. 1. 31.
//

#include "opencv_test/Shared_Autonomy.h"


int Shared_Autonomy_Vine::MAX_COUNT = 500;
int Shared_Autonomy_Vine::line_point_size;
int Shared_Autonomy_Vine::n=0;
int Shared_Autonomy_Vine::width=4;
int Shared_Autonomy_Vine::height=4;
int Shared_Autonomy_Vine::m=0;
int Shared_Autonomy_Vine::circle_radius=2;
int Shared_Autonomy_Vine::e=0;

bool Shared_Autonomy_Vine::needToInit = false;
bool Shared_Autonomy_Vine::nightMode = false;
bool Shared_Autonomy_Vine::features_found=false;
bool Shared_Autonomy_Vine::addRemovePt = false;
bool Shared_Autonomy_Vine::feature_selected=false;
bool Shared_Autonomy_Vine::first_run=true;

bool Shared_Autonomy_Vine::path_drawn=false;
bool Shared_Autonomy_Vine::feature_on_path_found=false;
bool Shared_Autonomy_Vine::path_feature_found=false;
bool Shared_Autonomy_Vine::drag;

Mat Shared_Autonomy_Vine::gray, Shared_Autonomy_Vine::prevGray, Shared_Autonomy_Vine::image, Shared_Autonomy_Vine::frame;
Mat Shared_Autonomy_Vine::mask;
Mat Shared_Autonomy_Vine::roi;
Mat Shared_Autonomy_Vine::transformation_bw_goal_nf;




double Shared_Autonomy_Vine::th_prev =0;
double Shared_Autonomy_Vine::t_y;
double Shared_Autonomy_Vine::camera_height=40;
double Shared_Autonomy_Vine::camera_pitch = 45;


vector<Point2f> Shared_Autonomy_Vine::points[2];
vector<Point2f> Shared_Autonomy_Vine::point_buffer;
vector<Point2f> Shared_Autonomy_Vine::pp;
vector<cv::Mat> Shared_Autonomy_Vine::transformation_bw_line;
vector<Point> Shared_Autonomy_Vine::line_points;
vector<Point> Shared_Autonomy_Vine::line_points_temp;
vector<Point> Shared_Autonomy_Vine::line_points_world;
vector<Point> Shared_Autonomy_Vine::transformed_points;
vector<Point> Shared_Autonomy_Vine::line_points_circle;
vector<Point> Shared_Autonomy_Vine::selected_points;

Point2f Shared_Autonomy_Vine::point;
Point2f Shared_Autonomy_Vine::point1;
Point2f Shared_Autonomy_Vine::desired_point;
Point2f Shared_Autonomy_Vine::previous,current;
Point2f Shared_Autonomy_Vine::nearest_feature;


Point Shared_Autonomy_Vine::Target_point;
Point Shared_Autonomy_Vine::Target_point_prev;
Point Shared_Autonomy_Vine::p;

geometry_msgs::Twist Shared_Autonomy_Vine::v;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(30,30);



Shared_Autonomy_Vine::Shared_Autonomy_Vine(): it(nh) {


    desired_point = Point(320, 475);                                                        // reference point that will be followed in VS
    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    image_sub = it.subscribe("image_raw", 1, &Shared_Autonomy_Vine::imageCb, this);

    pstate= FEATURE_FINDING;                                                               // setting the current state of the machine to path drawing mode
    sstate = PATH_DRAWING;                                                                 // by default we will find features on path to for servoing

    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );


}



void Shared_Autonomy_Vine::points_selector(vector<Point> &all_points){


    for(auto i:all_points)
    {

        if(first_run)
        {
            p=i;
            first_run=false;
            selected_points.push_back(i);
            continue;
        }

        int distance=sqrt(pow(p.x-i.x,2)+pow(p.y-i.y,2));
        if(distance>5)
        {

            selected_points.push_back(i);
            p=i;
        }

    }
    line_points_temp.clear();

}



void Shared_Autonomy_Vine::onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ ) {
    if (event == EVENT_LBUTTONDOWN && !drag) {

        drag = true;
    } else if (event == EVENT_MOUSEMOVE && drag) {

        line_points.push_back(CvPoint(x, y));
        line_points_temp.push_back(CvPoint(x, y));
        points_selector(line_points_temp);


    } else if (event == EVENT_LBUTTONUP) {
        drag = false;
    }
}

Mat Shared_Autonomy_Vine::transformation_calculate(Point2f x, Point2f y)
{
    double Tx=y.x-x.x;
    double Ty=y.y-x.y;
    double theta=atan((y.y-x.y)/(y.x-x.x));

    Mat T=Mat_<double>(3,3);
    T.at<double>(0,0)=cos(theta*3.1428/180);
    T.at<double>(0,1)=-sin(theta*3.1428/180);
    T.at<double>(0,2)=Tx;
    T.at<double>(1,0)=sin(theta*3.1428/180);
    T.at<double>(1,1)=cos(theta*3.1428/180);
    T.at<double>(1,2)=Ty;
    T.at<double>(2,0)=0;
    T.at<double>(2,1)=0;
    T.at<double>(2,2)=1;

    return T;
}




double Shared_Autonomy_Vine::distance(Point a,Point b)
{
    return sqrt(pow(b.x-a.x,2)+(b.y-a.y,2));
}




void Shared_Autonomy_Vine::imageCb(const sensor_msgs::ImageConstPtr& msg)
{


    // Image acquisition using CV bridge

    /// *********************************************************

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;

    /// *********************************************************


    switch (sstate)
    {

        case PATH_DRAWING:
        {
            ///************************ User draws a path **********************
            /// The first step would always be to draw a path so that it could be followed ///

            if( frame.empty() )
                return;

            frame.copyTo(image);
            cvtColor(image, gray, COLOR_BGR2GRAY);
            if (nightMode)
                image = Scalar::all(0);

            for (auto index:line_points) {

                circle(image, Point(index), 5, CV_RGB(255, 255, 0), 1, 8, 0);
            }

            for (auto g:selected_points) {
                rectangle(image, Point(g.x - width / 2, g.y - height / 2), Point(g.x + width / 2, g.y + height / 2),
                          CV_RGB(255, 0, 0), 1, 8, 0);

            }

            char c = (char) waitKey(10);
            if (c == 27)
            {
                pstate=FEATURE_FINDING;
                sstate=RELATIVE_TRANSFORMATION;
                cout<<"Entered Feature Finding \n";
                return;

            }

            switch (c) {
                case 'c':
                    line_points.clear();
                    points[0].clear();
                    points[1].clear();
                    selected_points.clear();
                    break;
            }

            circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);


            v.linear.y = 0;
            v.linear.x = 0;
            v.angular.z= 0;
            vel_pub.publish(v);

            imshow("LK Demo", image);

            break;

            ///************************** User draws a path-end *************************************///

        }





            /*
             * *************************************************************************************************************************
             * This is the case where we find global features in the whole image and use them to update the path the user drew.
             * Optical flow is used to track features' displacement in images and update the path as a function of image displacement
             */

        case GLOBAL_FEATURES:
        {

            switch (pstate)
            {
                case FEATURE_FINDING:
                {
                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);
                    if (nightMode)
                        image = Scalar::all(0);

                    if (needToInit) {
                        // automatic initialization
                        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
                        cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                        addRemovePt = false;
                    } else if (!points[0].empty()) {
                        size_t i, k;

                        for (i = k = 0; i < points[1].size(); i++) {
                            if (addRemovePt) {
                                if (norm(point - points[1][i]) <= 5) {
                                    addRemovePt = false;
                                    continue;
                                }
                            }

                            points[1][k++] = points[1][i];
                            circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
                        }


                        for(auto index:line_points)
                        {

                            circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);
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

                    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

                    needToInit = false;
                    imshow("LK Demo", image);

                    char c = (char) waitKey(10);
                    if (c == 27)
                    {
                        pstate=VISUAL_SERVOING;
                        return;
                    }

                    switch (c) {
                        case 'r':
                            needToInit = true;
                            cout<<"Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            sstate=PATH_DRAWING;
                            cout<<"Points Cleared \n";
                            break;
                        case 'n':
                            nightMode = !nightMode;
                            break;
                    }



                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);



                    line_point_size=line_points.size();
                    break;
                }


                case VISUAL_SERVOING:
                {
                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);
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

                        /// Find rigid transformation between points of optical flow and use it to transform your own points
                        Mat transformation=estimateRigidTransform(points[0],points[1],false);

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
                            circle(image, desired_point, 5, Scalar(0, 150, 0), -1, 8);
                            point1 = points[1][i];

                        }



                        transform(line_points,transformed_points,transformation);
                        line_points=transformed_points;



                        for(auto index:line_points)
                        {

                            circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);


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
                    features_found = true;

                    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);
                    imshow("LK Demo", image);
                    waitKey(1);



                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);

                    Target_point_prev=Target_point;


                    if(points[1].size()<30)
                    {
                        needToInit=true;
                    }


                    v.linear.y = (-line_points[n].x + desired_point.x) / 800;
                    v.linear.x = (-line_points[n].y + desired_point.y) / 1000;
                    v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));

                    vel_pub.publish(v);



                    if(distance(desired_point,line_points[n]) < 5)
                    {
                        n++;
                        if(n==line_points.size())
                            return;

                    }





                    char c = (char) waitKey(10);
                    if (c == 27)
                        return;
                    switch (c) {
                        case 'r':
                            needToInit = true;
                            cout<<"Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            sstate=PATH_DRAWING;
                            cout<<"Points Cleared \n";
                            break;
                        case 'n':
                            nightMode = !nightMode;
                            break;
                    }



                    break;
                }


            }



            break;
        }


            //************************************************************************************************************************





            /*
             * *************************************************************************************************************************
             * This is the case where we draw a path on image and then use a mask to find features exactly on the path drawn and ignore
             * other areas. The features found serve as the path later on and are tracked and updated using optical flow
             */

        case FEATURES_ON_LINE:
        {
            switch (pstate)
            {

                case FEATURE_FINDING:
                {
                    ///********************************* Finding Features **********************************///

                    waitKey(10);

                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);
                    if (nightMode)
                        image = Scalar::all(0);


                    if (needToInit) {

                        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                        for (auto it:selected_points) {

                            mask = Mat::zeros(gray.size(), CV_8U);
                            roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
                            roi = Scalar(255, 255, 255);


                            // automatic initialization
                            goodFeaturesToTrack(gray, point_buffer, MAX_COUNT, 0.01, 1, mask, 3, 5, 0, 0.04);
                            //cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);


                            for (auto index:point_buffer) {

                                points[1].push_back(index);

                            }
                        }




                        features_found=true;
                        needToInit=false;
                    }


                    if (features_found) {
                        for (auto q:points[1]) {
                            circle(image, q, 3, Scalar(0, 255, 0), -1, 8);
                            imshow("LK Demo", image);

                        }

                    }


                    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

                    if (!points[1].empty())
                        circle(image, points[1][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

                    imshow("LK Demo", image);

                    char c = (char) waitKey(30);
                    if (c == 27)
                    {
                       pstate=VISUAL_SERVOING;
                        cout<<"Entered Visual Servoing \n";
                        return;

                    }

                    switch (c) {
                        case 'r':
                            needToInit = true;
                            e = 0;
                            cout << "Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            selected_points.clear();
                            sstate=PATH_DRAWING;
                            features_found=false;
                            cout << "Points Cleared \n";
                            break;
                        case 'n':
                            //  nightMode = !nightMode;
                            n++;
                            break;
                    }

                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);

                    break;

                    ///********************************* Finding Features - end **********************************///

                }

                case VISUAL_SERVOING:
                {
                    ///********************************* VISUAL SERVOING **********************************///

                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);

                    if (nightMode)
                        image = Scalar::all(0);

                    if (needToInit) {

                        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                        mask = Mat::zeros(gray.size(), CV_8U);
                        for (auto it:selected_points) {


                            roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
                            roi = Scalar(255, 255, 255);

                        }
                        // automatic initialization
                        goodFeaturesToTrack(gray,point_buffer, MAX_COUNT, 0.01, 10, mask, 3, 3, 0, 0.04);
                        cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);
                        imshow("Mask", mask);

                        for (auto index:point_buffer) {

                            points[1].push_back(index);

                        }



                        addRemovePt = false;
                        features_found = true;
                    }


                    else if (!points[0].empty()) {

                        vector<uchar> status;
                        vector<float> err;
                        if (prevGray.empty())
                            gray.copyTo(prevGray);
                        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                             3, termcrit, 0, 0.001);


                        size_t i, k;
                        for( i = k = 0; i < points[1].size(); i++ )
                        {

                            points[1][k++] = points[1][i];
                            circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
                        }
                        points[1].resize(k);



                    }
                    needToInit = false;

                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);


                    circle(image, points[0][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

                    v.linear.y = (-points[0][n].x + desired_point.x) / 800;
                    v.linear.x = (-points[0][n].y + desired_point.y) / 1000;
                    v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));


                    vel_pub.publish(v);

                    circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

                    imshow("LK Demo", image);


                    first_run = false;



                    if(distance(desired_point,points[0][n]) < 1)
                    {

                        n++;
                        if(n==points[0].size())
                            waitKey();
                    }



                    char c = (char) waitKey(20);
                    if (c == 27)
                        return;

                    switch (c) {

                        case 'r':
                            needToInit = true;
                            pp.clear();
                            e = 0;
                            cout << "Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            n=0;
                            features_found = false;
                            cout << "Points Cleared \n";
                            sstate=PATH_DRAWING;
                            cout<<"Draw a path again \n";
                            break;
                        case 'n':
                            n++;
                    }

                   break;
                    ///********************************* VISUAL SERVOING -end  **********************************///

                }



            }

            break;

            //***************************************************Features on line -end************************************************************************
        }






            /*
             * ***********************************************************************************************************************************
             * This is the case in which we use relative transformation between path points and goal position to track and update the path.
             * The goal point is fixed and its actually a feature that we track and update using optical flow, so overall we only track a single
             * point in the image
             */

        case RELATIVE_TRANSFORMATION:
        {
            switch (pstate)
            {
                case FEATURE_FINDING:
                {
                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);
                    if (nightMode)
                        image = Scalar::all(0);

                    if (needToInit)
                    {
                        // automatic initialization
                        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
                        cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                        addRemovePt = false;
                        features_found=true;
                    } else if (!points[0].empty()) {
                        vector<uchar> status;
                        vector<float> err;
                        if (prevGray.empty())
                            gray.copyTo(prevGray);
                        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                             3, termcrit, 0, 0.001);
                        size_t i, k;

                        {
                            if (addRemovePt) {
                                if (norm(point - points[1][0]) <= 5) {
                                    addRemovePt = false;
                                    return;
                                }
                            }


                            circle(image, points[1].back(), 3, Scalar(0, 200, 0), -1, 8);

                        }




                    }

                    if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
                        vector<Point2f> tmp;
                        tmp.push_back(point);
                        cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
                        points[1].push_back(tmp[0]);
                        addRemovePt = false;
                        features_found=true;
                    }


                    for(auto index:line_points)
                    {

                        circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);
                    }

                    needToInit = false;
                    imshow("LK Demo", image);

                    char c = (char) waitKey(10);
                    if (c == 27)
                        pstate=FIRST_RUN;
                    switch (c) {
                        case 'r':
                            needToInit = true;
                            cout<<"Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            sstate-PATH_DRAWING;
                            cout<<"Points Cleared \n";
                            break;
                        case 'n':
                            nightMode = !nightMode;
                            break;
                    }

                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);


                    break;
                }


                case FIRST_RUN:
                {
                    line_point_size = line_points.size();


                    double distance_current;
                    double distance_previous = 2000000;
                    int nearest_index;
                    if (features_found) {

                        transformation_bw_goal_nf = transformation_calculate(line_points.back(), points[0].back());

                        cout << "\nTFG:" << transformation_bw_goal_nf << "\n";
                    }

                    waitKey();

                    transformation_bw_line.resize(line_points.size());
                    for (int i = 0; i < line_points.size(); i++) {
                        transformation_bw_line[i].push_back(
                                transformation_calculate(line_points[line_points.size() - 1], line_points[i]));

                        pstate=VISUAL_SERVOING;
                    }
                    break;
                }


                case VISUAL_SERVOING:
                {

                   cout<<"IN RELATIVE \n";

                    if(points[0].empty())
                    {
                        cout<<"Yes points are emty \n";
                    }

                    if( frame.empty() )
                        return;

                    frame.copyTo(image);
                    cvtColor(image, gray, COLOR_BGR2GRAY);

                    if (nightMode)
                        image = Scalar::all(0);


                    if (needToInit) {
                        // automatic initialization
                goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
                cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                        addRemovePt = false;
                        features_found=true;
                    } else if (!points[0].empty()) {
                        vector<uchar> status;
                        vector<float> err;
                        if (prevGray.empty())
                            gray.copyTo(prevGray);
                        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                             3, termcrit, 0, 0.001);



                        if (addRemovePt) {
                            if (norm(point - points[1][0]) <= 5) {
                                addRemovePt = false;
                                return;
                            }
                        }


                        circle(image, points[1][0], 3, Scalar(0, 255, 0), -1, 8);
                        circle(image, desired_point, 5, Scalar(0, 150, 0), -1, 8);



                        Mat temp,temp_feature,line;
                        temp.create(3,1,cv::DataType<double>::type) ;
                        temp_feature.create(3,1,cv::DataType<double>::type) ;
                        line.create(3,1,cv::DataType<double>::type) ;

                        temp.at<double>(0,0)=points[1].back().x;
                        temp.at<double>(1,0)=points[1].back().y;
                        temp.at<double>(2,0)=1;


                        cout<<"\nTFG_INV:"<<transformation_bw_goal_nf.inv(DECOMP_LU)<<"\n";
                        cout<<"\ntemp:"<<temp<<"\n";

                        temp_feature=transformation_bw_goal_nf.inv(DECOMP_LU)*temp;

                        for(int i=0;i<transformation_bw_line.size();i++)
                        {
                            line=transformation_bw_line[i]*temp_feature;
                            line_points[i].x=line.at<double>(0,0);
                            line_points[i].y=line.at<double>(1,0);

                        }






                        for(auto index:line_points)
                        {

                            circle(image,Point(index), 5, CV_RGB(255, 255, 0),1,8,0);


//                   line_points=transformed_points;

                        }


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




                    std::swap(points[1], points[0]);
                    cv::swap(prevGray, gray);



                    //////////////////////////////OpenCV part ends here /////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////


                    v.linear.y = (-line_points[n].x + desired_point.x) / 800;
                    v.linear.x = (-line_points[n].y + desired_point.y) / 1000;
                    v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));

                    vel_pub.publish(v);









                    if(distance(desired_point,line_points[n]) < 5)
                    {
                        n++;
                        if(n==line_points.size()-3)
                            return;

                        // needToInit=true;

                    }



                    char c = (char) waitKey(10);
                    if (c == 27)
                        return;
                    switch (c) {
                        case 'r':
                            needToInit = true;
                            cout<<"Re Initialized \n";
                            break;

                        case 'c':
                            points[0].clear();
                            points[1].clear();
                            line_points.clear();
                            line_points_circle.clear();
                            sstate=PATH_DRAWING;
                            cout<<"Points Cleared \n";
                            break;
                        case 'n':
                            nightMode = !nightMode;
                            break;
                    }

                    break;
                }



            }



            break;
        }


            //*********************************************************************************************************************************
    }


}