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



    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    image_sub = it.subscribe("image_raw", 1,
                             &Shared_Autonomy_Vine::imageCb, this);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    desired_point = Point(320, 475);

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
//        if(distance>(2*circle_radius*25))
        if(distance>5)
        {

            selected_points.push_back(i);
            cout<<"Point pushed \n";
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
    /// OpenCV part starts here that finds Good features and uses Shi-Tomasi to track ///
    ///////////////////////////////////////////////////////////////////////////////////

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame=cv_ptr->image;




    if (!path_drawn) {



        cout<<"First Part \n";

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
            path_drawn=true;

        }

        switch (c) {
            case 'c':
                line_points.clear();
                points[0].clear();
                points[1].clear();
                selected_points.clear();
                path_drawn= false;
                break;
        }

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);
        imshow("LK Demo", image);
        v.linear.y = 0;
        v.linear.x = 0;
        v.angular.z= 0;

        vel_pub.publish(v);

    }



    if (path_drawn  && !features_found ) {



        cout << "\n ****************************************\n"
             << "\nFinding Features on Drawn Path\n"
             << "\n*****************************************\n";
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
//                cornerSubPix(gray, point_buffer, subPixWinSize, Size(-1, -1), termcrit);


                for (auto index:point_buffer) {

                    points[1].push_back(index);

                }
            }


//            mask = Mat::zeros(gray.size(), CV_8U);
//            for (auto it:selected_points) {
//
//                roi = Mat(mask, Rect(it.x - width / 2, it.y - height / 2, width, height));
//                roi = Scalar(255, 255, 255);
//            }
//            imshow("Mask", mask);


            features_found = true;
            needToInit=false;
        }


        if (features_found) {
            for (auto q:points[1]) {
                circle(image, q, 3, Scalar(0, 255, 0), -1, 8);
                imshow("LK Demo", image);

            }

        }

//        for (auto index:line_points) {
//
////            circle(image, Point(index), 8, CV_RGB(255, 255, 0), 0.5, 8, 0);
////        }

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        if (!points[1].empty())
            circle(image, points[1][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

        imshow("LK Demo", image);

        char c = (char) waitKey(30);
        if (c == 27)
        {
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
                features_found = false;
                cout << "Points Cleared \n";
                path_drawn= false;
                feature_on_path_found=false;
                break;
            case 'n':
                //  nightMode = !nightMode;
                n++;
                break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);

    }


    //////////////////////////////OpenCV part ends here /////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////


    if(path_drawn && features_found) {



        cout << "\n**********************************\n"
             << "\n Visual Servoing Loop has started\n"
             << "\n**********************************\n";







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


        //////////////////////////////OpenCV part ends here /////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////

//        vpColVector v(2);
//
//

//
//
//

        circle(image, points[0][n], 10, CV_RGB(255, 255, 0), 1, 8, 0);

        v.linear.y = (-points[0][n].x + desired_point.x) / 800;
        v.linear.x = (-points[0][n].y + desired_point.y) / 1000;
        v.angular.z= (atan2(v.linear.y*800,v.linear.x*1000));

//        v.angular.z=1;
        vel_pub.publish(v);

        cout << "Angular Velocity: " << v.angular.z << endl;

        circle(image, desired_point, 15, Scalar(0, 150, 0), -1, 8);

        imshow("LK Demo", image);

        //   robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

//        cout << "n: " << n << endl;
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
                path_drawn= false;
                feature_on_path_found=false;
                break;
            case 'n':
                n++;
        }

    }
}