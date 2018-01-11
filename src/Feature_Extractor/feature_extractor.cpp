//
// Created by zain on 17. 11. 30.
//
/// Right now this programs extract SUF features in ROI drawn by the user
#include "opencv_test/feature_extractor.h"

/// Gandi Programming ke natije
/// Initializing static variables that shouldnt be static

//unity version

bool feature_extractor::drag;
vector<Point> feature_extractor::line_points;
vector<Point> feature_extractor::selected_points;
bool feature_extractor::first_run=true;
int feature_extractor::circle_radius=2;
int feature_extractor::width=16;
int feature_extractor:: height=16;
int feature_extractor:: minHessian=400;
Point feature_extractor::p;
vector<Point> feature_extractor::line_points_circle;
bool feature_extractor::start;
Mat feature_extractor::img;
Mat feature_extractor::gray;
Mat feature_extractor::prevGray;
Mat feature_extractor::main_disp;
vector<Point2f> feature_extractor::points[2];
Point2f feature_extractor::point;
bool feature_extractor::addRemovePt=false;
bool feature_extractor::LK_selector=false;
bool feature_extractor::SURF_selector=false;
Size subPixWinSize(10,10), winSize(31,31);
// Kernel size for Subpixel and normal keypoint/corner search
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);


feature_extractor::feature_extractor() :  it(nh) {

    // ROS related
    image_sub = it.subscribe("image_raw", 1,
                             &feature_extractor::imageCb, this);
    image_pub = it.advertise("/image_converter/output_video", 1);

    RNG rng(12345);
    color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

    Surf_detector=SURF::create(feature_extractor::minHessian);

    cout<<"SURF Feature Detector \n";




}


void feature_extractor::points_selector(vector<Point> &all_points){


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
        if(distance>16)
        {

            selected_points.push_back(i);
            p=i;
        }

    }
    line_points.clear();

}


void feature_extractor::CallBackFunc(int event, int x, int y, int flags, void* userdata)
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
    else if  ( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;

    }
    else if ( event == EVENT_MOUSEMOVE && drag)
    {
        line_points.push_back(CvPoint(x,y));
        line_points_circle.push_back(CvPoint(x,y));
        draw_circle(img,line_points_circle);
        points_selector(line_points);


        for(auto g:selected_points)
        {
            rectangle(img,Point(g.x- width/2,g.y-height/2),Point(g.x+width/2,g.y+height/2),CV_RGB(255, 0, 0),1,8,0);

        }

        imshow("Callback Window",img);
        waitKey(1);

    }
    else if(event ==EVENT_LBUTTONUP)
    {
        drag=false;
    }
}





void feature_extractor::draw_circle(Mat &imgg,vector<Point> &lp){
    for(auto index:lp)
    {
        circle(imgg,Point(index), circle_radius, CV_RGB(255, 0, 0),1,8,0);
    }

}



void feature_extractor::find_Surf_features(Mat &im) {


    mask=Mat::zeros(im.size(),CV_8U);
    surf_keypoints.resize(selected_points.size());
    surf_descriptor.resize((selected_points.size()));
    auto r=surf_keypoints.begin();
    auto s=surf_descriptor.begin();

    for(auto it:selected_points)
    {

        roi=Mat(mask,Rect(it.x-width/2,it.y-height/2,width,height));
        roi= Scalar(255, 255, 255);
        Surf_detector->detectAndCompute(im,mask,*r,*s);
        cout<<"I am within SURF "<<endl;
        r++;
        s++;

    }

    for(auto k:surf_keypoints)
    {
        concatenated_keypoints.insert(concatenated_keypoints.end(),k.begin(),k.end());
        drawKeypoints(im, concatenated_keypoints, img_surf_final, Scalar::all(-1), DrawMatchesFlags::DEFAULT);


    }

    for(auto g:selected_points)
    {
        rectangle(img_surf_final,Point(g.x- width/2,g.y-height/2),Point(g.x+width/2,g.y+height/2),CV_RGB(255, 0, 0),1,8,0);

    }

    imshow("SURF_Points",img_surf_final);

    SURF_selector=false;                 // setting false so that it only detects when user press s
    concatenated_keypoints.clear();
    surf_descriptor.clear();
    surf_keypoints.clear();

}


void feature_extractor::imageCb(const sensor_msgs::ImageConstPtr& msg) {




    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

       img=cv_ptr->image;


        char c = (char)waitKey(10);
        if( c == 27 )
            return;
        switch (c)
        {

            case 'r' :
                feature_extractor::selected_points.clear();
                feature_extractor::line_points.clear();
                feature_extractor::line_points_circle.clear();
                concatenated_keypoints.clear();
                surf_descriptor.clear();
                surf_keypoints.clear();
                SURF_selector=false;

                cout<<"All previous points have been cleared"<<endl
                    <<"Press any key to start again"<<endl;

            case 's':
                SURF_selector=true;


        }

    namedWindow("My Window",WINDOW_AUTOSIZE);
    setMouseCallback("My Window", CallBackFunc,NULL);






    if(SURF_selector)
    {
        find_Surf_features(cv_ptr->image);


    }

            imshow("My Window",cv_ptr->image);
            waitKey(1);

}


void feature_extractor::find_GoodFeatures(Mat &im){

    cvtColor(img, gray, COLOR_BGR2GRAY);
    cout<<"I am within LK "<<endl;
    if( needToInit )
    {
        // automatic initialization
        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        addRemovePt = false;
    }
    else if( !points[0].empty() )
    {
        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            gray.copyTo(prevGray);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                             3, termcrit, 0, 0.001);
        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {
            if( addRemovePt )
            {
                if( norm(point - points[1][i]) <= 5 )
                {
                    addRemovePt = false;
                    continue;
                }
            }

            if( !status[i] )
                continue;

            points[1][k++] = points[1][i];
            circle( im, points[1][i], 3, Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);
    }

    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
    {
        vector<Point2f> tmp;
        tmp.push_back(point);
        cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
        points[1].push_back(tmp[0]);
        addRemovePt = false;
    }

    needToInit = false;
    imshow("LK Demo", im);

    char t = (char)waitKey(10);
    if( t == 27 )
        return;
    switch( t )
    {
        case 'z':
            needToInit = true;
        case 'n':
            nightMode = !nightMode;
            break;
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);

}