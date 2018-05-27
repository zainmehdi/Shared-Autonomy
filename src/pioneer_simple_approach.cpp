//
// Created by zain on 17. 12. 17.
//

// Program that uses images coming from unity

#include <opencv_test/pioneer_simple_approach.h>


/// Callback function for incoming images

void imageCb(const sensor_msgs::ImageConstPtr& msg1, const geometry_msgs::PoseStampedConstPtr &msg2)
{



    acquire_image(msg1);

   // This part is not needed. I am operating in two different coordinate frames
    // Unity and opencv / image coordinates. The angle feedback is from unity
    // while angle calculation is in image coordinates. They cannot be used to calculate
    // error term/

    // Update: I doubt my above comments after few weeks. Its still vague to me
    // but it somehow seems to work.

    // I am using rotation values from unity to compensate for desired point
    // as the robot rotates the desired point also has to rotate along with it.

   acquire_pose(msg2);   // value of rotation will be stored in "yaw" variable

 //   cout<<" Yaw from Unity test :";
 //   printf("%.3f \n",pitch);


//     Allows drawing a ROI for KCF Tracker

//     if(!tracker_ROI)
//     {
//
//         select_tracker_ROI();
//     }

    if (!path_drawn ){
    //if (!path_drawn && tracker_ROI) {

//        cout << "\n ****************************************\n"
//             << "\nDrawing path on the image\n"
//             << "\n*****************************************\n";

        path_not_yet_drawn();


    }



    if (path_drawn  && !feature_on_path_found ) {

//
//        cout << "\n ****************************************\n"
//             << "\nFinding Features on Drawn Path\n"
//             << "\n*****************************************\n";

        path_drawn_features_nishta();
        arc_direction (selected_points);
    }




    if(path_drawn && feature_on_path_found) {



//           cout << "\n**********************************\n"
//                << "\n Visual Servoing Loop has started\n"
//                << "\n**********************************\n";


            visual_servo();

    }

    ROS_INFO("I heard: [%s]", decision.data.c_str());

}


void python_callback(std_msgs::StringConstPtr msg)
{
    decision.data=msg->data;

//    if(decision->data == "left_left_sequence")
//        cout<<" ////////////////////////////////////////"
//                "///////// Message read ///////////////"
//                "//////////////////////////////////////";

//    ROS_INFO("I heard: [%s]", msg->data.c_str());

}


int main(int argc, char **argv) {


    ros::init(argc, argv, "unity_autonomy");
    ros::NodeHandle nh;
    desired_point = Point(320, 475);
    image_transport::ImageTransport it(nh);

    tracker = TrackerKCF::create();

//    Surf_detector=SURF::create(400);
    image_transport::SubscriberFilter image_sub(it,"image_raw",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh,"pose",1);
    TimeSynchronizer<sensor_msgs::Image,geometry_msgs::PoseStamped> sync(image_sub,pose_sub,10);
    sync.registerCallback(boost::bind(&imageCb,_1,_2));
    vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    mask_pub=nh.advertise<sensor_msgs::Image>("mask",1000);
    ros::Subscriber python_msg = nh.subscribe("result", 1000, python_callback);
    // image_sub = it.subscribe("image_raw", 1,imageCb);
    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );
    ros::spin();
    return 0;
}